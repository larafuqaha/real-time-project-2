/*****************************************************************************
 *  controller.c
 *  ---------------------------------------------------------------------------
 *  Intersection Control Logic process.
 *
 *  Responsibilities
 *  ----------------
 *  - Owns the traffic-phase state machine (NS_GREEN -> NS_YELLOW -> ALL_RED
 *    -> EW_GREEN -> EW_YELLOW -> ALL_RED -> ...).
 *  - Reads events from the EVENT queue (vehicles, pedestrians, emergencies,
 *    faults) and adjusts behaviour accordingly.
 *  - Sends commands to each traffic-light process telling it which color to
 *    display; collects acknowledgements.
 *  - Continuously enforces safety invariants on the shared blackboard.
 *  - Performs adaptive timing: extends green if the cross-direction is empty,
 *    serves pedestrians within their max wait time, etc.
 *
 *  Pedestrian model
 *  ----------------
 *  There is NO dedicated all-red pedestrian phase. Pedestrians cross
 *  concurrently with the perpendicular vehicle green: when NS vehicles are
 *  green (E/W signals red), the east and west crosswalks are open; when EW
 *  vehicles are green (N/S signals red), the north and south crosswalks
 *  are open.
 *
 *  Emergency model
 *  ---------------
 *  Two-pass sequence: all signals (including the emergency street) go RED
 *  first while cross-traffic clears; then the emergency street goes GREEN
 *  so the vehicle can pass through.
 *
 *  Compile: see Makefile
 *****************************************************************************/
#include "common.h"
#include "ipc_init.h"

static int shmid = -1, semid = -1, qcmd = -1, qevt = -1, qlog = -1;
static shared_state_t *S = NULL;
static int next_cmd_id = 1;

/* ---------- helpers --------------------------------------------------- */

static void send_command(direction_t d, light_color_t c)
{
    /* If this light has been marked faulted by a GUI-injected fault,
     * refuse to send commands to it.  The light stays dark in shared
     * state, which is what the GUI renders.  Other directions keep
     * cycling normally. */
    sem_lock(semid);
    int faulted = S->light_faulted[d];
    sem_unlock(semid);
    if (faulted) return;

    cmd_msg_t m;
    memset(&m, 0, sizeof(m));
    m.mtype        = MTYPE_CMD_BASE + d + 1;   /* +1 because mtype>0 */
    m.direction    = d;
    m.target_color = c;
    m.timestamp    = time(NULL);
    m.cmd_id       = next_cmd_id++;
    if (msgsnd(qcmd, &m, sizeof(m) - sizeof(long), 0) < 0) {
        send_log(qlog, "CTRL", 2,
                 "Failed to send command %s -> %s",
                 DIR_NAMES[d], COLOR_NAMES[c]);
    }
}

/* Block (briefly) waiting for the ack of the most recent command for d.   */
static int wait_for_ack(direction_t d, int timeout_sec) __attribute__((unused));
static int wait_for_ack(direction_t d, int timeout_sec)
{
    ack_msg_t a;
    time_t start = time(NULL);
    while (time(NULL) - start < timeout_sec) {
        ssize_t r = msgrcv(qcmd, &a, sizeof(a) - sizeof(long),
                           MTYPE_ACK_BASE + d + 1, IPC_NOWAIT);
        if (r > 0) return a.success;
        usleep(50 * 1000);
    }
    return 0;
}

/* Set all conflicting directions to RED, then return.                     */
static void set_all_red(void) __attribute__((unused));
static void set_all_red(void)
{
    for (int i = 0; i < NUM_DIRECTIONS; ++i)
        send_command((direction_t)i, LIGHT_RED);
    /* small grace period for lights to flip */
    sleep(1);
}

/* Update shared phase + remaining time atomically                         */
static void set_phase(phase_t p, int duration)
{
    sem_lock(semid);
    S->current_phase    = p;
    S->phase_started    = time(NULL);
    S->phase_remaining  = duration;
    sem_unlock(semid);
    send_log(qlog, "CTRL", 0, "Phase -> %s (duration=%ds)",
             PHASE_NAMES[p], duration);
}

/* Read flags atomically into out-params */
static void read_flags(int *ped_pending_any, direction_t *ped_dir,
                       int *emerg_active,    direction_t *emerg_dir,
                       int *waiting_NS,      int *waiting_EW)
{
    sem_lock(semid);
    *ped_pending_any = 0;
    *ped_dir         = DIR_NORTH;
    for (int i = 0; i < NUM_DIRECTIONS; ++i) {
        if (S->pedestrian_pending[i]) {
            *ped_pending_any = 1;
            *ped_dir = (direction_t)i;
            break;
        }
    }
    *emerg_active = S->emergency_active;
    *emerg_dir    = S->emergency_direction;
    *waiting_NS   = S->waiting_vehicles[DIR_NORTH] +
                    S->waiting_vehicles[DIR_SOUTH];
    *waiting_EW   = S->waiting_vehicles[DIR_EAST] +
                    S->waiting_vehicles[DIR_WEST];
    sem_unlock(semid);
}

/* Read current emergency_active flag from shared state. Used by the main
 * loop between phases to detect a preemption that occurred during the
 * previous phase (e.g., an emergency event interrupted execute_ns_green
 * causing it to break early; the local em_active variable from the last
 * read_flags() call is now stale). */
static int check_emergency(void)
{
    sem_lock(semid);
    int e = S->emergency_active;
    sem_unlock(semid);
    return e;
}

/* Read current ticket_active flag.  A ticket fires when the GUI reports a
 * red-light runner; the controller freezes its state machine until the
 * GUI signals "police caught the runner" (EVT_TICKET_DONE). */
static int check_ticket(void)
{
    sem_lock(semid);
    int t = S->ticket_active;
    sem_unlock(semid);
    return t;
}

/* Forward decls for the responsive sleep helper. */
static void drain_events(void);

/* Sleep up to `seconds` total in 100 ms ticks, draining events each tick.
 * Returns 1 immediately if an emergency arrives during the wait, 0 if
 * the full duration elapses.  This is what gives mid-green emergency
 * preemption near-instant response (≤100 ms) instead of up-to-1-second
 * latency. */
static int sleep_emergency_aware(int seconds)
{
    int ticks = seconds * 10;
    for (int i = 0; i < ticks; ++i) {
        drain_events();
        sem_lock(semid);
        int em = S->emergency_active;
        sem_unlock(semid);
        if (em) return 1;
        usleep(100 * 1000);
    }
    return 0;
}

/* ---------- event draining ------------------------------------------- */
static void drain_events(void)
{
    evt_msg_t e;
    /* Non-blocking pull until queue is empty */
    while (msgrcv(qevt, &e, sizeof(e) - sizeof(long), 0, IPC_NOWAIT) > 0) {
        sem_lock(semid);
        switch (e.mtype) {
        case MTYPE_EVT_VEHICLE:
            if (e.direction >= 0 && e.direction < NUM_DIRECTIONS) {
                S->waiting_vehicles[e.direction] += e.count;
                if (S->waiting_vehicles[e.direction] > MAX_QUEUE_VEHICLES)
                    S->waiting_vehicles[e.direction] = MAX_QUEUE_VEHICLES;
            }
            send_log(qlog, "CTRL", 0, "+%d vehicle(s) %s (queue=%d)",
                     e.count, DIR_NAMES[e.direction],
                     S->waiting_vehicles[e.direction]);
            break;
        case MTYPE_EVT_PEDESTRIAN:
            /* 0/1 flag: set to 1 on first request, cleared to 0 the
             * moment the crossing opens (in ped_walk_open). */
            if (!S->pedestrian_pending[e.direction]) {
                S->pedestrian_pending[e.direction]      = 1;
                S->pedestrian_request_time[e.direction] = time(NULL);
                send_log(qlog, "CTRL", 0,
                         "Pedestrian request from %s", DIR_NAMES[e.direction]);
            }
            break;
        case MTYPE_EVT_EMERGENCY:
            if (!S->emergency_active) {
                S->emergency_active    = 1;
                S->emergency_direction = e.direction;
                S->emergency_started   = time(NULL);
                S->total_emergencies++;
                send_log(qlog, "CTRL", 1,
                         "EMERGENCY vehicle from %s", DIR_NAMES[e.direction]);
            }
            break;
        case MTYPE_EVT_FAULT:
            send_log(qlog, "CTRL", 2, "FAULT reported by %s: %s",
                     DIR_NAMES[e.direction], e.info);
            break;
        case MTYPE_EVT_TICKET:
            /* Red-light-running car reported by the GUI.  Freeze the
             * state machine: hold current lights, do not advance phases,
             * do not serve any new vehicle/ped/emergency events while
             * the chase is in progress. */
            if (!S->ticket_active) {
                S->ticket_active    = 1;
                S->ticket_direction = e.direction;
                strncpy(S->ticket_plate, e.info, sizeof(S->ticket_plate) - 1);
                S->ticket_plate[sizeof(S->ticket_plate) - 1] = '\0';
                S->total_tickets++;
                send_log(qlog, "CTRL", 1,
                         "TICKET: red-light violation from %s, plate %s",
                         DIR_NAMES[e.direction], S->ticket_plate);
            }
            break;
        case MTYPE_EVT_TICKET_DONE:
            /* Police caught the runner — resume normal operation. */
            if (S->ticket_active) {
                S->ticket_active   = 0;
                S->ticket_plate[0] = '\0';
                send_log(qlog, "CTRL", 1,
                         "TICKET: chase complete, resuming normal operation");
            }
            break;
        case MTYPE_EVT_FAULT_INJECT:
            /* GUI requested a fault on this direction.  Mark the light
             * as faulted, force the shared light color to OFF (treated
             * as RED here so no one rolls through), and bump the
             * safety_violations counter for HUD bookkeeping.            */
            if (e.direction >= 0 && e.direction < NUM_DIRECTIONS &&
                !S->light_faulted[e.direction]) {
                S->light_faulted[e.direction] = 1;
                S->light[e.direction] = LIGHT_RED;
                S->safety_violations++;
                send_log(qlog, "CTRL", 2,
                         "FAULT injected on %s — light shut down, police dispatched",
                         DIR_NAMES[e.direction]);
            }
            break;
        case MTYPE_EVT_FAULT_CLEAR:
            /* GUI cleared a fault.  Light returns to normal control on
             * the next phase command sent by the controller.            */
            if (e.direction >= 0 && e.direction < NUM_DIRECTIONS &&
                S->light_faulted[e.direction]) {
                S->light_faulted[e.direction] = 0;
                send_log(qlog, "CTRL", 1,
                         "FAULT cleared on %s — light restored",
                         DIR_NAMES[e.direction]);
            }
            break;
        }
        sem_unlock(semid);
    }
}

/* While a ticket is active, drain events but make no progress.  Returns
 * when the GUI sends EVT_TICKET_DONE.  Called at safe points where the
 * lights are stable (start of main loop, between phase executors, and
 * inside the green loops). */
static void wait_for_ticket_done(void)
{
    if (!check_ticket()) return;
    send_log(qlog, "CTRL", 1, "TICKET: freezing simulation until chase ends");
    while (check_ticket()) {
        drain_events();
        sem_lock(semid);
        int running = S->running;
        sem_unlock(semid);
        if (!running) return;
        usleep(100 * 1000);
    }
}

/* ---------- phase executors ----------------------------------------- */

/* Synchronized pedestrian crossing.
 *
 * The dedicated all-red PHASE_PEDESTRIAN phase has been removed. Instead,
 * pedestrians cross concurrently with the perpendicular vehicle green
 * phase: when N-S vehicles have GREEN (so E and W vehicle signals are
 * RED), pedestrians on the east and west sides may walk; symmetrically
 * for the E-W vehicle green phase.
 *
 * The two helpers below open/close the walk signal for the pair of
 * crosswalks that are safe given which vehicle axis currently holds the
 * green, and clear the matching pending requests + counters so that the
 * controller no longer needs a separate ped phase.
 */
static void ped_walk_open(direction_t a, direction_t b)
{
    sem_lock(semid);
    /* Never open a pedestrian crossing while an emergency is in progress. */
    if (S->emergency_active) {
        sem_unlock(semid);
        return;
    }
    /* Only activate if at least one pedestrian is actually waiting on this
     * axis — avoids the banner firing every green phase even with no one
     * waiting. */
    int any = S->pedestrian_pending[a] || S->pedestrian_pending[b];
    if (!any) {
        sem_unlock(semid);
        return;
    }
    S->pedestrian_active    = 1;
    S->pedestrian_remaining = S->t_pedestrian;   /* will count down in green loop */
    /* Clear the 0/1 flag immediately — pedestrians start crossing now. */
    if (S->pedestrian_pending[a]) {
        S->pedestrian_pending[a] = 0;
        S->total_pedestrians_served++;
    }
    if (S->pedestrian_pending[b]) {
        S->pedestrian_pending[b] = 0;
        S->total_pedestrians_served++;
    }
    sem_unlock(semid);
    send_log(qlog, "CTRL", 0,
             "Pedestrian WALK: %s/%s crosswalks (synchronized with perpendicular vehicle green)",
             DIR_NAMES[a], DIR_NAMES[b]);
}

static void ped_walk_close(void)
{
    sem_lock(semid);
    S->pedestrian_active    = 0;
    S->pedestrian_remaining = 0;
    sem_unlock(semid);
    send_log(qlog, "CTRL", 0, "Pedestrian WALK closed");
}

/* Run NS green: send GREEN to N,S and RED to E,W                         */
static void execute_ns_green(int duration)
{
    /* Pre-green yellow: lights N,S currently RED (from the previous all-red
     * phase) must not jump straight to GREEN.  Flash YELLOW first so the
     * sequence is RED -> YELLOW -> GREEN, then drive the perpendicular
     * directions to RED (they already are, this is a safe re-issue). */
    send_command(DIR_NORTH, LIGHT_YELLOW);
    send_command(DIR_SOUTH, LIGHT_YELLOW);
    sleep(S->t_yellow);

    send_command(DIR_NORTH, LIGHT_GREEN);
    send_command(DIR_SOUTH, LIGHT_GREEN);
    send_command(DIR_EAST,  LIGHT_RED);
    send_command(DIR_WEST,  LIGHT_RED);
    set_phase(PHASE_NS_GREEN, duration);

    /* Synchronized pedestrian crossing: EW vehicles are red, so east and
     * west crosswalks are safe to open in parallel with this phase. */
    ped_walk_open(DIR_EAST, DIR_WEST);

    time_t start = time(NULL);
    while (time(NULL) - start < duration) {
        drain_events();

        sem_lock(semid);
        if (S->waiting_vehicles[DIR_NORTH] > 0) S->waiting_vehicles[DIR_NORTH]--;
        if (S->waiting_vehicles[DIR_SOUTH] > 0) S->waiting_vehicles[DIR_SOUTH]--;
        S->total_vehicles_served +=
            (S->waiting_vehicles[DIR_NORTH] > 0 ? 1 : 0) +
            (S->waiting_vehicles[DIR_SOUTH] > 0 ? 1 : 0);
        S->phase_remaining = duration - (int)(time(NULL) - start);
        /* Tick the pedestrian countdown while the walk is active. */
        if (S->pedestrian_active && S->pedestrian_remaining > 0)
            S->pedestrian_remaining--;
        int ped_done = (S->pedestrian_active && S->pedestrian_remaining == 0);

        int emerg = S->emergency_active;
        int now = time(NULL);
        int ped_timeout = 0;
        for (int i = 0; i < NUM_DIRECTIONS; ++i) {
            if (S->pedestrian_pending[i] &&
                now - S->pedestrian_request_time[i] > S->t_pedestrian_max_wait) {
                ped_timeout = 1; break;
            }
        }
        sem_unlock(semid);

        if (emerg) {
            ped_walk_close();   /* stop crossing immediately on emergency */
            break;
        }

        /* Walk timer expired but the vehicle green continues. */
        if (ped_done) ped_walk_close();

        if (ped_timeout && time(NULL) - start >= S->t_green_min) {
            send_log(qlog, "CTRL", 1,
                     "TIMING VIOLATION: pedestrian waited > %ds — cutting NS-GREEN short",
                     S->t_pedestrian_max_wait);
            break;
        }

        sem_lock(semid);
        int ew_waiting = S->waiting_vehicles[DIR_EAST] + S->waiting_vehicles[DIR_WEST];
        int ew_starved = (ew_waiting > 0 &&
                         time(NULL) - start >= S->t_vehicle_max_wait);
        sem_unlock(semid);

        if (ew_starved && time(NULL) - start >= S->t_green_min) {
            send_log(qlog, "CTRL", 1,
                     "TIMING VIOLATION: EW starved > %ds — cutting NS-GREEN short",
                     S->t_vehicle_max_wait);
            break;
        }

        /* Ticket freeze: if a red-light runner was reported, hold here
         * until the GUI signals the chase is over.  Add the time spent
         * frozen back to `start` so the green phase still gets its full
         * intended wall-clock duration of vehicle service. */
        if (check_ticket()) {
            time_t pause_start = time(NULL);
            wait_for_ticket_done();
            start += time(NULL) - pause_start;
        }

        /* Emergency-aware tick: returns immediately on emergency so the
         * preemption is near-instant (≤100 ms) instead of the up-to-1-s
         * latency a plain sleep(1) would produce. */
        if (sleep_emergency_aware(1)) {
            ped_walk_close();
            break;
        }
    }

    /* Close the synchronized pedestrian crossing before yielding the
     * intersection to the EW vehicle phase. */
    ped_walk_close();
}

static void execute_ew_green(int duration)
{
    /* Pre-green yellow: lights E,W currently RED must not jump straight
     * to GREEN.  Sequence: RED -> YELLOW -> GREEN. */
    send_command(DIR_EAST,  LIGHT_YELLOW);
    send_command(DIR_WEST,  LIGHT_YELLOW);
    sleep(S->t_yellow);

    send_command(DIR_EAST,  LIGHT_GREEN);
    send_command(DIR_WEST,  LIGHT_GREEN);
    send_command(DIR_NORTH, LIGHT_RED);
    send_command(DIR_SOUTH, LIGHT_RED);
    set_phase(PHASE_EW_GREEN, duration);

    /* Synchronized pedestrian crossing: NS vehicles are red, so north and
     * south crosswalks are safe to open in parallel with this phase. */
    ped_walk_open(DIR_NORTH, DIR_SOUTH);

    time_t start = time(NULL);
    while (time(NULL) - start < duration) {
        drain_events();

        sem_lock(semid);
        if (S->waiting_vehicles[DIR_EAST] > 0) S->waiting_vehicles[DIR_EAST]--;
        if (S->waiting_vehicles[DIR_WEST] > 0) S->waiting_vehicles[DIR_WEST]--;
        S->total_vehicles_served +=
            (S->waiting_vehicles[DIR_EAST] > 0 ? 1 : 0) +
            (S->waiting_vehicles[DIR_WEST] > 0 ? 1 : 0);
        S->phase_remaining = duration - (int)(time(NULL) - start);
        /* Tick the pedestrian countdown while the walk is active. */
        if (S->pedestrian_active && S->pedestrian_remaining > 0)
            S->pedestrian_remaining--;
        int ped_done = (S->pedestrian_active && S->pedestrian_remaining == 0);

        int emerg = S->emergency_active;
        int now = time(NULL);
        int ped_timeout = 0;
        for (int i = 0; i < NUM_DIRECTIONS; ++i) {
            if (S->pedestrian_pending[i] &&
                now - S->pedestrian_request_time[i] > S->t_pedestrian_max_wait) {
                ped_timeout = 1; break;
            }
        }
        sem_unlock(semid);

        if (emerg) {
            ped_walk_close();   /* stop crossing immediately on emergency */
            break;
        }

        /* Walk timer expired but the vehicle green continues. */
        if (ped_done) ped_walk_close();

        if (ped_timeout && time(NULL) - start >= S->t_green_min) {
            send_log(qlog, "CTRL", 1,
                     "TIMING VIOLATION: pedestrian waited > %ds — cutting EW-GREEN short",
                     S->t_pedestrian_max_wait);
            break;
        }

        sem_lock(semid);
        int ns_waiting = S->waiting_vehicles[DIR_NORTH] + S->waiting_vehicles[DIR_SOUTH];
        int ns_starved = (ns_waiting > 0 &&
                         time(NULL) - start >= S->t_vehicle_max_wait);
        sem_unlock(semid);

        if (ns_starved && time(NULL) - start >= S->t_green_min) {
            send_log(qlog, "CTRL", 1,
                     "TIMING VIOLATION: NS starved > %ds — cutting EW-GREEN short",
                     S->t_vehicle_max_wait);
            break;
        }

        /* Ticket freeze. */
        if (check_ticket()) {
            time_t pause_start = time(NULL);
            wait_for_ticket_done();
            start += time(NULL) - pause_start;
        }

        /* Emergency-aware tick. */
        if (sleep_emergency_aware(1)) {
            ped_walk_close();
            break;
        }
    }

    /* Close the synchronized pedestrian crossing before yielding the
     * intersection to the NS vehicle phase. */
    ped_walk_close();
}

static void execute_yellow(direction_t a, direction_t b, phase_t p)
{
    send_command(a, LIGHT_YELLOW);
    send_command(b, LIGHT_YELLOW);
    set_phase(p, S->t_yellow);
    send_log(qlog, "CTRL", 0,
             "TIMING: yellow phase started — must last exactly %ds", S->t_yellow);
    for (int i = 0; i < S->t_yellow; ++i) {
        drain_events();
        sem_lock(semid);
        S->phase_remaining = S->t_yellow - i;
        int emerg = S->emergency_active;
        sem_unlock(semid);
        if (emerg) break;   /* execute_emergency() will issue its own
                             * yellow+red clearance; we can hand off early. */
        sleep(1);
    }
}
static void execute_all_red(phase_t p)
{
    for (int i = 0; i < NUM_DIRECTIONS; ++i)
        send_command((direction_t)i, LIGHT_RED);
    set_phase(p, S->t_all_red);
    for (int i = 0; i < S->t_all_red; ++i) {
        drain_events();
        sem_lock(semid);
        S->phase_remaining = S->t_all_red - i;
        int emerg = S->emergency_active;
        sem_unlock(semid);
        if (emerg) break;   /* let the main loop preempt to emergency. */
        sleep(1);
    }
}

/* Note: the dedicated execute_pedestrian() (all-red walk phase) has been
 * removed. Pedestrians are now served inside execute_ns_green() and
 * execute_ew_green() via the ped_walk_open()/ped_walk_close() helpers,
 * which open the crosswalks perpendicular to the active vehicle axis.
 */

static void execute_emergency(void)
{
    direction_t ed;
    sem_lock(semid);
    ed = S->emergency_direction;
    sem_unlock(semid);

    /* Ambulance coming FROM ed travels TOWARD the opposite side.
       Give green to the light FACING the ambulance.                  */
    direction_t green_dir;
    switch (ed) {
        case DIR_NORTH: green_dir = DIR_SOUTH; break;
        case DIR_SOUTH: green_dir = DIR_NORTH; break;
        case DIR_EAST:  green_dir = DIR_EAST;  break;  /* no flip for E/W */
        case DIR_WEST:  green_dir = DIR_WEST;  break;  /* no flip for E/W */
        default:        green_dir = ed;        break;
    }

    send_log(qlog, "CTRL", 1, "EMERGENCY priority for %s — clearing intersection",
             DIR_NAMES[ed]);

    /* If a pedestrian walk was active when the emergency preempted the
     * green phase, force it closed immediately. */
    ped_walk_close();

    /* Emergency timings are deliberately MUCH shorter than the normal
     * clearance: peds are gone, vehicles need to get out of the way, and
     * the user wants the corridor to open ASAP.  Use 500 ms per safety
     * step (yellow, all-red, yellow) instead of the full configured
     * t_yellow/t_all_red, giving ~1.5 s from emergency event to green
     * corridor instead of the previous ~3-6 s. */
    const useconds_t EMERG_STEP_US = 500 * 1000;   /* 0.5 s */

    /* Step 1 — safe clearance: drive every direction (including the
     * emergency street itself) to yellow, then ALL-RED.  The emergency
     * vehicle is still approaching; it must see RED while cross-traffic
     * clears.  This is the "first pass" the user requires. */
    for (int i = 0; i < NUM_DIRECTIONS; ++i) {
        sem_lock(semid);
        light_color_t c = S->light[i];
        sem_unlock(semid);
        if (c == LIGHT_GREEN) send_command((direction_t)i, LIGHT_YELLOW);
    }
    usleep(EMERG_STEP_US);
    for (int i = 0; i < NUM_DIRECTIONS; ++i)
        send_command((direction_t)i, LIGHT_RED);
    usleep(EMERG_STEP_US);

    /* Step 2 — corridor open: emergency direction goes RED -> YELLOW ->
     * GREEN ("second pass").  Same no-jumping-red-to-green rule applies
     * to the emergency corridor. */
    send_command(green_dir, LIGHT_YELLOW);
    usleep(EMERG_STEP_US);
    send_command(green_dir, LIGHT_GREEN);
    set_phase(PHASE_EMERGENCY, S->t_green_min);

    /* Hold until emergency cleared OR min green satisfied                */
    time_t start = time(NULL);
    while (1) {
        drain_events();
        sem_lock(semid);
        int still = S->emergency_active;
        time_t passed = time(NULL) - start;
        if (passed < S->t_green_min) still = 1;
        S->phase_remaining = (S->t_green_min - passed > 0) ?
                             (int)(S->t_green_min - passed) : 0;
        sem_unlock(semid);
        if (!still) break;

        if (passed > (time_t)(S->t_emergency_max_hold)) {
            sem_lock(semid);
            S->emergency_active = 0;
            sem_unlock(semid);
            break;
        }
        sleep(1);
    }

    /* Safe exit: yellow then red — also use the fast step so we don't
     * tail-pad emergency handling with several seconds of all-red. */
    send_command(green_dir, LIGHT_YELLOW);
    usleep(EMERG_STEP_US);
    send_command(green_dir, LIGHT_RED);
    usleep(EMERG_STEP_US);

    send_log(qlog, "CTRL", 1, "EMERGENCY cleared");
}

/* ---------- adaptive duration --------------------------------------- */
static int compute_green_duration(int waiting_this, int waiting_cross)
{
    if (waiting_this == 0 && waiting_cross == 0) return S->t_green_min;
    int extra = waiting_this * 1 - waiting_cross / 2;
    int dur   = S->t_green_min + (extra > 0 ? extra : 0);
    if (dur < S->t_green_min) dur = S->t_green_min;
    if (dur > S->t_green_max) dur = S->t_green_max;
    return dur;
}

/* ---------- safety monitor (called every iteration) ----------------- */
static void check_safety(void)
{
    sem_lock(semid);
    if (!state_is_safe(S)) {
        S->safety_violations++;
        send_log(qlog, "CTRL", 2,
                 "SAFETY VIOLATION detected — forcing ALL-RED");
        for (int i = 0; i < NUM_DIRECTIONS; ++i)
            S->light[i] = LIGHT_RED;
    }
    sem_unlock(semid);
}

/* ---------- signal handler ------------------------------------------ */
static volatile sig_atomic_t stop_flag = 0;
static void on_signal(int sig) { (void)sig; stop_flag = 1; }

/* ---------- main ---------------------------------------------------- */
int main(int argc, char **argv)
{
    (void)argc; (void)argv;
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    shmid = ipc_shm_attach_existing();
    semid = ipc_sem_attach_existing();
    qcmd  = ipc_q_attach_existing(IPC_ID_MQ_CMD);
    qevt  = ipc_q_attach_existing(IPC_ID_MQ_EVT);
    qlog  = ipc_q_attach_existing(IPC_ID_MQ_LOG);
    if (shmid < 0 || semid < 0 || qcmd < 0 || qevt < 0 || qlog < 0) {
        fprintf(stderr, "controller: IPC attach failed — run main first\n");
        return 1;
    }
    S = (shared_state_t *)shmat(shmid, NULL, 0);
    if (S == (void *)-1) { perror("shmat"); return 1; }

    send_log(qlog, "CTRL", 0, "Controller process started (pid=%d)", getpid());

    /* Start in ALL-RED for one cycle, then NS green                      */
    execute_all_red(PHASE_ALL_RED_1);

    while (!stop_flag) {
        sem_lock(semid);
        int running = S->running;
        sem_unlock(semid);
        if (!running) break;

        /* Ticket freeze: if a red-light runner is being chased, hold
         * here.  Any phase that was in progress already completed
         * (or paused inside its own loop). */
        wait_for_ticket_done();

        int ped_any, em_active, w_ns, w_ew;
        direction_t ped_dir, em_dir;
        read_flags(&ped_any, &ped_dir, &em_active, &em_dir, &w_ns, &w_ew);

        if (em_active) {
            execute_emergency();
            /* After emergency, all lights are red. Run a yellow transition
             * before the next green so lights never jump red → green. */
            execute_all_red(PHASE_ALL_RED_1);
            continue;
        }

        /* If pedestrian waited too long, serve them between phases       */
        int now = time(NULL);
        int ped_urgent = 0;
        sem_lock(semid);
        for (int i = 0; i < NUM_DIRECTIONS; ++i) {
            if (S->pedestrian_pending[i] &&
                now - S->pedestrian_request_time[i] > S->t_pedestrian_max_wait/2) {
                ped_urgent = 1; break;
            }
        }
        sem_unlock(semid);

        /* ----- NS phase ----- */
        int dur_ns = compute_green_duration(w_ns, w_ew);
        execute_ns_green(dur_ns);
        check_safety();
        if (stop_flag) continue;
        /* Re-read shared state: the local em_active is stale if an
         * emergency arrived during execute_ns_green. */
        if (check_emergency()) {
            execute_emergency();
            execute_all_red(PHASE_ALL_RED_1); /* ensure red before next green */
            continue;
        }

        execute_yellow(DIR_NORTH, DIR_SOUTH, PHASE_NS_YELLOW);
        if (check_emergency()) {
            execute_emergency();
            execute_all_red(PHASE_ALL_RED_1);
            continue;
        }

        execute_all_red(PHASE_ALL_RED_1);
        if (check_emergency()) {
            execute_emergency();
            execute_all_red(PHASE_ALL_RED_1);
            continue;
        }

        /* Pedestrians are served concurrently with the perpendicular
         * vehicle green inside execute_ns_green()/execute_ew_green().
         * No dedicated all-red pedestrian phase is needed here. */
        check_safety();
        if (stop_flag) continue;

        /* ----- EW phase ----- */
        read_flags(&ped_any, &ped_dir, &em_active, &em_dir, &w_ns, &w_ew);
        if (em_active) {
            execute_emergency();
            execute_all_red(PHASE_ALL_RED_1);
            continue;
        }

        int dur_ew = compute_green_duration(w_ew, w_ns);
        execute_ew_green(dur_ew);
        check_safety();
        if (check_emergency()) {
            execute_emergency();
            execute_all_red(PHASE_ALL_RED_1);
            continue;
        }

        execute_yellow(DIR_EAST, DIR_WEST, PHASE_EW_YELLOW);
        if (check_emergency()) {
            execute_emergency();
            execute_all_red(PHASE_ALL_RED_1);
            continue;
        }

        execute_all_red(PHASE_ALL_RED_2);
        check_safety();
        if (check_emergency()) {
            execute_emergency();
            execute_all_red(PHASE_ALL_RED_1);
            continue;
        }

        (void)ped_urgent;  /* silence unused warning in some builds */
    }

    send_log(qlog, "CTRL", 0, "Controller shutting down");
    shmdt(S);
    return 0;
}
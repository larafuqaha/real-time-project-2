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
            if (!S->pedestrian_pending[e.direction]) {
                S->pedestrian_pending[e.direction]    = 1;
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
        }
        sem_unlock(semid);
    }
}

/* ---------- phase executors ----------------------------------------- */

/* Run NS green: send GREEN to N,S and RED to E,W                         */
static void execute_ns_green(int duration)
{
    send_command(DIR_NORTH, LIGHT_GREEN);
    send_command(DIR_SOUTH, LIGHT_GREEN);
    send_command(DIR_EAST,  LIGHT_RED);
    send_command(DIR_WEST,  LIGHT_RED);
    set_phase(PHASE_NS_GREEN, duration);

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

        if (emerg) break;

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

        sleep(1);
    }
}

static void execute_ew_green(int duration)
{
    send_command(DIR_EAST,  LIGHT_GREEN);
    send_command(DIR_WEST,  LIGHT_GREEN);
    send_command(DIR_NORTH, LIGHT_RED);
    send_command(DIR_SOUTH, LIGHT_RED);
    set_phase(PHASE_EW_GREEN, duration);

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

        if (emerg) break;

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

        sleep(1);
    }
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
        sem_unlock(semid);
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
        sem_unlock(semid);
        sleep(1);
    }
}

static void execute_pedestrian(void)
{
    /* All vehicles already red (must come from ALL_RED).                 */
    sem_lock(semid);
    S->pedestrian_active    = 1;
    S->pedestrian_remaining = S->t_pedestrian;
    /* Clear all pending requests — they are being served now */
    for (int i = 0; i < NUM_DIRECTIONS; ++i) {
        if (S->pedestrian_pending[i]) {
            S->pedestrian_pending[i] = 0;
            S->total_pedestrians_served++;
        }
    }
    sem_unlock(semid);
    set_phase(PHASE_PEDESTRIAN, S->t_pedestrian);
    send_log(qlog, "CTRL", 0, "Pedestrian crossing OPEN");
    send_log(qlog, "CTRL", 0, "Pedestrian request ACCEPTED for all pending directions");

    for (int i = 0; i < S->t_pedestrian; ++i) {
        drain_events();
        sem_lock(semid);
        S->pedestrian_remaining = S->t_pedestrian - i;
        S->phase_remaining      = S->t_pedestrian - i;
        int emerg = S->emergency_active;
        sem_unlock(semid);
        if (emerg) break;
        sleep(1);
    }

    sem_lock(semid);
    S->pedestrian_active    = 0;
    S->pedestrian_remaining = 0;
    int served = S->total_pedestrians_served;
    sem_unlock(semid);
    send_log(qlog, "CTRL", 0, "Pedestrian crossing CLOSED");
    send_log(qlog, "CTRL", 0, "Pedestrian request COMPLETED — total served so far: %d", served);
}

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

    /* Safe transition: anything green becomes yellow then red.           */
    for (int i = 0; i < NUM_DIRECTIONS; ++i) {
        sem_lock(semid);
        light_color_t c = S->light[i];
        sem_unlock(semid);
        if (c == LIGHT_GREEN) send_command((direction_t)i, LIGHT_YELLOW);
    }
    sleep(S->t_yellow);
    for (int i = 0; i < NUM_DIRECTIONS; ++i)
        send_command((direction_t)i, LIGHT_RED);
    sleep(S->t_all_red);

    /* Give green to the light FACING the ambulance                       */
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

    /* Safe exit: yellow then red                                         */
    send_command(green_dir, LIGHT_YELLOW);
    sleep(S->t_yellow);
    send_command(green_dir, LIGHT_RED);
    sleep(S->t_all_red);

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

        int ped_any, em_active, w_ns, w_ew;
        direction_t ped_dir, em_dir;
        read_flags(&ped_any, &ped_dir, &em_active, &em_dir, &w_ns, &w_ew);

        if (em_active) {
            execute_emergency();
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
        if (stop_flag || em_active) continue;

        execute_yellow(DIR_NORTH, DIR_SOUTH, PHASE_NS_YELLOW);
        execute_all_red(PHASE_ALL_RED_1);

        /* Pedestrian served between phases if pending */
        sem_lock(semid);
        int any_ped = 0;
        for (int i = 0; i < NUM_DIRECTIONS; ++i)
            if (S->pedestrian_pending[i]) { any_ped = 1; break; }
        sem_unlock(semid);
        if (any_ped) execute_pedestrian();
        check_safety();
        if (stop_flag) continue;

        /* ----- EW phase ----- */
        read_flags(&ped_any, &ped_dir, &em_active, &em_dir, &w_ns, &w_ew);
        if (em_active) { execute_emergency(); continue; }

        int dur_ew = compute_green_duration(w_ew, w_ns);
        execute_ew_green(dur_ew);
        check_safety();

        execute_yellow(DIR_EAST, DIR_WEST, PHASE_EW_YELLOW);
        execute_all_red(PHASE_ALL_RED_2);
        check_safety();

        (void)ped_urgent;  /* silence unused warning in some builds */
    }

    send_log(qlog, "CTRL", 0, "Controller shutting down");
    shmdt(S);
    return 0;
}

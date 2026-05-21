/******************************************************************************
 * control_logic.c
 *
 * The "brain" of the intersection. Consumes events from the event queue and
 * issues commands on the command queue while enforcing all safety constraints.
 *
 * Phase sequence (normal operation):
 *   NS_THROUGH (g/y) -> ALL_RED -> NS_LEFT (g/y) -> ALL_RED ->
 *   EW_THROUGH (g/y) -> ALL_RED -> EW_LEFT (g/y) -> ALL_RED -> ...
 *
 * Pedestrians are served concurrently with the THROUGH phase of the
 * perpendicular axis: e.g. when EW_THROUGH is green, North and South
 * crosswalks (which cross EW traffic, so vehicles there are red) are WALK.
 * This matches real US/EU intersection behaviour and removes the need for a
 * separate all-pedestrian phase, exactly as the user requested.
 *
 * Emergencies preempt the schedule via a safe ALL_RED transition, then give
 * the emergency direction a dedicated "emergency corridor" green.
 *****************************************************************************/
#include "common.h"

static volatile sig_atomic_t g_running = 1;

static int shmid, evtid, cmdid, stsid, semid;
static SharedState *S;

static void on_signal(int sig)
{
    UNUSED(sig);
    g_running = 0;
}

/* -- Helpers to send commands -------------------------------------------- */
static void cmd_set_light(Direction d, Lane l, LightState st)
{
    CommandMsg m = {0};
    m.mtype       = CMD_SET_LIGHT * 10 + d;
    m.direction   = d;
    m.lane        = l;
    m.new_state   = st;
    m.timestamp   = time(NULL);
    msgsnd(cmdid, &m, sizeof(m) - sizeof(long), 0);
}

static void cmd_set_ped(Direction d, PedState st)
{
    CommandMsg m = {0};
    m.mtype       = CMD_SET_PED * 10 + d;
    m.direction   = d;
    m.new_state   = st;
    m.timestamp   = time(NULL);
    msgsnd(cmdid, &m, sizeof(m) - sizeof(long), 0);
}

/* -- Set all vehicle lights to RED --------------------------------------- */
static void all_red(void)
{
    for (int d = 0; d < NUM_DIRS; d++)
        for (int l = 0; l < NUM_LANES; l++)
            cmd_set_light(d, l, LIGHT_RED);
}

/* -- Drain all pedestrian signals to DONT_WALK --------------------------- */
static void all_ped_stop(void)
{
    for (int d = 0; d < NUM_DIRS; d++)
        cmd_set_ped(d, PED_DONT_WALK);
}

/* -- Sleep helper that respects shutdown --------------------------------- */
static int sleep_phase(int seconds)
{
    for (int i = 0; i < seconds * 10 && g_running && S->system_running; i++) {
        usleep(100000); /* 100 ms */
        sem_lock(semid);
        S->phase_remaining_sec = seconds - (i / 10);
        if (S->emergency_active && S->current_phase != PHASE_EMERGENCY) {
            sem_unlock(semid);
            return 1; /* preempted */
        }
        sem_unlock(semid);
    }
    return 0;
}

/* -- Drain pending events into shared state ------------------------------ */
static void drain_events(void)
{
    EventMsg e;
    while (msgrcv(evtid, &e, sizeof(e) - sizeof(long), 0, IPC_NOWAIT) >= 0) {
        sem_lock(semid);
        switch (e.mtype) {
        case EVT_VEHICLE_ARRIVE: {
            int d = e.direction, l = e.lane;
            if (d < 0 || d >= NUM_DIRS) { sem_unlock(semid); break; }
            if (l < 0 || l >= NUM_LANES) l = LANE_THROUGH;
            if (S->queue[d][l].count == 0)
                S->queue[d][l].oldest_wait_time = time(NULL);
            if (S->queue[d][l].count < MAX_QUEUE_LEN)
                S->queue[d][l].count++;
            sem_unlock(semid);
            log_event(S, semid, "Vehicle arrived %s/%s (queue=%d)",
                      dir_name(d), lane_name(l), S->queue[d][l].count);
            break;
        }
        case EVT_VEHICLE_DEPART: {
            int d = e.direction, l = e.lane;
            if (d < 0 || d >= NUM_DIRS) { sem_unlock(semid); break; }
            if (l < 0 || l >= NUM_LANES) l = LANE_THROUGH;
            if (S->queue[d][l].count > 0) {
                S->queue[d][l].count--;
                S->total_vehicles_served++;
            }
            if (S->queue[d][l].count == 0)
                S->queue[d][l].oldest_wait_time = 0;
            else
                S->queue[d][l].oldest_wait_time = time(NULL);
            sem_unlock(semid);
            break;
        }
        case EVT_PED_REQUEST: {
            int d = e.direction;
            if (d < 0 || d >= NUM_DIRS) { sem_unlock(semid); break; }
            if (!S->ped_request_pending[d]) {
                S->ped_request_pending[d] = 1;
                S->ped_request_time[d] = time(NULL);
            }
            sem_unlock(semid);
            log_event(S, semid, "Pedestrian request at %s crosswalk",
                      dir_name(d));
            break;
        }
        case EVT_EMERGENCY: {
            int d = e.direction;
            if (d < 0 || d >= NUM_DIRS) { sem_unlock(semid); break; }
            if (!S->emergency_active) {
                S->emergency_active = 1;
                S->emergency_direction = (Direction)d;
                S->emergency_start_time = time(NULL);
                S->total_emergencies_handled++;
            }
            sem_unlock(semid);
            log_event(S, semid, "!! EMERGENCY VEHICLE detected from %s",
                      dir_name(d));
            break;
        }
        case EVT_SHUTDOWN:
            sem_unlock(semid);
            S->system_running = 0;
            g_running = 0;
            return;
        default:
            sem_unlock(semid);
            break;
        }
    }
}

/* -- Pedestrian helpers --------------------------------------------------- */
/* A pedestrian crosswalk crosses traffic that comes FROM that direction.
 * So when EW vehicles are RED, the EAST and WEST crosswalks are safe (a
 * pedestrian crossing the East approach is walking perpendicular to E-bound
 * cars, which are stopped). Similarly NS crosswalks are safe when NS
 * vehicles are RED. */

static void serve_pedestrians_for_axis(int ns_axis_red)
{
    /* When NS vehicles are red -> serve North & South crosswalks.
     * When EW vehicles are red -> serve East & West crosswalks. */
    if (ns_axis_red) {
        cmd_set_ped(DIR_NORTH, PED_WALK);
        cmd_set_ped(DIR_SOUTH, PED_WALK);
    } else {
        cmd_set_ped(DIR_EAST, PED_WALK);
        cmd_set_ped(DIR_WEST, PED_WALK);
    }
}

static void flash_pedestrians_for_axis(int ns_axis_red)
{
    if (ns_axis_red) {
        cmd_set_ped(DIR_NORTH, PED_WALK_FLASHING);
        cmd_set_ped(DIR_SOUTH, PED_WALK_FLASHING);
    } else {
        cmd_set_ped(DIR_EAST, PED_WALK_FLASHING);
        cmd_set_ped(DIR_WEST, PED_WALK_FLASHING);
    }
}

/* -- Phase routines ------------------------------------------------------- */
static void run_all_red(int seconds)
{
    sem_lock(semid);
    S->current_phase = PHASE_ALL_RED;
    S->phase_sub     = SUB_RED;
    S->phase_start_time = time(NULL);
    sem_unlock(semid);
    all_red();
    all_ped_stop();
    log_event(S, semid, "Phase: ALL_RED (%ds)", seconds);
    sleep_phase(seconds);
}

static int has_demand_left(Direction a, Direction b)
{
    sem_lock(semid);
    int d = (S->queue[a][LANE_LEFT].count > 0 ||
             S->queue[b][LANE_LEFT].count > 0);
    sem_unlock(semid);
    return d;
}

static int has_demand_through(Direction a, Direction b)
{
    sem_lock(semid);
    int d = (S->queue[a][LANE_THROUGH].count > 0 ||
             S->queue[b][LANE_THROUGH].count > 0);
    sem_unlock(semid);
    return d;
}

/* Compute green time based on demand (semi-dynamic). */
static int compute_green_sec(int base_min, int base_max, int queued)
{
    int g = base_min + queued; /* 1 sec per waiting vehicle, roughly */
    if (g > base_max) g = base_max;
    if (g < base_min) g = base_min;
    return g;
}

/* Run a vehicle-movement phase: GREEN for the two named (dir,lane) groups,
 * pedestrians on the orthogonal axis (if traffic on those approaches is red),
 * then YELLOW, then handing off to ALL_RED. */
static void run_vehicle_phase(TrafficPhase phase,
                              Direction a, Direction b, Lane lane,
                              int ns_axis_active)
{
    sem_lock(semid);
    S->current_phase    = phase;
    S->phase_sub        = SUB_GREEN;
    S->phase_start_time = time(NULL);
    int qa = S->queue[a][lane].count;
    int qb = S->queue[b][lane].count;
    sem_unlock(semid);

    int total_q = qa + qb;
    int green_sec;
    if (lane == LANE_LEFT) {
        green_sec = compute_green_sec(S->cfg.left_turn_green_sec,
                                      S->cfg.left_turn_green_sec + 4,
                                      total_q);
    } else {
        green_sec = compute_green_sec(S->cfg.green_min_sec,
                                      S->cfg.green_max_sec,
                                      total_q);
    }

    /* Skip empty phases (unless a pedestrian is waiting on the orthogonal
     * axis, in which case we still need a green here so its red holds). */
    if (total_q == 0 && lane == LANE_LEFT) {
        log_event(S, semid, "Phase %s skipped (no demand)", phase_name(phase));
        return;
    }

    /* All red before transition (safe baseline) */
    all_red();
    all_ped_stop();
    usleep(200000);

    /* Set the two greens */
    cmd_set_light(a, lane, LIGHT_GREEN);
    cmd_set_light(b, lane, LIGHT_GREEN);

    log_event(S, semid, "Phase: %s GREEN (%ds, q=%d+%d)",
              phase_name(phase), green_sec, qa, qb);

    /* For THROUGH phases on one axis, pedestrians on the OTHER axis walk. */
    if (lane == LANE_THROUGH) {
        /* The orthogonal axis is fully red (we just set everything red, then
         * raised these two greens), so pedestrians on the other axis are
         * safe. */
        int ns_red = !ns_axis_active;
        sleep_phase(1); /* let red commands propagate */
        serve_pedestrians_for_axis(ns_red);
    }

    /* Hold green for green_sec - ped_flash_sec to allow clearance */
    int hold = green_sec - S->cfg.ped_flash_sec;
    if (hold < 2) hold = 2;
    if (sleep_phase(hold)) goto preempted;

    /* Flashing don't-walk during clearance */
    if (lane == LANE_THROUGH) {
        flash_pedestrians_for_axis(!ns_axis_active);
        if (sleep_phase(S->cfg.ped_flash_sec)) goto preempted;
        all_ped_stop();
    }

    /* Yellow */
    sem_lock(semid);
    S->phase_sub = SUB_YELLOW;
    sem_unlock(semid);
    cmd_set_light(a, lane, LIGHT_YELLOW);
    cmd_set_light(b, lane, LIGHT_YELLOW);
    log_event(S, semid, "Phase: %s YELLOW (%ds)",
              phase_name(phase), S->cfg.yellow_sec);
    if (sleep_phase(S->cfg.yellow_sec)) goto preempted;

    cmd_set_light(a, lane, LIGHT_RED);
    cmd_set_light(b, lane, LIGHT_RED);
    return;

preempted:
    all_red();
    all_ped_stop();
    log_event(S, semid, "Phase %s preempted by emergency", phase_name(phase));
}

/* -- Emergency handling --------------------------------------------------- */
static void run_emergency(void)
{
    sem_lock(semid);
    Direction edir = S->emergency_direction;
    S->current_phase = PHASE_EMERGENCY;
    S->phase_sub = SUB_RED;
    S->phase_start_time = time(NULL);
    sem_unlock(semid);

    log_event(S, semid, "!! Entering EMERGENCY mode for %s corridor",
              dir_name(edir));

    /* Safe clearance: yellow then all red */
    for (int d = 0; d < NUM_DIRS; d++)
        for (int l = 0; l < NUM_LANES; l++)
            cmd_set_light((Direction)d, (Lane)l, LIGHT_YELLOW);
    all_ped_stop();
    sleep_phase(S->cfg.yellow_sec);

    all_red();
    sleep_phase(S->cfg.emergency_clear_sec);

    /* Give the emergency direction a dedicated through green – this is the
     * "small lane" the user asked for. We only raise the THROUGH lane so we
     * don't create left-turn conflicts. */
    cmd_set_light(edir, LANE_THROUGH, LIGHT_GREEN);
    sem_lock(semid);
    S->phase_sub = SUB_GREEN;
    sem_unlock(semid);
    log_event(S, semid, "Emergency corridor OPEN for %s (%ds)",
              dir_name(edir), S->cfg.emergency_green_sec);
    sleep_phase(S->cfg.emergency_green_sec);

    /* Yellow + red */
    cmd_set_light(edir, LANE_THROUGH, LIGHT_YELLOW);
    sleep_phase(S->cfg.yellow_sec);
    cmd_set_light(edir, LANE_THROUGH, LIGHT_RED);
    sleep_phase(S->cfg.all_red_sec);

    sem_lock(semid);
    S->emergency_active = 0;
    sem_unlock(semid);
    log_event(S, semid, "Emergency mode CLEARED");
}

/* -- Main loop ------------------------------------------------------------ */
int main(int argc, char **argv)
{
    UNUSED(argc); UNUSED(argv);

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    shmid = get_shm(0);
    evtid = get_evt_queue(0);
    cmdid = get_cmd_queue(0);
    stsid = get_sts_queue(0);
    semid = get_sem(0);
    if (shmid < 0 || evtid < 0 || cmdid < 0 || stsid < 0 || semid < 0) {
        fprintf(stderr, "control_logic: IPC attach failed\n");
        return EXIT_FAILURE;
    }
    S = (SharedState *)shmat(shmid, NULL, 0);
    if (S == (void *)-1) { perror("shmat"); return EXIT_FAILURE; }

    log_event(S, semid, "ControlLogic started (PID %d)", getpid());

    /* Initialise: all red, all don't walk */
    all_red();
    all_ped_stop();
    sleep_phase(S->cfg.all_red_sec);

    while (g_running && S->system_running) {
        drain_events();

        if (S->emergency_active) {
            run_emergency();
            continue;
        }

        /* NS through */
        if (has_demand_through(DIR_NORTH, DIR_SOUTH) ||
            S->ped_request_pending[DIR_EAST] ||
            S->ped_request_pending[DIR_WEST]) {
            run_vehicle_phase(PHASE_NS_THROUGH,
                              DIR_NORTH, DIR_SOUTH, LANE_THROUGH, 1);
            run_all_red(S->cfg.all_red_sec);
        }

        drain_events();
        if (S->emergency_active) { run_emergency(); continue; }

        /* NS left */
        if (has_demand_left(DIR_NORTH, DIR_SOUTH)) {
            run_vehicle_phase(PHASE_NS_LEFT,
                              DIR_NORTH, DIR_SOUTH, LANE_LEFT, 1);
            run_all_red(S->cfg.all_red_sec);
        }

        drain_events();
        if (S->emergency_active) { run_emergency(); continue; }

        /* EW through */
        if (has_demand_through(DIR_EAST, DIR_WEST) ||
            S->ped_request_pending[DIR_NORTH] ||
            S->ped_request_pending[DIR_SOUTH]) {
            run_vehicle_phase(PHASE_EW_THROUGH,
                              DIR_EAST, DIR_WEST, LANE_THROUGH, 0);
            run_all_red(S->cfg.all_red_sec);
        }

        drain_events();
        if (S->emergency_active) { run_emergency(); continue; }

        /* EW left */
        if (has_demand_left(DIR_EAST, DIR_WEST)) {
            run_vehicle_phase(PHASE_EW_LEFT,
                              DIR_EAST, DIR_WEST, LANE_LEFT, 0);
            run_all_red(S->cfg.all_red_sec);
        }

        /* If absolutely nothing is happening, keep a small idle gap so the
         * GUI shows steady all-red rather than busy-looping. */
        if (!has_demand_through(DIR_NORTH, DIR_SOUTH) &&
            !has_demand_through(DIR_EAST, DIR_WEST) &&
            !has_demand_left(DIR_NORTH, DIR_SOUTH)   &&
            !has_demand_left(DIR_EAST, DIR_WEST)) {
            usleep(200000);
        }
    }

    log_event(S, semid, "ControlLogic exiting");
    shmdt(S);
    return EXIT_SUCCESS;
}

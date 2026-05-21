/*******************************************************************************
 *  test_integration.c  —  Integration Tests
 *  ENCS4330 Project #2  —  Birzeit University
 *
 *  Tests the real IPC resources (shared memory, semaphores, message queues)
 *  in isolation, simulating the interactions between system components without
 *  spawning the full set of processes.
 *
 *  Suites:
 *    A — IPC lifecycle (create, attach, destroy)
 *    B — Shared memory: concurrent read/write under semaphore
 *    C — Command queue: send cmd_msg_t, receive ack_msg_t
 *    D — Event queue: all event types round-trip
 *    E — Log queue: fire-and-forget log delivery
 *    F — Phase state machine transitions (logic only, no real processes)
 *    G — Safety invariants enforced across simulated phase changes
 *    H — Config file loaded into shared memory
 *    I — Emergency interrupt sequence (state-machine simulation)
 *    J — Pedestrian request lifecycle
 *
 *  Build:  make test_integration
 *  Run:    ./tests/test_integration
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <sys/msg.h>
#include <sys/wait.h>

#include "../src/common.h"
#include "../src/config.h"
#include "../src/ipc_init.h"

/* ── tiny framework ─────────────────────────────────────────────────────── */
static int g_passed = 0;
static int g_failed = 0;

#define TEST(n) do { printf("  %-60s", n); fflush(stdout); } while(0)
#define PASS()  do { printf("PASS\n"); g_passed++; } while(0)
#define FAIL(m) do { printf("FAIL  (%s)\n", m); g_failed++; } while(0)
#define CHECK(c,m) do { if (c) PASS(); else FAIL(m); } while(0)

/* ── IPC handles (test-local, different keys from production) ────────────── */
static int t_shmid = -1, t_semid = -1;
static int t_qcmd  = -1, t_qevt  = -1, t_qlog  = -1;
static shared_state_t *T = NULL;          /* test shared-memory pointer */

/* Use project ids that differ from production to avoid collisions */
#define TKEY_PATH  "/tmp"
#define TKEY_SHM   'a'
#define TKEY_SEM   'b'
#define TKEY_CMD   'c'
#define TKEY_EVT   'd'
#define TKEY_LOG   'e'

static key_t tkey(int id) { return ftok(TKEY_PATH, id); }

static void ipc_create_all(void)
{
    /* Remove leftovers first */
    int tmp;
    if ((tmp = shmget(tkey(TKEY_SHM), sizeof(shared_state_t), 0666)) >= 0)
        shmctl(tmp, IPC_RMID, NULL);
    if ((tmp = semget(tkey(TKEY_SEM), 1, 0666)) >= 0)
        semctl(tmp, 0, IPC_RMID);
    if ((tmp = msgget(tkey(TKEY_CMD), 0666)) >= 0)
        msgctl(tmp, IPC_RMID, NULL);
    if ((tmp = msgget(tkey(TKEY_EVT), 0666)) >= 0)
        msgctl(tmp, IPC_RMID, NULL);
    if ((tmp = msgget(tkey(TKEY_LOG), 0666)) >= 0)
        msgctl(tmp, IPC_RMID, NULL);

    t_shmid = shmget(tkey(TKEY_SHM), sizeof(shared_state_t),
                     IPC_CREAT | 0666);
    t_semid = semget(tkey(TKEY_SEM), 1, IPC_CREAT | 0666);
    t_qcmd  = msgget(tkey(TKEY_CMD), IPC_CREAT | 0666);
    t_qevt  = msgget(tkey(TKEY_EVT), IPC_CREAT | 0666);
    t_qlog  = msgget(tkey(TKEY_LOG), IPC_CREAT | 0666);

    if (t_shmid < 0 || t_semid < 0 ||
        t_qcmd < 0  || t_qevt < 0 || t_qlog < 0) {
        fprintf(stderr, "FATAL: IPC creation failed\n");
        exit(1);
    }

    union semun arg; arg.val = 1;
    semctl(t_semid, 0, SETVAL, arg);

    T = (shared_state_t *)shmat(t_shmid, NULL, 0);
    if (T == (void *)-1) { perror("shmat"); exit(1); }
    memset(T, 0, sizeof(shared_state_t));
    for (int i = 0; i < NUM_DIRECTIONS; i++) T->light[i] = LIGHT_RED;
}

static void ipc_destroy_all_test(void)
{
    if (T && T != (void *)-1) shmdt(T);
    if (t_qcmd  >= 0) msgctl(t_qcmd,  IPC_RMID, NULL);
    if (t_qevt  >= 0) msgctl(t_qevt,  IPC_RMID, NULL);
    if (t_qlog  >= 0) msgctl(t_qlog,  IPC_RMID, NULL);
    if (t_semid >= 0) semctl(t_semid, 0, IPC_RMID);
    if (t_shmid >= 0) shmctl(t_shmid, IPC_RMID, NULL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SUITE A — IPC lifecycle
 * ═════════════════════════════════════════════════════════════════════════ */
static void suite_ipc_lifecycle(void)
{
    printf("\n[Suite A] IPC lifecycle\n");

    TEST("Shared memory created successfully");
    CHECK(t_shmid >= 0, "shmget failed");

    TEST("Semaphore created successfully");
    CHECK(t_semid >= 0, "semget failed");

    TEST("Command queue created successfully");
    CHECK(t_qcmd >= 0, "msgget(cmd) failed");

    TEST("Event queue created successfully");
    CHECK(t_qevt >= 0, "msgget(evt) failed");

    TEST("Log queue created successfully");
    CHECK(t_qlog >= 0, "msgget(log) failed");

    TEST("Shared memory attachment succeeds");
    CHECK(T != NULL && T != (void *)-1, "shmat failed");

    TEST("Shared memory is zeroed after calloc-style init");
    int ok = 1;
    for (int i = 0; i < NUM_DIRECTIONS; i++)
        /* We set lights to RED in ipc_create_all */
        if (T->light[i] != LIGHT_RED) ok = 0;
    CHECK(ok, "initial state not all-red");

    TEST("shmget with existing key (IPC_CREAT) returns same id");
    int dup = shmget(tkey(TKEY_SHM), sizeof(shared_state_t),
                     IPC_CREAT | 0666);
    CHECK(dup == t_shmid, "duplicate shmget returned different id");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SUITE B — Shared memory concurrent access
 * ═════════════════════════════════════════════════════════════════════════ */
static void suite_shm_concurrent(void)
{
    printf("\n[Suite B] Shared memory under semaphore\n");

    TEST("sem_lock + write + sem_unlock works");
    sem_lock(t_semid);
    T->total_vehicles_served = 42;
    sem_unlock(t_semid);
    CHECK(T->total_vehicles_served == 42, "write lost");

    TEST("Forked child increments counter safely");
    T->total_vehicles_served = 0;
    pid_t child = fork();
    if (child == 0) {
        /* child: 1000 increments */
        shared_state_t *C = (shared_state_t *)shmat(t_shmid, NULL, 0);
        for (int i = 0; i < 100; i++) {
            sem_lock(t_semid);
            C->total_vehicles_served++;
            sem_unlock(t_semid);
        }
        shmdt(C);
        exit(0);
    } else {
        /* parent: 1000 increments */
        for (int i = 0; i < 100; i++) {
            sem_lock(t_semid);
            T->total_vehicles_served++;
            sem_unlock(t_semid);
        }
        waitpid(child, NULL, 0);
        CHECK(T->total_vehicles_served == 200,
              "race condition — counter != 200");
    }

    TEST("Phase remaining can be decremented atomically");
    sem_lock(t_semid);
    T->phase_remaining = 10;
    T->phase_remaining--;
    sem_unlock(t_semid);
    CHECK(T->phase_remaining == 9, "decrement failed");

    TEST("Safety violation counter increments correctly");
    sem_lock(t_semid);
    T->safety_violations = 0;
    T->safety_violations++;
    sem_unlock(t_semid);
    CHECK(T->safety_violations == 1, "violation counter wrong");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SUITE C — Command queue round-trip
 * ═════════════════════════════════════════════════════════════════════════ */
static void suite_cmd_queue(void)
{
    printf("\n[Suite C] Command queue (cmd + ack)\n");

    TEST("Send cmd_msg_t to queue");
    cmd_msg_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.mtype        = MTYPE_CMD_BASE + DIR_NORTH + 1;
    cmd.direction    = DIR_NORTH;
    cmd.target_color = LIGHT_GREEN;
    cmd.timestamp    = time(NULL);
    cmd.cmd_id       = 1;
    int r = msgsnd(t_qcmd, &cmd, sizeof(cmd) - sizeof(long), IPC_NOWAIT);
    CHECK(r == 0, "msgsnd cmd failed");

    TEST("Receive cmd_msg_t from queue");
    cmd_msg_t rxcmd;
    ssize_t n = msgrcv(t_qcmd, &rxcmd, sizeof(rxcmd) - sizeof(long),
                       MTYPE_CMD_BASE + DIR_NORTH + 1, IPC_NOWAIT);
    CHECK(n > 0, "msgrcv cmd failed");

    TEST("Received cmd has correct direction and color");
    CHECK(rxcmd.direction == DIR_NORTH &&
          rxcmd.target_color == LIGHT_GREEN, "cmd fields wrong");

    TEST("Send ack_msg_t and receive it");
    ack_msg_t ack;
    memset(&ack, 0, sizeof(ack));
    ack.mtype         = MTYPE_ACK_BASE + DIR_NORTH + 1;
    ack.direction     = DIR_NORTH;
    ack.current_color = LIGHT_GREEN;
    ack.cmd_id        = 1;
    ack.success       = 1;
    msgsnd(t_qcmd, &ack, sizeof(ack) - sizeof(long), IPC_NOWAIT);
    ack_msg_t rxack;
    n = msgrcv(t_qcmd, &rxack, sizeof(rxack) - sizeof(long),
               MTYPE_ACK_BASE + DIR_NORTH + 1, IPC_NOWAIT);
    CHECK(n > 0 && rxack.success == 1 &&
          rxack.current_color == LIGHT_GREEN, "ack round-trip failed");

    TEST("Commands for different directions use different mtypes");
    cmd_msg_t c1, c2;
    memset(&c1, 0, sizeof(c1)); memset(&c2, 0, sizeof(c2));
    c1.mtype = MTYPE_CMD_BASE + DIR_NORTH + 1;
    c2.mtype = MTYPE_CMD_BASE + DIR_EAST  + 1;
    c1.direction = DIR_NORTH; c2.direction = DIR_EAST;
    c1.target_color = LIGHT_GREEN; c2.target_color = LIGHT_RED;
    msgsnd(t_qcmd, &c1, sizeof(c1) - sizeof(long), IPC_NOWAIT);
    msgsnd(t_qcmd, &c2, sizeof(c2) - sizeof(long), IPC_NOWAIT);
    cmd_msg_t rx;
    /* Receive EAST first (IPC_NOWAIT with specific mtype) */
    n = msgrcv(t_qcmd, &rx, sizeof(rx) - sizeof(long),
               MTYPE_CMD_BASE + DIR_EAST + 1, IPC_NOWAIT);
    CHECK(n > 0 && rx.direction == DIR_EAST &&
          rx.target_color == LIGHT_RED, "selective receive failed");
    /* Drain NORTH */
    msgrcv(t_qcmd, &rx, sizeof(rx) - sizeof(long),
           MTYPE_CMD_BASE + DIR_NORTH + 1, IPC_NOWAIT);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SUITE D — Event queue round-trip
 * ═════════════════════════════════════════════════════════════════════════ */
static void suite_evt_queue(void)
{
    printf("\n[Suite D] Event queue\n");

    /* Helper lambda-equivalent */
    #define SEND_EVT(mtype_, dir_, count_) do { \
        evt_msg_t e; memset(&e, 0, sizeof(e)); \
        e.mtype = (mtype_); e.direction = (dir_); \
        e.count = (count_); e.timestamp = time(NULL); \
        msgsnd(t_qevt, &e, sizeof(e) - sizeof(long), IPC_NOWAIT); \
    } while(0)

    TEST("Vehicle event sent and received");
    SEND_EVT(MTYPE_EVT_VEHICLE, DIR_NORTH, 2);
    evt_msg_t ev;
    ssize_t n = msgrcv(t_qevt, &ev, sizeof(ev) - sizeof(long), 0, IPC_NOWAIT);
    CHECK(n > 0 && ev.mtype == MTYPE_EVT_VEHICLE &&
          ev.direction == DIR_NORTH && ev.count == 2, "vehicle event wrong");

    TEST("Pedestrian event sent and received");
    SEND_EVT(MTYPE_EVT_PEDESTRIAN, DIR_SOUTH, 1);
    n = msgrcv(t_qevt, &ev, sizeof(ev) - sizeof(long), 0, IPC_NOWAIT);
    CHECK(n > 0 && ev.mtype == MTYPE_EVT_PEDESTRIAN, "pedestrian event wrong");

    TEST("Emergency event sent and received");
    SEND_EVT(MTYPE_EVT_EMERGENCY, DIR_WEST, 1);
    n = msgrcv(t_qevt, &ev, sizeof(ev) - sizeof(long), 0, IPC_NOWAIT);
    CHECK(n > 0 && ev.mtype == MTYPE_EVT_EMERGENCY &&
          ev.direction == DIR_WEST, "emergency event wrong");

    TEST("Fault event sent and received");
    SEND_EVT(MTYPE_EVT_FAULT, DIR_EAST, 0);
    n = msgrcv(t_qevt, &ev, sizeof(ev) - sizeof(long), 0, IPC_NOWAIT);
    CHECK(n > 0 && ev.mtype == MTYPE_EVT_FAULT, "fault event wrong");

    TEST("Multiple events queued, drained in FIFO order");
    SEND_EVT(MTYPE_EVT_VEHICLE,    DIR_NORTH, 1);
    SEND_EVT(MTYPE_EVT_PEDESTRIAN, DIR_SOUTH, 1);
    SEND_EVT(MTYPE_EVT_EMERGENCY,  DIR_EAST,  1);
    int cnt = 0;
    while (msgrcv(t_qevt, &ev, sizeof(ev) - sizeof(long), 0, IPC_NOWAIT) > 0)
        cnt++;
    CHECK(cnt == 3, "not all events received");

    #undef SEND_EVT
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SUITE E — Log queue
 * ═════════════════════════════════════════════════════════════════════════ */
static void suite_log_queue(void)
{
    printf("\n[Suite E] Log queue\n");

    TEST("send_log() delivers to real queue");
    send_log(t_qlog, "INTEG", 0, "integration test log entry %d", 1);
    log_msg_t lm;
    ssize_t n = msgrcv(t_qlog, &lm, sizeof(lm) - sizeof(long),
                       MTYPE_LOG_ANY, IPC_NOWAIT);
    CHECK(n > 0, "log not received");

    TEST("Log source field preserved correctly");
    send_log(t_qlog, "CTRL", 1, "warning");
    n = msgrcv(t_qlog, &lm, sizeof(lm) - sizeof(long),
               MTYPE_LOG_ANY, IPC_NOWAIT);
    CHECK(n > 0 && strcmp(lm.source, "CTRL") == 0, "source field wrong");

    TEST("Log severity field set correctly");
    send_log(t_qlog, "X", 2, "error");
    n = msgrcv(t_qlog, &lm, sizeof(lm) - sizeof(long),
               MTYPE_LOG_ANY, IPC_NOWAIT);
    CHECK(n > 0 && lm.severity == 2, "severity wrong");

    TEST("Log timestamp is recent");
    send_log(t_qlog, "X", 0, "ts test");
    n = msgrcv(t_qlog, &lm, sizeof(lm) - sizeof(long),
               MTYPE_LOG_ANY, IPC_NOWAIT);
    time_t now = time(NULL);
    CHECK(n > 0 && lm.timestamp >= now - 2 && lm.timestamp <= now + 2,
          "timestamp out of range");

    TEST("50 rapid log entries all delivered");
    for (int i = 0; i < 50; i++)
        send_log(t_qlog, "STRESS", 0, "msg %d", i);
    int cnt = 0;
    while (msgrcv(t_qlog, &lm, sizeof(lm) - sizeof(long),
                  MTYPE_LOG_ANY, IPC_NOWAIT) > 0)
        cnt++;
    CHECK(cnt == 50, "some log messages dropped");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SUITE F — Phase state machine (logic only)
 * ═════════════════════════════════════════════════════════════════════════ */
static void suite_phase_sm(void)
{
    printf("\n[Suite F] Phase state machine logic\n");

    /* Helper: apply lights for a given phase */
    auto void apply_phase(phase_t p);
    void apply_phase(phase_t p) {
        for (int i = 0; i < NUM_DIRECTIONS; i++) T->light[i] = LIGHT_RED;
        switch (p) {
        case PHASE_NS_GREEN:
            T->light[DIR_NORTH] = LIGHT_GREEN;
            T->light[DIR_SOUTH] = LIGHT_GREEN;
            break;
        case PHASE_NS_YELLOW:
            T->light[DIR_NORTH] = LIGHT_YELLOW;
            T->light[DIR_SOUTH] = LIGHT_YELLOW;
            break;
        case PHASE_EW_GREEN:
            T->light[DIR_EAST]  = LIGHT_GREEN;
            T->light[DIR_WEST]  = LIGHT_GREEN;
            break;
        case PHASE_EW_YELLOW:
            T->light[DIR_EAST]  = LIGHT_YELLOW;
            T->light[DIR_WEST]  = LIGHT_YELLOW;
            break;
        case PHASE_ALL_RED_1:
        case PHASE_ALL_RED_2:
            break;  /* already all-red */
        default: break;
        }
        T->current_phase = p;
    }

    TEST("PHASE_NS_GREEN: NS green, EW red");
    apply_phase(PHASE_NS_GREEN);
    CHECK(T->light[DIR_NORTH] == LIGHT_GREEN &&
          T->light[DIR_EAST]  == LIGHT_RED, "NS_GREEN wrong");

    TEST("PHASE_NS_GREEN is safe");
    CHECK(state_is_safe(T) == 1, "NS_GREEN unsafe");

    TEST("PHASE_NS_YELLOW: NS yellow, EW red");
    apply_phase(PHASE_NS_YELLOW);
    CHECK(T->light[DIR_NORTH] == LIGHT_YELLOW &&
          T->light[DIR_EAST]  == LIGHT_RED, "NS_YELLOW wrong");

    TEST("PHASE_ALL_RED_1: all red");
    apply_phase(PHASE_ALL_RED_1);
    int ok = 1;
    for (int i = 0; i < NUM_DIRECTIONS; i++)
        if (T->light[i] != LIGHT_RED) ok = 0;
    CHECK(ok, "ALL_RED not all red");

    TEST("PHASE_EW_GREEN: EW green, NS red");
    apply_phase(PHASE_EW_GREEN);
    CHECK(T->light[DIR_EAST]  == LIGHT_GREEN &&
          T->light[DIR_NORTH] == LIGHT_RED, "EW_GREEN wrong");

    TEST("PHASE_EW_GREEN is safe");
    CHECK(state_is_safe(T) == 1, "EW_GREEN unsafe");

    TEST("PHASE_EW_YELLOW: EW yellow, NS red");
    apply_phase(PHASE_EW_YELLOW);
    CHECK(T->light[DIR_EAST]  == LIGHT_YELLOW &&
          T->light[DIR_NORTH] == LIGHT_RED, "EW_YELLOW wrong");

    TEST("All phases pass safety check");
    phase_t phases[] = {PHASE_NS_GREEN, PHASE_NS_YELLOW, PHASE_ALL_RED_1,
                        PHASE_EW_GREEN, PHASE_EW_YELLOW, PHASE_ALL_RED_2};
    int all_safe = 1;
    for (size_t i = 0; i < sizeof(phases)/sizeof(phases[0]); i++) {
        apply_phase(phases[i]);
        if (!state_is_safe(T)) { all_safe = 0; break; }
    }
    CHECK(all_safe, "a phase failed safety check");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SUITE G — Safety invariant stress
 * ═════════════════════════════════════════════════════════════════════════ */
static void suite_safety_stress(void)
{
    printf("\n[Suite G] Safety invariant stress\n");

    TEST("100 random safe-phase assignments stay safe");
    int ok = 1;
    srand(12345);
    for (int trial = 0; trial < 100; trial++) {
        for (int i = 0; i < NUM_DIRECTIONS; i++) T->light[i] = LIGHT_RED;
        /* Randomly make NS or EW green — never both */
        int axis = rand() % 2;  /* 0=NS, 1=EW */
        if (axis == 0) {
            T->light[DIR_NORTH] = LIGHT_GREEN;
            T->light[DIR_SOUTH] = LIGHT_GREEN;
        } else {
            T->light[DIR_EAST] = LIGHT_GREEN;
            T->light[DIR_WEST] = LIGHT_GREEN;
        }
        T->emergency_active  = 0;
        T->pedestrian_active = 0;
        if (!state_is_safe(T)) { ok = 0; break; }
    }
    CHECK(ok, "a safe assignment incorrectly flagged unsafe");

    TEST("Injecting NS+EW green is always detected as unsafe");
    ok = 1;
    for (int trial = 0; trial < 20; trial++) {
        for (int i = 0; i < NUM_DIRECTIONS; i++) T->light[i] = LIGHT_RED;
        T->light[DIR_NORTH] = LIGHT_GREEN;
        T->light[DIR_SOUTH] = LIGHT_GREEN;
        T->light[DIR_EAST]  = LIGHT_GREEN;
        T->emergency_active = 0; T->pedestrian_active = 0;
        if (state_is_safe(T) != 0) { ok = 0; break; }
    }
    CHECK(ok, "conflict not always detected");

    TEST("ALL-RED always safe regardless of flags");
    for (int i = 0; i < NUM_DIRECTIONS; i++) T->light[i] = LIGHT_RED;
    T->emergency_active   = 0;
    T->pedestrian_active  = 1;
    CHECK(state_is_safe(T) == 1, "all-red with ped_active should be safe");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SUITE H — Config loaded into shared memory
 * ═════════════════════════════════════════════════════════════════════════ */
static void suite_config_to_shm(void)
{
    printf("\n[Suite H] Config → shared memory\n");

    const char *cfgpath = "/tmp/test_integ_cfg.txt";
    FILE *f = fopen(cfgpath, "w");
    if (!f) { printf("  [SKIP] cannot write temp file\n"); return; }
    fprintf(f, "T_GREEN_MIN = 9\n");
    fprintf(f, "T_YELLOW = 4\n");
    fprintf(f, "T_ALL_RED = 2\n");
    fprintf(f, "VEHICLE_SPAWN_RATE = 30\n");
    fclose(f);

    memset(T, 0, sizeof(shared_state_t));
    load_config(cfgpath, T);
    unlink(cfgpath);

    TEST("t_green_min loaded from file into shared state");
    CHECK(T->t_green_min == 9, "t_green_min wrong");

    TEST("t_yellow loaded from file into shared state");
    CHECK(T->t_yellow == 4, "t_yellow wrong");

    TEST("t_all_red loaded from file into shared state");
    CHECK(T->t_all_red == 2, "t_all_red wrong");

    TEST("vehicle_spawn_rate loaded from file");
    CHECK(T->vehicle_spawn_rate == 30, "spawn rate wrong");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SUITE I — Emergency interrupt sequence (simulated)
 * ═════════════════════════════════════════════════════════════════════════ */
static void suite_emergency_seq(void)
{
    printf("\n[Suite I] Emergency interrupt sequence\n");

    /* Simulate: NS_GREEN → emergency arrives from EAST → ALL_RED → EMERG */
    memset(T, 0, sizeof(shared_state_t));
    for (int i = 0; i < NUM_DIRECTIONS; i++) T->light[i] = LIGHT_RED;
    T->light[DIR_NORTH] = LIGHT_GREEN;
    T->light[DIR_SOUTH] = LIGHT_GREEN;
    T->current_phase    = PHASE_NS_GREEN;

    TEST("Pre-emergency: NS_GREEN is safe");
    CHECK(state_is_safe(T) == 1, "pre-emerg unsafe");

    /* Step 1: transition to yellow */
    T->light[DIR_NORTH] = LIGHT_YELLOW;
    T->light[DIR_SOUTH] = LIGHT_YELLOW;
    TEST("Yellow transition before emergency is safe");
    CHECK(state_is_safe(T) == 1, "yellow transition unsafe");

    /* Step 2: all-red */
    for (int i = 0; i < NUM_DIRECTIONS; i++) T->light[i] = LIGHT_RED;
    TEST("All-red during emergency pre-clear is safe");
    CHECK(state_is_safe(T) == 1, "all-red unsafe");

    /* Step 3: emergency direction goes green */
    T->light[DIR_EAST] = LIGHT_GREEN;
    T->emergency_active    = 1;
    T->emergency_direction = DIR_EAST;
    T->current_phase       = PHASE_EMERGENCY;
    TEST("Emergency green (EAST only) is safe");
    CHECK(state_is_safe(T) == 1, "emergency green unsafe");

    /* Step 4: end of emergency */
    T->light[DIR_EAST] = LIGHT_YELLOW;
    TEST("Emergency yellow before return is safe");
    CHECK(state_is_safe(T) == 1, "emerg-yellow unsafe");

    for (int i = 0; i < NUM_DIRECTIONS; i++) T->light[i] = LIGHT_RED;
    T->emergency_active = 0;
    TEST("Post-emergency all-red is safe");
    CHECK(state_is_safe(T) == 1, "post-emerg unsafe");

    /* Event delivery */
    TEST("Emergency event queued and received correctly");
    evt_msg_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.mtype     = MTYPE_EVT_EMERGENCY;
    ev.direction = DIR_EAST;
    ev.priority  = 1;
    ev.timestamp = time(NULL);
    msgsnd(t_qevt, &ev, sizeof(ev) - sizeof(long), IPC_NOWAIT);
    evt_msg_t rx;
    ssize_t n = msgrcv(t_qevt, &rx, sizeof(rx) - sizeof(long),
                       0, IPC_NOWAIT);
    CHECK(n > 0 && rx.mtype == MTYPE_EVT_EMERGENCY &&
          rx.direction == DIR_EAST && rx.priority == 1, "emergency event wrong");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SUITE J — Pedestrian request lifecycle
 * ═════════════════════════════════════════════════════════════════════════ */
static void suite_pedestrian_lifecycle(void)
{
    printf("\n[Suite J] Pedestrian request lifecycle\n");

    memset(T, 0, sizeof(shared_state_t));
    for (int i = 0; i < NUM_DIRECTIONS; i++) T->light[i] = LIGHT_RED;

    TEST("Pedestrian event queued correctly");
    evt_msg_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.mtype     = MTYPE_EVT_PEDESTRIAN;
    ev.direction = DIR_NORTH;
    ev.timestamp = time(NULL);
    msgsnd(t_qevt, &ev, sizeof(ev) - sizeof(long), IPC_NOWAIT);
    evt_msg_t rx;
    ssize_t n = msgrcv(t_qevt, &rx, sizeof(rx) - sizeof(long),
                       0, IPC_NOWAIT);
    CHECK(n > 0 && rx.mtype == MTYPE_EVT_PEDESTRIAN &&
          rx.direction == DIR_NORTH, "ped event wrong");

    TEST("Pending pedestrian flag set in shared state");
    T->pedestrian_pending[DIR_NORTH] = 1;
    CHECK(T->pedestrian_pending[DIR_NORTH] == 1, "pending not set");

    TEST("Pedestrian phase active — no vehicle green — is safe");
    T->pedestrian_active = 1;
    T->emergency_active  = 0;
    for (int i = 0; i < NUM_DIRECTIONS; i++) T->light[i] = LIGHT_RED;
    CHECK(state_is_safe(T) == 1, "ped phase unsafe");

    TEST("Pedestrian remaining time can be tracked");
    T->pedestrian_remaining = 8;
    for (int i = 0; i < 8; i++) T->pedestrian_remaining--;
    CHECK(T->pedestrian_remaining == 0, "countdown wrong");

    TEST("Clearing ped flags after crossing");
    T->pedestrian_active             = 0;
    T->pedestrian_pending[DIR_NORTH] = 0;
    T->total_pedestrians_served++;
    CHECK(T->pedestrian_active == 0 &&
          T->total_pedestrians_served == 1, "clear failed");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  main
 * ═════════════════════════════════════════════════════════════════════════ */
int main(void)
{
    printf("==========================================================\n");
    printf("  Integration Tests — Real-Time Traffic Light System\n");
    printf("  Birzeit University  ENCS4330  Project #2\n");
    printf("==========================================================\n");

    ipc_create_all();

    suite_ipc_lifecycle();
    suite_shm_concurrent();
    suite_cmd_queue();
    suite_evt_queue();
    suite_log_queue();
    suite_phase_sm();
    suite_safety_stress();
    suite_config_to_shm();
    suite_emergency_seq();
    suite_pedestrian_lifecycle();

    ipc_destroy_all_test();

    printf("\n==========================================================\n");
    printf("  Results:  %d passed,  %d failed\n", g_passed, g_failed);
    printf("==========================================================\n");

    return (g_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}

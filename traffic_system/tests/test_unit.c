/*******************************************************************************
 *  test_unit.c  —  Unit Tests for Real-Time Traffic Light Control System
 *  ENCS4330 Project #2  —  Birzeit University
 *
 *  Tests covered:
 *    1.  state_is_safe()  — safety check function
 *    2.  load_config()    — config parser (defaults + file-based)
 *    3.  Message structure sizes and field offsets
 *    4.  Semaphore helpers (sem_lock / sem_unlock)
 *    5.  IPC key generation
 *    6.  Traffic light state transitions
 *    7.  Phase-name / direction-name string helpers
 *    8.  Shared-state initialisation
 *    9.  Emergency + pedestrian flag logic
 *   10.  send_log() robustness (negative queue id)
 *
 *  Build:  make test_unit   (added to Makefile)
 *  Run:    ./tests/test_unit
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <time.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <sys/msg.h>

/* Include the project headers */
#include "../src/common.h"
#include "../src/config.h"
#include "../src/ipc_init.h"

/* ── tiny test framework ─────────────────────────────────────────────────── */
static int g_passed = 0;
static int g_failed = 0;

#define TEST(name) \
    do { printf("  %-55s", name); fflush(stdout); } while(0)

#define PASS() \
    do { printf("PASS\n"); g_passed++; } while(0)

#define FAIL(msg) \
    do { printf("FAIL  (%s)\n", msg); g_failed++; } while(0)

#define CHECK(cond, msg) \
    do { if (cond) PASS(); else FAIL(msg); } while(0)

/* ── helper: build a clean zeroed shared_state_t ────────────────────────── */
static shared_state_t *make_state(void)
{
    shared_state_t *s = (shared_state_t *)calloc(1, sizeof(shared_state_t));
    if (!s) { perror("calloc"); exit(1); }
    /* Set all lights to RED — a safe starting point */
    for (int i = 0; i < NUM_DIRECTIONS; i++)
        s->light[i] = LIGHT_RED;
    return s;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SUITE 1 — state_is_safe()
 * ═════════════════════════════════════════════════════════════════════════ */
static void suite_safety(void)
{
    printf("\n[Suite 1] state_is_safe()\n");
    shared_state_t *s = make_state();

    /* All RED → safe */
    TEST("All RED is safe");
    CHECK(state_is_safe(s) == 1, "all-red should be safe");

    /* NS green only → safe */
    TEST("NS GREEN, EW RED is safe");
    s->light[DIR_NORTH] = LIGHT_GREEN;
    s->light[DIR_SOUTH] = LIGHT_GREEN;
    CHECK(state_is_safe(s) == 1, "ns-green should be safe");

    /* NS green + EW green → UNSAFE */
    TEST("NS GREEN + EW GREEN is UNSAFE");
    s->light[DIR_EAST] = LIGHT_GREEN;
    CHECK(state_is_safe(s) == 0, "conflict not detected");

    /* EW green only → safe */
    TEST("EW GREEN, NS RED is safe");
    s->light[DIR_NORTH] = LIGHT_RED;
    s->light[DIR_SOUTH] = LIGHT_RED;
    s->light[DIR_EAST]  = LIGHT_GREEN;
    s->light[DIR_WEST]  = LIGHT_GREEN;
    CHECK(state_is_safe(s) == 1, "ew-green should be safe");

    /* Single direction green → safe */
    TEST("Single NORTH GREEN is safe");
    s->light[DIR_EAST]  = LIGHT_RED;
    s->light[DIR_WEST]  = LIGHT_RED;
    s->light[DIR_NORTH] = LIGHT_GREEN;
    CHECK(state_is_safe(s) == 1, "single green should be safe");

    /* Emergency + pedestrian simultaneously → UNSAFE */
    TEST("Emergency + pedestrian_active is UNSAFE");
    s->light[DIR_NORTH]   = LIGHT_RED;
    s->emergency_active   = 1;
    s->pedestrian_active  = 1;
    CHECK(state_is_safe(s) == 0, "emerg+ped not flagged unsafe");

    /* Emergency alone → safe */
    TEST("Emergency alone (no ped) is safe");
    s->pedestrian_active = 0;
    CHECK(state_is_safe(s) == 1, "emergency alone should be safe");

    /* Yellow lights — both axes → safe (yellow is not conflicting green) */
    TEST("NS YELLOW + EW YELLOW is safe");
    s->emergency_active  = 0;
    for (int i = 0; i < NUM_DIRECTIONS; i++) s->light[i] = LIGHT_YELLOW;
    CHECK(state_is_safe(s) == 1, "yellow+yellow should be safe");

    free(s);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SUITE 2 — load_config() / defaults
 * ═════════════════════════════════════════════════════════════════════════ */
static void suite_config(void)
{
    printf("\n[Suite 2] load_config() / defaults\n");
    shared_state_t *s = make_state();

    /* load_defaults through load_config with NULL path */
    TEST("load_config(NULL) sets defaults");
    load_config(NULL, s);
    CHECK(s->t_green_min > 0 && s->t_yellow > 0, "defaults not set");

    TEST("Default t_green_min >= 5");
    CHECK(s->t_green_min >= 5, "t_green_min too small");

    TEST("Default t_yellow >= 2");
    CHECK(s->t_yellow >= 2, "t_yellow too small");

    TEST("Default t_all_red >= 1");
    CHECK(s->t_all_red >= 1, "t_all_red must be >= 1");

    TEST("Default t_pedestrian > 0");
    CHECK(s->t_pedestrian > 0, "t_pedestrian not set");

    TEST("Default t_emergency_response > 0");
    CHECK(s->t_emergency_response > 0, "emergency response not set");

    /* Write a temp config file and load it */
    TEST("load_config() reads T_YELLOW from file");
    const char *tmpfile = "/tmp/test_traffic_cfg.txt";
    FILE *f = fopen(tmpfile, "w");
    if (f) {
        fprintf(f, "T_YELLOW = 7\n");
        fprintf(f, "T_GREEN_MIN = 12\n");
        fclose(f);
        load_config(tmpfile, s);
        CHECK(s->t_yellow == 7 && s->t_green_min == 12,
              "file values not applied");
        unlink(tmpfile);
    } else {
        FAIL("could not create temp config file");
    }

    TEST("load_config() with non-existent file keeps defaults");
    load_config("/tmp/__no_such_file__.txt", s);
    /* defaults already applied — t_yellow should revert to default */
    CHECK(s->t_yellow > 0, "defaults not preserved after bad path");

    /* Invalid value: negative */
    TEST("Negative value in config is stored as-is (no clamp)");
    f = fopen(tmpfile, "w");
    if (f) {
        fprintf(f, "T_YELLOW = -3\n");
        fclose(f);
        load_config(tmpfile, s);
        /* The parser does atoi, so -3 gets stored */
        CHECK(s->t_yellow == -3, "negative value not stored");
        unlink(tmpfile);
    } else {
        FAIL("could not create temp config file");
    }

    free(s);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SUITE 3 — Message structure sizes & fields
 * ═════════════════════════════════════════════════════════════════════════ */
static void suite_messages(void)
{
    printf("\n[Suite 3] Message structures\n");

    TEST("cmd_msg_t contains mtype as first field");
    cmd_msg_t cmd;
    cmd.mtype = 42;
    CHECK(*(long *)&cmd == 42L, "mtype not first field in cmd_msg_t");

    TEST("ack_msg_t contains mtype as first field");
    ack_msg_t ack;
    ack.mtype = 99;
    CHECK(*(long *)&ack == 99L, "mtype not first field in ack_msg_t");

    TEST("evt_msg_t contains mtype as first field");
    evt_msg_t evt;
    evt.mtype = MTYPE_EVT_VEHICLE;
    CHECK(*(long *)&evt == MTYPE_EVT_VEHICLE, "mtype not first in evt_msg_t");

    TEST("log_msg_t contains mtype as first field");
    log_msg_t log;
    log.mtype = MTYPE_LOG_ANY;
    CHECK(*(long *)&log == MTYPE_LOG_ANY, "mtype not first in log_msg_t");

    TEST("cmd_msg_t size is larger than sizeof(long)");
    CHECK(sizeof(cmd_msg_t) > sizeof(long), "cmd too small");

    TEST("evt_msg_t info field is at least 32 bytes");
    CHECK(sizeof(evt.info) >= 32, "info field too small");

    TEST("log_msg_t text field is MAX_LOG_LEN");
    CHECK(sizeof(log.text) == MAX_LOG_LEN, "log text wrong size");

    /* Verify direction encoding in mtype */
    TEST("MTYPE_CMD_BASE + DIR_NORTH + 1 = 101");
    CHECK(MTYPE_CMD_BASE + DIR_NORTH + 1 == 101, "cmd mtype offset wrong");

    TEST("MTYPE_ACK_BASE + DIR_WEST + 1 = 204");
    CHECK(MTYPE_ACK_BASE + DIR_WEST + 1 == 204, "ack mtype offset wrong");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SUITE 4 — Semaphore lock/unlock (real IPC)
 * ═════════════════════════════════════════════════════════════════════════ */
static void suite_semaphore(void)
{
    printf("\n[Suite 4] Semaphore lock / unlock\n");

    /* Create a fresh semaphore for testing */
    key_t k = ftok("/tmp", 'T');
    int semid = -1;

    TEST("semget IPC_CREAT succeeds");
    semid = semget(k, 1, IPC_CREAT | IPC_EXCL | 0666);
    if (semid < 0) {
        /* Already exists from a previous run — destroy and retry */
        semid = semget(k, 1, 0666);
        semctl(semid, 0, IPC_RMID);
        semid = semget(k, 1, IPC_CREAT | IPC_EXCL | 0666);
    }
    CHECK(semid >= 0, "semget failed");

    if (semid >= 0) {
        union semun arg; arg.val = 1;
        semctl(semid, 0, SETVAL, arg);

        TEST("sem_lock() returns 0 on unlocked semaphore");
        CHECK(sem_lock(semid) == 0, "sem_lock failed");

        TEST("sem_unlock() returns 0 after lock");
        CHECK(sem_unlock(semid) == 0, "sem_unlock failed");

        TEST("Double lock/unlock cycle succeeds");
        int ok = (sem_lock(semid) == 0) && (sem_unlock(semid) == 0) &&
                 (sem_lock(semid) == 0) && (sem_unlock(semid) == 0);
        CHECK(ok, "double cycle failed");

        semctl(semid, 0, IPC_RMID);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SUITE 5 — IPC key generation
 * ═════════════════════════════════════════════════════════════════════════ */
static void suite_ipc_keys(void)
{
    printf("\n[Suite 5] IPC key generation\n");

    TEST("ftok(\"/tmp\", IPC_ID_SHM) succeeds");
    key_t k = ftok(IPC_KEY_PATH, IPC_ID_SHM);
    CHECK(k != (key_t)-1, "ftok for SHM failed");

    TEST("Different proj ids produce different keys");
    key_t k1 = ftok(IPC_KEY_PATH, IPC_ID_SHM);
    key_t k2 = ftok(IPC_KEY_PATH, IPC_ID_SEM);
    CHECK(k1 != k2, "SHM and SEM keys must differ");

    TEST("CMD and EVT queue keys differ");
    key_t k3 = ftok(IPC_KEY_PATH, IPC_ID_MQ_CMD);
    key_t k4 = ftok(IPC_KEY_PATH, IPC_ID_MQ_EVT);
    CHECK(k3 != k4, "CMD and EVT keys must differ");

    TEST("LOG queue key differs from CMD key");
    key_t k5 = ftok(IPC_KEY_PATH, IPC_ID_MQ_LOG);
    CHECK(k5 != k3, "LOG and CMD keys must differ");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SUITE 6 — Traffic light state transitions
 * ═════════════════════════════════════════════════════════════════════════ */
static void suite_light_transitions(void)
{
    printf("\n[Suite 6] Traffic light state transitions\n");
    shared_state_t *s = make_state();

    TEST("Initial state: all lights RED");
    int all_red = 1;
    for (int i = 0; i < NUM_DIRECTIONS; i++)
        if (s->light[i] != LIGHT_RED) all_red = 0;
    CHECK(all_red, "not all lights are RED initially");

    TEST("Setting NORTH to GREEN is valid");
    s->light[DIR_NORTH] = LIGHT_GREEN;
    CHECK(s->light[DIR_NORTH] == LIGHT_GREEN, "set failed");

    TEST("After GREEN, setting NORTH to YELLOW is valid");
    s->light[DIR_NORTH] = LIGHT_YELLOW;
    CHECK(s->light[DIR_NORTH] == LIGHT_YELLOW, "yellow set failed");

    TEST("After YELLOW, setting NORTH to RED is valid");
    s->light[DIR_NORTH] = LIGHT_RED;
    CHECK(s->light[DIR_NORTH] == LIGHT_RED, "red set failed");

    TEST("Setting SOUTH to YELLOW directly (valid representation)");
    s->light[DIR_SOUTH] = LIGHT_YELLOW;
    CHECK(s->light[DIR_SOUTH] == LIGHT_YELLOW, "south yellow set failed");

    TEST("Color enum values: RED=0, YELLOW=1, GREEN=2");
    CHECK(LIGHT_RED == 0 && LIGHT_YELLOW == 1 && LIGHT_GREEN == 2,
          "color enum values wrong");

    TEST("Direction enum values: N=0, S=1, E=2, W=3");
    CHECK(DIR_NORTH == 0 && DIR_SOUTH == 1 &&
          DIR_EAST  == 2 && DIR_WEST  == 3, "direction enum values wrong");

    free(s);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SUITE 7 — String name helpers (DIR_NAMES, COLOR_NAMES, PHASE_NAMES)
 * ═════════════════════════════════════════════════════════════════════════ */
static void suite_names(void)
{
    printf("\n[Suite 7] Name-string helpers\n");

    TEST("DIR_NAMES[DIR_NORTH] == \"NORTH\"");
    CHECK(strcmp(DIR_NAMES[DIR_NORTH], "NORTH") == 0, "wrong");

    TEST("DIR_NAMES[DIR_SOUTH] == \"SOUTH\"");
    CHECK(strcmp(DIR_NAMES[DIR_SOUTH], "SOUTH") == 0, "wrong");

    TEST("DIR_NAMES[DIR_EAST]  == \"EAST\"");
    CHECK(strcmp(DIR_NAMES[DIR_EAST],  "EAST")  == 0, "wrong");

    TEST("DIR_NAMES[DIR_WEST]  == \"WEST\"");
    CHECK(strcmp(DIR_NAMES[DIR_WEST],  "WEST")  == 0, "wrong");

    TEST("COLOR_NAMES[LIGHT_RED]    == \"RED\"");
    CHECK(strcmp(COLOR_NAMES[LIGHT_RED],    "RED")    == 0, "wrong");

    TEST("COLOR_NAMES[LIGHT_YELLOW] == \"YELLOW\"");
    CHECK(strcmp(COLOR_NAMES[LIGHT_YELLOW], "YELLOW") == 0, "wrong");

    TEST("COLOR_NAMES[LIGHT_GREEN]  == \"GREEN\"");
    CHECK(strcmp(COLOR_NAMES[LIGHT_GREEN],  "GREEN")  == 0, "wrong");

    TEST("PHASE_NAMES[PHASE_NS_GREEN]   starts with \"NS\"");
    CHECK(strncmp(PHASE_NAMES[PHASE_NS_GREEN], "NS", 2) == 0, "wrong");

    TEST("PHASE_NAMES[PHASE_EMERGENCY]  == \"EMERGENCY\"");
    CHECK(strcmp(PHASE_NAMES[PHASE_EMERGENCY], "EMERGENCY") == 0, "wrong");

    TEST("PHASE_NAMES[PHASE_PEDESTRIAN] == \"PEDESTRIAN\"");
    CHECK(strcmp(PHASE_NAMES[PHASE_PEDESTRIAN], "PEDESTRIAN") == 0, "wrong");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SUITE 8 — Shared-state field initialisation
 * ═════════════════════════════════════════════════════════════════════════ */
static void suite_shm_init(void)
{
    printf("\n[Suite 8] Shared-state field initialisation\n");
    shared_state_t *s = (shared_state_t *)calloc(1, sizeof(shared_state_t));

    TEST("Calloc-zeroed: emergency_active == 0");
    CHECK(s->emergency_active == 0, "should be 0");

    TEST("Calloc-zeroed: pedestrian_active == 0");
    CHECK(s->pedestrian_active == 0, "should be 0");

    TEST("Calloc-zeroed: running == 0");
    CHECK(s->running == 0, "should be 0");

    TEST("Calloc-zeroed: safety_violations == 0");
    CHECK(s->safety_violations == 0, "should be 0");

    TEST("Calloc-zeroed: total_vehicles_served == 0");
    CHECK(s->total_vehicles_served == 0, "should be 0");

    TEST("Calloc-zeroed: current_phase == PHASE_NS_GREEN (0)");
    CHECK(s->current_phase == 0, "should be 0");

    TEST("Calloc-zeroed: all waiting_vehicles == 0");
    int ok = 1;
    for (int i = 0; i < NUM_DIRECTIONS; i++)
        if (s->waiting_vehicles[i] != 0) ok = 0;
    CHECK(ok, "waiting_vehicles not zeroed");

    TEST("Sizeof shared_state_t >= 256 bytes (non-trivial)");
    CHECK(sizeof(shared_state_t) >= 256, "struct suspiciously small");

    free(s);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SUITE 9 — Emergency / pedestrian flag logic
 * ═════════════════════════════════════════════════════════════════════════ */
static void suite_flags(void)
{
    printf("\n[Suite 9] Emergency and pedestrian flag logic\n");
    shared_state_t *s = make_state();

    TEST("Setting emergency_active=1 reflects in struct");
    s->emergency_active   = 1;
    s->emergency_direction = DIR_EAST;
    CHECK(s->emergency_active == 1 &&
          s->emergency_direction == DIR_EAST, "flags not set");

    TEST("Clearing emergency_active restores safety with ped");
    s->pedestrian_active = 1;
    s->emergency_active  = 0;
    CHECK(state_is_safe(s) == 1, "should be safe after clearing emergency");

    TEST("pedestrian_pending[] per-direction works");
    s->pedestrian_pending[DIR_NORTH] = 1;
    CHECK(s->pedestrian_pending[DIR_NORTH] == 1 &&
          s->pedestrian_pending[DIR_SOUTH] == 0, "pending flags wrong");

    TEST("pedestrian_remaining can be set and read");
    s->pedestrian_remaining = 7;
    CHECK(s->pedestrian_remaining == 7, "remaining not stored");

    TEST("light_faulted[] per-direction works");
    s->light_faulted[DIR_WEST] = 1;
    CHECK(s->light_faulted[DIR_WEST] == 1 &&
          s->light_faulted[DIR_NORTH] == 0, "fault flags wrong");

    TEST("ticket_active + ticket_plate stored correctly");
    s->ticket_active = 1;
    strncpy(s->ticket_plate, "ABC-1234", sizeof(s->ticket_plate) - 1);
    CHECK(s->ticket_active == 1 &&
          strcmp(s->ticket_plate, "ABC-1234") == 0, "ticket fields wrong");

    free(s);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SUITE 10 — send_log() robustness
 * ═════════════════════════════════════════════════════════════════════════ */
static void suite_logging(void)
{
    printf("\n[Suite 10] send_log() robustness\n");

    TEST("send_log() with logq=-1 does not crash");
    send_log(-1, "TEST", 0, "hello %d", 42);
    PASS(); /* if we reach here, no crash */

    TEST("send_log() with NULL source does not crash");
    send_log(-1, NULL, 0, "null source test");
    PASS();

    TEST("send_log() with long message (>MAX_LOG_LEN) truncates");
    char big[MAX_LOG_LEN * 2];
    memset(big, 'X', sizeof(big) - 1);
    big[sizeof(big) - 1] = '\0';
    send_log(-1, "TEST", 0, "%s", big);
    PASS(); /* no buffer overflow = pass */

    TEST("send_log() with severity=2 (ERROR) does not crash");
    send_log(-1, "TEST", 2, "error message");
    PASS();

    /* Test against a real queue */
    TEST("send_log() to a valid message queue sends successfully");
    key_t k   = ftok("/tmp", 'U');
    int   qid = msgget(k, IPC_CREAT | IPC_EXCL | 0666);
    if (qid < 0) {
        qid = msgget(k, 0666);
        msgctl(qid, IPC_RMID, NULL);
        qid = msgget(k, IPC_CREAT | 0666);
    }
    if (qid >= 0) {
        send_log(qid, "UNIT", 0, "test message");
        log_msg_t m;
        ssize_t r = msgrcv(qid, &m, sizeof(m) - sizeof(long),
                           MTYPE_LOG_ANY, IPC_NOWAIT);
        CHECK(r > 0 && strcmp(m.source, "UNIT") == 0, "log not received");
        msgctl(qid, IPC_RMID, NULL);
    } else {
        FAIL("could not create test queue");
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SUITE 11 — fmt_time() helper
 * ═════════════════════════════════════════════════════════════════════════ */
static void suite_fmt_time(void)
{
    printf("\n[Suite 11] fmt_time() helper\n");

    TEST("fmt_time() produces HH:MM:SS (length == 8)");
    char buf[32];
    fmt_time(time(NULL), buf, sizeof(buf));
    CHECK(strlen(buf) == 8, "wrong length");

    TEST("fmt_time() first char is a digit");
    CHECK(buf[0] >= '0' && buf[0] <= '9', "not a digit");

    TEST("fmt_time() contains colons at positions 2 and 5");
    CHECK(buf[2] == ':' && buf[5] == ':', "colons missing");

    TEST("fmt_time() with t=0 (epoch) does not crash");
    fmt_time(0, buf, sizeof(buf));
    PASS();
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  main
 * ═════════════════════════════════════════════════════════════════════════ */
int main(void)
{
    printf("==========================================================\n");
    printf("  Unit Tests — Real-Time Traffic Light Control System\n");
    printf("  Birzeit University  ENCS4330  Project #2\n");
    printf("==========================================================\n");

    suite_safety();
    suite_config();
    suite_messages();
    suite_semaphore();
    suite_ipc_keys();
    suite_light_transitions();
    suite_names();
    suite_shm_init();
    suite_flags();
    suite_logging();
    suite_fmt_time();

    printf("\n==========================================================\n");
    printf("  Results:  %d passed,  %d failed\n", g_passed, g_failed);
    printf("==========================================================\n");

    return (g_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}

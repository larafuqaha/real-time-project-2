/*******************************************************************************
 *  test_performance.c  —  Performance & Timing Tests
 *  ENCS4330 Project #2  —  Birzeit University
 *
 *  Measures and validates:
 *    P1 — IPC throughput  : messages per second on each queue
 *    P2 — Shared-memory   : latency of sem-protected read/write
 *    P3 — Safety check    : state_is_safe() throughput
 *    P4 — Config parse    : time to parse a large config file
 *    P5 — Message latency : round-trip time for a cmd+ack pair
 *    P6 — Semaphore overhead per lock/unlock cycle
 *    P7 — Concurrent IPC  : fork-based message throughput
 *    P8 — Queue saturation : behaviour near capacity
 *
 *  Each test prints: operation, iterations, total_ms, ops/sec, result
 *
 *  Build:  make test_performance
 *  Run:    ./tests/test_performance
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

/* ── timing ──────────────────────────────────────────────────────────────── */
typedef struct { struct timespec t; } stamp_t;

static void ts_now(stamp_t *s)
{
    clock_gettime(CLOCK_MONOTONIC, &s->t);
}

/* elapsed milliseconds */
static double ts_ms(const stamp_t *start, const stamp_t *end)
{
    double ds = (double)(end->t.tv_sec  - start->t.tv_sec)  * 1000.0;
    double dn = (double)(end->t.tv_nsec - start->t.tv_nsec) / 1e6;
    return ds + dn;
}

/* elapsed microseconds */
static double ts_us(const stamp_t *start, const stamp_t *end)
{
    return ts_ms(start, end) * 1000.0;
}

/* ── framework ───────────────────────────────────────────────────────────── */
static int g_passed = 0, g_failed = 0;

#define PERF_HEADER() \
    printf("  %-38s %8s %8s %12s  %s\n", \
           "Test", "Iters", "ms", "ops/sec", "Result")

#define PERF_ROW(name, iters, ms, ops_sec, pass) do { \
    printf("  %-38s %8d %8.2f %12.0f  %s\n", \
           name, (int)(iters), ms, ops_sec, (pass) ? "PASS" : "FAIL"); \
    if (pass) g_passed++; else g_failed++; \
} while(0)

/* ── IPC setup ───────────────────────────────────────────────────────────── */
static int p_shmid = -1, p_semid = -1;
static int p_qcmd  = -1, p_qevt  = -1, p_qlog  = -1;
static shared_state_t *P = NULL;

#define PKEY_PATH  "/tmp"
#define PKEY_SHM   'p'
#define PKEY_SEM   'q'
#define PKEY_CMD   'r'
#define PKEY_EVT   's'
#define PKEY_LOG   't'

static key_t pk(int id) { return ftok(PKEY_PATH, id); }

static void perf_ipc_create(void)
{
    /* clean up any stale resources */
    int tmp;
    if ((tmp = shmget(pk(PKEY_SHM), sizeof(shared_state_t), 0666)) >= 0)
        shmctl(tmp, IPC_RMID, NULL);
    if ((tmp = semget(pk(PKEY_SEM), 1, 0666)) >= 0)
        semctl(tmp, 0, IPC_RMID);
    if ((tmp = msgget(pk(PKEY_CMD), 0666)) >= 0) msgctl(tmp, IPC_RMID, NULL);
    if ((tmp = msgget(pk(PKEY_EVT), 0666)) >= 0) msgctl(tmp, IPC_RMID, NULL);
    if ((tmp = msgget(pk(PKEY_LOG), 0666)) >= 0) msgctl(tmp, IPC_RMID, NULL);

    p_shmid = shmget(pk(PKEY_SHM), sizeof(shared_state_t), IPC_CREAT | 0666);
    p_semid = semget(pk(PKEY_SEM), 1, IPC_CREAT | 0666);
    p_qcmd  = msgget(pk(PKEY_CMD), IPC_CREAT | 0666);
    p_qevt  = msgget(pk(PKEY_EVT), IPC_CREAT | 0666);
    p_qlog  = msgget(pk(PKEY_LOG), IPC_CREAT | 0666);

    if (p_shmid < 0 || p_semid < 0 ||
        p_qcmd < 0  || p_qevt < 0 || p_qlog < 0) {
        fprintf(stderr, "FATAL: perf IPC creation failed\n"); exit(1);
    }
    union semun arg; arg.val = 1;
    semctl(p_semid, 0, SETVAL, arg);
    P = (shared_state_t *)shmat(p_shmid, NULL, 0);
    if (P == (void *)-1) { perror("shmat"); exit(1); }
    memset(P, 0, sizeof(shared_state_t));
}

static void perf_ipc_destroy(void)
{
    if (P && P != (void *)-1) shmdt(P);
    if (p_qcmd  >= 0) msgctl(p_qcmd,  IPC_RMID, NULL);
    if (p_qevt  >= 0) msgctl(p_qevt,  IPC_RMID, NULL);
    if (p_qlog  >= 0) msgctl(p_qlog,  IPC_RMID, NULL);
    if (p_semid >= 0) semctl(p_semid, 0, IPC_RMID);
    if (p_shmid >= 0) shmctl(p_shmid, IPC_RMID, NULL);
}

/* ── drain a queue ───────────────────────────────────────────────────────── */
static void drain_q(int qid)
{
    char buf[1024];
    while (msgrcv(qid, buf, sizeof(buf) - sizeof(long), 0, IPC_NOWAIT) > 0) {}
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  P1 — Message queue throughput
 * ═════════════════════════════════════════════════════════════════════════ */
static void perf_msgq_throughput(void)
{
    printf("\n[P1] Message queue throughput\n");
    PERF_HEADER();

    const int N = 5000;
    evt_msg_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.mtype     = MTYPE_EVT_VEHICLE;
    ev.direction = DIR_NORTH;
    ev.count     = 1;
    ev.timestamp = time(NULL);

    /* Send N messages */
    stamp_t s, e;
    ts_now(&s);
    for (int i = 0; i < N; i++)
        msgsnd(p_qevt, &ev, sizeof(ev) - sizeof(long), IPC_NOWAIT);
    ts_now(&e);
    double send_ms  = ts_ms(&s, &e);
    double send_ops = (double)N / (send_ms / 1000.0);
    PERF_ROW("msgsnd (evt_msg_t)", N, send_ms, send_ops, send_ms < 3000.0);

    /* Receive N messages */
    ts_now(&s);
    int received = 0;
    while (msgrcv(p_qevt, &ev, sizeof(ev) - sizeof(long), 0, IPC_NOWAIT) > 0)
        received++;
    ts_now(&e);
    double recv_ms  = ts_ms(&s, &e);
    double recv_ops = received > 0 ? (double)received / (recv_ms / 1000.0) : 0;
    PERF_ROW("msgrcv (evt_msg_t)", received, recv_ms, recv_ops,
             received > 0 && recv_ms < 3000.0);

    /* cmd_msg_t (slightly larger) */
    cmd_msg_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.mtype = MTYPE_CMD_BASE + DIR_NORTH + 1;
    cmd.target_color = LIGHT_GREEN;

    ts_now(&s);
    for (int i = 0; i < N; i++)
        msgsnd(p_qcmd, &cmd, sizeof(cmd) - sizeof(long), IPC_NOWAIT);
    ts_now(&e);
    double cmd_ms = ts_ms(&s, &e);
    PERF_ROW("msgsnd (cmd_msg_t)", N, cmd_ms,
             (double)N / (cmd_ms / 1000.0), cmd_ms < 3000.0);
    drain_q(p_qcmd);

    /* log_msg_t (largest) */
    log_msg_t lm;
    memset(&lm, 0, sizeof(lm));
    lm.mtype = MTYPE_LOG_ANY;
    strcpy(lm.source, "PERF");
    snprintf(lm.text, sizeof(lm.text), "performance test message");

    ts_now(&s);
    for (int i = 0; i < N; i++)
        msgsnd(p_qlog, &lm, sizeof(lm) - sizeof(long), IPC_NOWAIT);
    ts_now(&e);
    double log_ms = ts_ms(&s, &e);
    PERF_ROW("msgsnd (log_msg_t)", N, log_ms,
             (double)N / (log_ms / 1000.0), log_ms < 3000.0);
    drain_q(p_qlog);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  P2 — Shared memory read/write latency
 * ═════════════════════════════════════════════════════════════════════════ */
static void perf_shm_latency(void)
{
    printf("\n[P2] Shared memory latency\n");
    PERF_HEADER();

    const int N_RAW = 100000;
    const int N_SEM = 10000;   /* semaphore calls are slower */
    stamp_t s, e;

    /* Raw write (no semaphore) */
    ts_now(&s);
    for (int i = 0; i < N_RAW; i++) {
        P->total_vehicles_served = i;
        P->phase_remaining       = i & 0xFF;
    }
    ts_now(&e);
    double raw_ms = ts_ms(&s, &e);
    PERF_ROW("Raw SHM write (no sem)", N_RAW, raw_ms,
             (double)N_RAW / (raw_ms / 1000.0), raw_ms < 1000.0);

    /* Raw read */
    volatile int dummy = 0;
    ts_now(&s);
    for (int i = 0; i < N_RAW; i++) {
        dummy += P->total_vehicles_served;
        dummy += P->light[DIR_NORTH];
    }
    ts_now(&e);
    double rrd_ms = ts_ms(&s, &e);
    PERF_ROW("Raw SHM read (no sem)", N_RAW, rrd_ms,
             (double)N_RAW / (rrd_ms / 1000.0), rrd_ms < 1000.0);
    (void)dummy;

    /* Write with semaphore */
    ts_now(&s);
    for (int i = 0; i < N_SEM; i++) {
        sem_lock(p_semid);
        P->total_vehicles_served = i;
        sem_unlock(p_semid);
    }
    ts_now(&e);
    double sem_ms = ts_ms(&s, &e);
    PERF_ROW("SHM write with sem_lock/unlock", N_SEM, sem_ms,
             (double)N_SEM / (sem_ms / 1000.0), sem_ms < 10000.0);

    /* Read under semaphore */
    volatile int dummy2 = 0;
    ts_now(&s);
    for (int i = 0; i < N_SEM; i++) {
        sem_lock(p_semid);
        dummy2 += P->total_vehicles_served;
        sem_unlock(p_semid);
    }
    ts_now(&e);
    double semr_ms = ts_ms(&s, &e);
    PERF_ROW("SHM read with sem_lock/unlock", N_SEM, semr_ms,
             (double)N_SEM / (semr_ms / 1000.0), semr_ms < 10000.0);
    (void)dummy2;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  P3 — state_is_safe() throughput
 * ═════════════════════════════════════════════════════════════════════════ */
static void perf_safety_check(void)
{
    printf("\n[P3] state_is_safe() throughput\n");
    PERF_HEADER();

    const int N = 200000;
    stamp_t s, e;

    /* Safe states */
    P->light[DIR_NORTH] = LIGHT_GREEN;
    P->light[DIR_SOUTH] = LIGHT_GREEN;
    P->light[DIR_EAST]  = LIGHT_RED;
    P->light[DIR_WEST]  = LIGHT_RED;
    P->emergency_active = 0; P->pedestrian_active = 0;

    volatile int res = 0;
    ts_now(&s);
    for (int i = 0; i < N; i++)
        res += state_is_safe(P);
    ts_now(&e);
    double safe_ms = ts_ms(&s, &e);
    PERF_ROW("state_is_safe() safe state", N, safe_ms,
             (double)N / (safe_ms / 1000.0),
             safe_ms < 1000.0 && res == N);

    /* Unsafe states (conflict) */
    P->light[DIR_EAST] = LIGHT_GREEN;
    res = 0;
    ts_now(&s);
    for (int i = 0; i < N; i++)
        res += state_is_safe(P);
    ts_now(&e);
    double unsafe_ms = ts_ms(&s, &e);
    PERF_ROW("state_is_safe() unsafe state", N, unsafe_ms,
             (double)N / (unsafe_ms / 1000.0),
             unsafe_ms < 1000.0 && res == 0);
    (void)res;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  P4 — Config file parsing speed
 * ═════════════════════════════════════════════════════════════════════════ */
static void perf_config_parse(void)
{
    printf("\n[P4] Config parse speed\n");
    PERF_HEADER();

    const char *cfgpath = "/tmp/perf_cfg.txt";
    FILE *f = fopen(cfgpath, "w");
    if (!f) { printf("  [SKIP] cannot write temp cfg\n"); return; }
    fprintf(f,
        "T_GREEN_MIN = 8\n"
        "T_GREEN_MAX = 15\n"
        "T_YELLOW = 3\n"
        "T_ALL_RED = 2\n"
        "T_PEDESTRIAN = 10\n"
        "T_PEDESTRIAN_MAX_WAIT = 45\n"
        "T_VEHICLE_MAX_WAIT = 60\n"
        "T_EMERGENCY_RESPONSE = 5\n"
        "T_EMERGENCY_MAX_HOLD = 15\n"
        "VEHICLE_SPAWN_RATE = 25\n"
        "PEDESTRIAN_SPAWN_RATE = 10\n"
        "EMERGENCY_SPAWN_RATE = 5\n"
        "# comment line ignored\n"
        "UNKNOWN_KEY = 999\n"
    );
    fclose(f);

    const int N = 1000;
    stamp_t s, e;
    shared_state_t tmp;

    ts_now(&s);
    for (int i = 0; i < N; i++)
        load_config(cfgpath, &tmp);
    ts_now(&e);
    double cfg_ms = ts_ms(&s, &e);
    unlink(cfgpath);

    PERF_ROW("load_config() per file", N, cfg_ms,
             (double)N / (cfg_ms / 1000.0), cfg_ms < 5000.0);

    /* Verify correctness */
    printf("  %-38s t_yellow=%d t_green_min=%d  %s\n",
           "Config values correct?", tmp.t_yellow, tmp.t_green_min,
           (tmp.t_yellow == 3 && tmp.t_green_min == 8) ? "PASS" : "FAIL");
    if (tmp.t_yellow == 3 && tmp.t_green_min == 8) g_passed++; else g_failed++;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  P5 — Round-trip latency (cmd → ack), non-blocking poll
 * ═════════════════════════════════════════════════════════════════════════ */
static void perf_roundtrip(void)
{
    printf("\n[P5] Command round-trip latency\n");
    PERF_HEADER();

    const int N = 200;
    double total_us = 0;
    int ok = 0;

    for (int i = 0; i < N; i++) {
        cmd_msg_t cmd;
        memset(&cmd, 0, sizeof(cmd));
        cmd.mtype        = MTYPE_CMD_BASE + DIR_NORTH + 1;
        cmd.direction    = DIR_NORTH;
        cmd.target_color = LIGHT_GREEN;
        cmd.cmd_id       = i;

        stamp_t s, e;
        ts_now(&s);
        msgsnd(p_qcmd, &cmd, sizeof(cmd) - sizeof(long), IPC_NOWAIT);

        /* Simulate light: receive cmd */
        cmd_msg_t rcmd;
        msgrcv(p_qcmd, &rcmd, sizeof(rcmd) - sizeof(long),
               MTYPE_CMD_BASE + DIR_NORTH + 1, 0);

        /* Simulate light: send ack */
        ack_msg_t ack;
        memset(&ack, 0, sizeof(ack));
        ack.mtype         = MTYPE_ACK_BASE + DIR_NORTH + 1;
        ack.direction     = DIR_NORTH;
        ack.current_color = LIGHT_GREEN;
        ack.cmd_id        = rcmd.cmd_id;
        ack.success       = 1;
        msgsnd(p_qcmd, &ack, sizeof(ack) - sizeof(long), IPC_NOWAIT);

        /* Receive ack with busy-poll (IPC_NOWAIT) */
        ack_msg_t rack;
        int retries = 0;
        while (msgrcv(p_qcmd, &rack, sizeof(rack) - sizeof(long),
                      MTYPE_ACK_BASE + DIR_NORTH + 1, IPC_NOWAIT) < 0) {
            if (errno != ENOMSG || ++retries > 1000) break;
            usleep(100);
        }
        ts_now(&e);

        if (rack.success && rack.cmd_id == i) ok++;
        total_us += ts_us(&s, &e);
    }

    double avg_us  = total_us / N;
    double rtt_ops = (double)N / (total_us / 1e6);
    PERF_ROW("cmd+ack round-trip", N, total_us / 1000.0,
             rtt_ops, avg_us < 20000.0 && ok == N);
    printf("  %-38s avg latency = %.1f µs\n", "", avg_us);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  P6 — Semaphore overhead
 * ═════════════════════════════════════════════════════════════════════════ */
static void perf_semaphore_overhead(void)
{
    printf("\n[P6] Semaphore lock/unlock overhead\n");
    PERF_HEADER();

    const int N = 5000;
    stamp_t s, e;

    ts_now(&s);
    for (int i = 0; i < N; i++) {
        sem_lock(p_semid);
        sem_unlock(p_semid);
    }
    ts_now(&e);

    double sem_ms  = ts_ms(&s, &e);
    double avg_us  = (sem_ms * 1000.0) / N;
    double ops_sec = (double)N / (sem_ms / 1000.0);

    PERF_ROW("sem_lock + sem_unlock cycle", N, sem_ms, ops_sec,
             avg_us < 500.0);  /* each cycle < 500 µs */
    printf("  %-38s avg per cycle = %.2f µs\n", "", avg_us);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  P7 — Concurrent message throughput (fork, with timeout guard)
 * ═════════════════════════════════════════════════════════════════════════ */
static void perf_concurrent_ipc(void)
{
    printf("\n[P7] Concurrent IPC (2 producers + parent consumer)\n");
    PERF_HEADER();

    const int PER_PROC = 100;
    drain_q(p_qevt);

    stamp_t s, e;
    ts_now(&s);

    pid_t p1 = fork();
    if (p1 == 0) {
        evt_msg_t ev;
        memset(&ev, 0, sizeof(ev));
        ev.mtype = MTYPE_EVT_VEHICLE; ev.direction = DIR_NORTH;
        for (int i = 0; i < PER_PROC; i++)
            msgsnd(p_qevt, &ev, sizeof(ev) - sizeof(long), IPC_NOWAIT);
        exit(0);
    }

    pid_t p2 = fork();
    if (p2 == 0) {
        evt_msg_t ev;
        memset(&ev, 0, sizeof(ev));
        ev.mtype = MTYPE_EVT_VEHICLE; ev.direction = DIR_SOUTH;
        for (int i = 0; i < PER_PROC; i++)
            msgsnd(p_qevt, &ev, sizeof(ev) - sizeof(long), IPC_NOWAIT);
        exit(0);
    }

    /* Wait with timeout */
    int wstatus;
    if (p1 > 0) { alarm(5); waitpid(p1, &wstatus, 0); alarm(0); }
    if (p2 > 0) { alarm(5); waitpid(p2, &wstatus, 0); alarm(0); }

    int cnt = 0;
    evt_msg_t ev;
    while (msgrcv(p_qevt, &ev, sizeof(ev) - sizeof(long), 0, IPC_NOWAIT) > 0)
        cnt++;

    ts_now(&e);
    double ms  = ts_ms(&s, &e);
    double ops = (cnt > 0) ? (double)cnt / (ms / 1000.0) : 0;
    PERF_ROW("2-producer concurrent send", cnt, ms, ops,
             cnt >= PER_PROC);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  P8 — Queue saturation behaviour
 * ═════════════════════════════════════════════════════════════════════════ */
static void perf_queue_saturation(void)
{
    printf("\n[P8] Queue saturation & drain\n");
    PERF_HEADER();

    drain_q(p_qevt);

    /* Fill until queue full (IPC_NOWAIT + EAGAIN) */
    evt_msg_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.mtype = MTYPE_EVT_VEHICLE;
    ev.direction = DIR_EAST;

    int sent = 0;
    stamp_t s, e;
    ts_now(&s);
    while (msgsnd(p_qevt, &ev, sizeof(ev) - sizeof(long), IPC_NOWAIT) == 0)
        sent++;
    ts_now(&e);
    double fill_ms = ts_ms(&s, &e);
    PERF_ROW("Fill queue to saturation", sent, fill_ms,
             sent > 0 ? (double)sent / (fill_ms / 1000.0) : 0, sent > 0);
    printf("  %-38s queue capacity ≈ %d messages\n", "", sent);

    /* Drain the saturated queue */
    int drained = 0;
    ts_now(&s);
    while (msgrcv(p_qevt, &ev, sizeof(ev) - sizeof(long), 0, IPC_NOWAIT) > 0)
        drained++;
    ts_now(&e);
    double drain_ms = ts_ms(&s, &e);
    PERF_ROW("Drain saturated queue", drained, drain_ms,
             drained > 0 ? (double)drained / (drain_ms / 1000.0) : 0,
             drained == sent);

    /* Verify queue is now empty */
    printf("  %-38s  %s\n",
           "Queue empty after drain",
           drained == sent ? "PASS" : "FAIL");
    if (drained == sent) g_passed++; else g_failed++;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Overall summary
 * ═════════════════════════════════════════════════════════════════════════ */
int main(void)
{
    printf("==========================================================\n");
    printf("  Performance Tests — Real-Time Traffic Light System\n");
    printf("  Birzeit University  ENCS4330  Project #2\n");
    printf("  (CLOCK_MONOTONIC, single core, no -O2 warmup)\n");
    printf("==========================================================\n");

    perf_ipc_create();

    perf_msgq_throughput();
    perf_shm_latency();
    perf_safety_check();
    perf_config_parse();
    perf_roundtrip();
    perf_semaphore_overhead();
    perf_concurrent_ipc();
    perf_queue_saturation();

    perf_ipc_destroy();

    printf("\n==========================================================\n");
    printf("  Overall:  %d passed,  %d failed\n", g_passed, g_failed);
    printf("==========================================================\n");

    return (g_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}

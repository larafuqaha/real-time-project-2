/*****************************************************************************
 *  main.c
 *  ---------------------------------------------------------------------------
 *  Top-level launcher.
 *
 *  Responsibilities
 *  ----------------
 *  1. Parse the configuration file passed on the command line.
 *  2. Create shared memory, semaphore set, and three message queues.
 *  3. Initialise the shared-memory blackboard with default + config values.
 *  4. Fork every component:
 *         - logger
 *         - 4 x traffic_light (one per direction)
 *         - controller
 *         - vehicle_detector
 *         - pedestrian
 *         - emergency
 *         - gui_opengl  (optional, started last)
 *  5. Wait for SIGINT/SIGTERM, then signal shutdown, reap children and
 *     destroy IPC objects so nothing is left in the kernel.
 *
 *  Usage:
 *      ./main <config-file> [--no-gui]
 *****************************************************************************/
#include "common.h"
#include "ipc_init.h"
#include "config.h"

#define MAX_CHILDREN 16

static pid_t children[MAX_CHILDREN];
static int   n_children = 0;
static int   shmid = -1, semid = -1, qcmd = -1, qevt = -1, qlog = -1;
static shared_state_t *S = NULL;

static volatile sig_atomic_t stop_flag = 0;
static void on_signal(int sig) { (void)sig; stop_flag = 1; }

static pid_t spawn(const char *path, char *const argv[])
{
    pid_t p = fork();
    if (p == 0) {
        execv(path, argv);
        perror("execv");
        _exit(127);
    }
    if (p < 0) { perror("fork"); return -1; }
    children[n_children++] = p;
    return p;
}

static void shutdown_all(void)
{
    if (S) {
        sem_lock(semid);
        S->running = 0;
        sem_unlock(semid);
    }
    printf("\n[MAIN] Stopping children...\n");

    /* polite SIGTERM */
    for (int i = 0; i < n_children; ++i)
        if (children[i] > 0) kill(children[i], SIGTERM);

    /* wait up to 3 s then SIGKILL stragglers */
    for (int wait_s = 0; wait_s < 3; ++wait_s) {
        int alive = 0;
        for (int i = 0; i < n_children; ++i) {
            if (children[i] <= 0) continue;
            int st;
            pid_t r = waitpid(children[i], &st, WNOHANG);
            if (r == 0) alive = 1;
            else        children[i] = -1;
        }
        if (!alive) break;
        sleep(1);
    }
    for (int i = 0; i < n_children; ++i)
        if (children[i] > 0) { kill(children[i], SIGKILL); waitpid(children[i], NULL, 0); }

    if (S) shmdt(S);
    ipc_destroy_all(shmid, semid, qcmd, qevt, qlog);
    printf("[MAIN] IPC destroyed. Bye.\n");
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr,
            "Usage: %s <config-file> [--no-gui]\n"
            "Example: %s config/system.cfg\n", argv[0], argv[0]);
        return 1;
    }
    int use_gui = 1;
    for (int i = 2; i < argc; ++i)
        if (!strcmp(argv[i], "--no-gui")) use_gui = 0;

    /* make sure ftok base file exists */
    int fd = open(IPC_KEY_PATH, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr,
                "Cannot stat %s — required for ftok().\n", IPC_KEY_PATH);
        return 1;
    }
    close(fd);

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    /* 1. create IPC resources --------------------------------------- */
    shmid = ipc_shm_create();   if (shmid < 0) { perror("shmget"); return 1; }
    semid = ipc_sem_create();   if (semid < 0) { perror("semget"); return 1; }
    qcmd  = ipc_q_create(IPC_ID_MQ_CMD);
    qevt  = ipc_q_create(IPC_ID_MQ_EVT);
    qlog  = ipc_q_create(IPC_ID_MQ_LOG);
    if (qcmd < 0 || qevt < 0 || qlog < 0) {
        perror("msgget"); shutdown_all(); return 1;
    }

    /* 2. attach shared memory and initialise ------------------------ */
    S = (shared_state_t *)shmat(shmid, NULL, 0);
    if (S == (void *)-1) { perror("shmat"); shutdown_all(); return 1; }
    memset(S, 0, sizeof(*S));
    if (load_config(argv[1], S) < 0) {
        fprintf(stderr,
            "[MAIN] WARNING: could not open '%s' — using defaults.\n", argv[1]);
    }
    S->running        = 1;
    S->current_phase  = PHASE_STARTUP;
    for (int i = 0; i < NUM_DIRECTIONS; ++i) S->light[i] = LIGHT_RED;

    printf("[MAIN] Configuration:\n");
    printf("       green_min=%d  green_max=%d  yellow=%d  all_red=%d\n",
           S->t_green_min, S->t_green_max, S->t_yellow, S->t_all_red);
    printf("       ped_dur=%d  ped_max_wait=%d  emerg_resp=%d\n",
           S->t_pedestrian, S->t_pedestrian_max_wait, S->t_emergency_response);
    printf("       spawn rates: veh=%d%%  ped=%d%%  emerg=%d/1000\n",
           S->vehicle_spawn_rate, S->pedestrian_spawn_rate,
           S->emergency_spawn_rate);
    printf("[MAIN] IPC ids: shm=%d sem=%d qcmd=%d qevt=%d qlog=%d\n",
           shmid, semid, qcmd, qevt, qlog);

    /* 3. fork children --------------------------------------------- */
    char *args0[] = { (char *)"./logger", (char *)"logs/system.log", NULL };
    spawn("./logger", args0);
    usleep(150 * 1000);

    char dnum[4];
    for (int i = 0; i < NUM_DIRECTIONS; ++i) {
        snprintf(dnum, sizeof(dnum), "%d", i);
        char *a[] = { (char *)"./traffic_light", dnum, NULL };
        spawn("./traffic_light", a);
    }

    char *a1[] = { (char *)"./controller", NULL };       spawn("./controller", a1);
    char *a2[] = { (char *)"./vehicle_detector", NULL }; spawn("./vehicle_detector", a2);
    char *a3[] = { (char *)"./pedestrian", NULL };       spawn("./pedestrian", a3);
    char *a4[] = { (char *)"./emergency", NULL };        spawn("./emergency", a4);

    if (use_gui) {
        char *a5[] = { (char *)"./gui_opengl", NULL };
        spawn("./gui_opengl", a5);
    }

    printf("[MAIN] %d processes launched. Ctrl-C to stop.\n", n_children);

    /* 4. main loop just supervises ---------------------------------- */
    while (!stop_flag) {
        int st;
        pid_t r = waitpid(-1, &st, WNOHANG);
        if (r > 0) {
            fprintf(stderr,
                "[MAIN] child %d exited (status=%d)\n", r, WEXITSTATUS(st));
            /* remove from list */
            for (int i = 0; i < n_children; ++i)
                if (children[i] == r) children[i] = -1;
        }
        /* also exit if everyone is gone */
        int alive = 0;
        for (int i = 0; i < n_children; ++i) if (children[i] > 0) alive = 1;
        if (!alive) break;
        sleep(1);
    }

    shutdown_all();
    return 0;
}

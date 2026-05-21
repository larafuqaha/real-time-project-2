/******************************************************************************
 * pedestrian_button.c
 *
 * Simulates pedestrian "WALK" button presses at each crosswalk. Posts
 * EVT_PED_REQUEST events. The control logic decides when it is safe to
 * actually grant the WALK signal.
 *****************************************************************************/
#include "common.h"

static volatile sig_atomic_t g_running = 1;
static void on_signal(int s) { UNUSED(s); g_running = 0; }

int main(void)
{
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    int shmid = get_shm(0);
    int evtid = get_evt_queue(0);
    int semid = get_sem(0);
    if (shmid < 0 || evtid < 0 || semid < 0) {
        fprintf(stderr, "pedestrian_button: IPC attach failed\n");
        return EXIT_FAILURE;
    }
    SharedState *S = (SharedState *)shmat(shmid, NULL, 0);
    if (S == (void *)-1) { perror("shmat"); return EXIT_FAILURE; }

    srand((unsigned)(time(NULL) ^ getpid()));

    log_event(S, semid, "PedestrianButton started (PID %d)", getpid());

    while (g_running && S->system_running) {
        usleep(500000); /* 500 ms tick */
        if (!S->cfg.enable_random_pedestrians) continue;

        int rate_ms = S->cfg.pedestrian_spawn_rate_ms;
        if (rate_ms < 500) rate_ms = 500;
        double p = 500.0 / (double)rate_ms;

        for (int d = 0; d < NUM_DIRS; d++) {
            sem_lock(semid);
            int pending = S->ped_request_pending[d];
            sem_unlock(semid);
            if (pending) continue;            /* already queued */
            if ((double)rand() / RAND_MAX < p / NUM_DIRS) {
                EventMsg e = {0};
                e.mtype = EVT_PED_REQUEST;
                e.direction = d;
                e.timestamp = time(NULL);
                snprintf(e.info, sizeof(e.info), "ped@%s", dir_name(d));
                msgsnd(evtid, &e, sizeof(e) - sizeof(long), IPC_NOWAIT);
            }
        }
    }

    log_event(S, semid, "PedestrianButton stopped");
    shmdt(S);
    return EXIT_SUCCESS;
}

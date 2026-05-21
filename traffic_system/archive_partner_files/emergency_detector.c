/******************************************************************************
 * emergency_detector.c
 *
 * Generates rare emergency-vehicle approach events. Posts EVT_EMERGENCY on
 * the event queue. The control logic preempts the schedule, runs a safe
 * yellow + all-red, then opens an emergency corridor for the requested
 * direction.
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
        fprintf(stderr, "emergency_detector: IPC attach failed\n");
        return EXIT_FAILURE;
    }
    SharedState *S = (SharedState *)shmat(shmid, NULL, 0);
    if (S == (void *)-1) { perror("shmat"); return EXIT_FAILURE; }

    srand((unsigned)(time(NULL) ^ getpid()));

    log_event(S, semid, "EmergencyDetector started (PID %d)", getpid());

    while (g_running && S->system_running) {
        usleep(1000000); /* 1 s tick */
        if (!S->cfg.enable_random_emergencies) continue;

        int rate_ms = S->cfg.emergency_spawn_rate_ms;
        if (rate_ms < 5000) rate_ms = 5000;
        double p = 1000.0 / (double)rate_ms;

        sem_lock(semid);
        int active = S->emergency_active;
        sem_unlock(semid);
        if (active) continue; /* already busy */

        if ((double)rand() / RAND_MAX < p) {
            Direction d = (Direction)(rand() % NUM_DIRS);
            EventMsg e = {0};
            e.mtype = EVT_EMERGENCY;
            e.direction = d;
            e.timestamp = time(NULL);
            snprintf(e.info, sizeof(e.info), "emerg@%s", dir_name(d));
            msgsnd(evtid, &e, sizeof(e) - sizeof(long), IPC_NOWAIT);
        }
    }

    log_event(S, semid, "EmergencyDetector stopped");
    shmdt(S);
    return EXIT_SUCCESS;
}

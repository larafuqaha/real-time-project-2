/*****************************************************************************
 *  pedestrian.c
 *  ---------------------------------------------------------------------------
 *  Simulates pedestrians pressing crossing buttons.
 *  A pedestrian "presses the button" at a low rate (per direction, per
 *  second). The request is queued as an EVT_PEDESTRIAN event; the controller
 *  decides when it is safe to actually open the crosswalk.
 *****************************************************************************/
#include "common.h"
#include "ipc_init.h"

static volatile sig_atomic_t stop_flag = 0;
static void on_signal(int sig) { (void)sig; stop_flag = 1; }

int main(void)
{
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);
    srand((time(NULL) ^ getpid()) + 17);

    int shmid = ipc_shm_attach_existing();
    int semid = ipc_sem_attach_existing();
    int qevt  = ipc_q_attach_existing(IPC_ID_MQ_EVT);
    int qlog  = ipc_q_attach_existing(IPC_ID_MQ_LOG);
    if (shmid < 0 || semid < 0 || qevt < 0 || qlog < 0) {
        fprintf(stderr, "pedestrian: IPC attach failed\n");
        return 1;
    }
    shared_state_t *S = (shared_state_t *)shmat(shmid, NULL, 0);
    if (S == (void *)-1) { perror("shmat"); return 1; }

    send_log(qlog, "PEDEST", 0, "Pedestrian generator started (pid=%d)",
             getpid());

    while (!stop_flag) {
        sem_lock(semid);
        int running = S->running;
        int rate    = S->pedestrian_spawn_rate;
        sem_unlock(semid);
        if (!running) break;

        for (int i = 0; i < NUM_DIRECTIONS; ++i) {
            if ((rand() % 100) < rate) {
                evt_msg_t e;
                memset(&e, 0, sizeof(e));
                e.mtype     = MTYPE_EVT_PEDESTRIAN;
                e.direction = (direction_t)i;
                e.count     = 1;
                e.priority  = 0;
                e.timestamp = time(NULL);
                snprintf(e.info, sizeof(e.info), "button-%s", DIR_NAMES[i]);
                msgsnd(qevt, &e, sizeof(e) - sizeof(long), IPC_NOWAIT);
            }
        }
        sleep(1);
    }

    send_log(qlog, "PEDEST", 0, "Pedestrian generator exiting");
    shmdt(S);
    return 0;
}

/*****************************************************************************
 *  emergency.c
 *  ---------------------------------------------------------------------------
 *  Simulates the rare arrival of an emergency vehicle (ambulance, fire truck,
 *  police).  Sends an EVT_EMERGENCY event; the controller is required to
 *  safely interrupt the normal cycle.
 *****************************************************************************/
#include "common.h"
#include "ipc_init.h"

static volatile sig_atomic_t stop_flag = 0;
static void on_signal(int sig) { (void)sig; stop_flag = 1; }

int main(void)
{
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);
    srand((time(NULL) ^ getpid()) + 9001);

    int shmid = ipc_shm_attach_existing();
    int semid = ipc_sem_attach_existing();
    int qevt  = ipc_q_attach_existing(IPC_ID_MQ_EVT);
    int qlog  = ipc_q_attach_existing(IPC_ID_MQ_LOG);
    if (shmid < 0 || semid < 0 || qevt < 0 || qlog < 0) {
        fprintf(stderr, "emergency: IPC attach failed\n");
        return 1;
    }
    shared_state_t *S = (shared_state_t *)shmat(shmid, NULL, 0);
    if (S == (void *)-1) { perror("shmat"); return 1; }

    send_log(qlog, "EMERG", 0, "Emergency generator started (pid=%d)",
             getpid());

    while (!stop_flag) {
        sem_lock(semid);
        int running = S->running;
        int rate    = S->emergency_spawn_rate;
        int already = S->emergency_active;
        sem_unlock(semid);
        if (!running) break;

        if (!already && (rand() % 1000) < rate) {
            direction_t d = (direction_t)(rand() % NUM_DIRECTIONS);
            evt_msg_t e;
            memset(&e, 0, sizeof(e));
            e.mtype     = MTYPE_EVT_EMERGENCY;
            e.direction = d;
            e.count     = 1;
            e.priority  = 1;
            e.timestamp = time(NULL);
            snprintf(e.info, sizeof(e.info), "siren-detected-%s",
                     DIR_NAMES[d]);
            msgsnd(qevt, &e, sizeof(e) - sizeof(long), IPC_NOWAIT);
            send_log(qlog, "EMERG", 1,
                     "Generated EMERGENCY from %s", DIR_NAMES[d]);
        }
        sleep(1);
    }

    send_log(qlog, "EMERG", 0, "Emergency generator exiting");
    shmdt(S);
    return 0;
}

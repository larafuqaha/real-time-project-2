/*****************************************************************************
 *  vehicle_detector.c
 *  ---------------------------------------------------------------------------
 *  Simulates inductive-loop / camera vehicle detectors at each lane.
 *  Each second, for each direction, with probability `vehicle_spawn_rate`%
 *  it pushes a "vehicle arrived" event onto the EVENT queue.
 *  The controller is responsible for consuming the events; this process
 *  never writes to shared memory directly.
 *****************************************************************************/
#include "common.h"
#include "ipc_init.h"

static volatile sig_atomic_t stop_flag = 0;
static void on_signal(int sig) { (void)sig; stop_flag = 1; }

int main(void)
{
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);
    srand(time(NULL) ^ getpid());

    int shmid = ipc_shm_attach_existing();
    int semid = ipc_sem_attach_existing();
    int qevt  = ipc_q_attach_existing(IPC_ID_MQ_EVT);
    int qlog  = ipc_q_attach_existing(IPC_ID_MQ_LOG);
    if (shmid < 0 || semid < 0 || qevt < 0 || qlog < 0) {
        fprintf(stderr, "vehicle_detector: IPC attach failed\n");
        return 1;
    }
    shared_state_t *S = (shared_state_t *)shmat(shmid, NULL, 0);
    if (S == (void *)-1) { perror("shmat"); return 1; }

    send_log(qlog, "VEHICLE", 0, "Vehicle detector started (pid=%d)", getpid());

    while (!stop_flag) {
        sem_lock(semid);
        int running = S->running;
        int rate    = S->vehicle_spawn_rate;
        sem_unlock(semid);
        if (!running) break;

        for (int i = 0; i < NUM_DIRECTIONS; ++i) {
            if ((rand() % 100) < rate) {
                evt_msg_t e;
                memset(&e, 0, sizeof(e));
                e.mtype     = MTYPE_EVT_VEHICLE;
                e.direction = (direction_t)i;
                e.count     = 1 + (rand() % 2);
                e.priority  = 0;
                e.timestamp = time(NULL);
                snprintf(e.info, sizeof(e.info), "sensor-loop-%d", i);
                msgsnd(qevt, &e, sizeof(e) - sizeof(long), IPC_NOWAIT);
            }
        }
        sleep(1);
    }

    send_log(qlog, "VEHICLE", 0, "Vehicle detector exiting");
    shmdt(S);
    return 0;
}

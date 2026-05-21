/*****************************************************************************
 *  traffic_light.c
 *  ---------------------------------------------------------------------------
 *  One instance per direction (N/S/E/W).
 *
 *  Responsibilities
 *  ----------------
 *  - Wait on the command queue for messages addressed to this direction.
 *  - When a command arrives, update the corresponding entry in shared memory
 *    inside a critical section.
 *  - Send an acknowledgement back on the same queue (different mtype).
 *  - Simulate a small chance of a hardware fault, in which case the light
 *    reports failure and stays at the safest color (RED).
 *
 *  Usage: traffic_light <direction 0..3>
 *****************************************************************************/
#include "common.h"
#include "ipc_init.h"

static volatile sig_atomic_t stop_flag = 0;
static void on_signal(int sig) { (void)sig; stop_flag = 1; }

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s <0..3>\n", argv[0]); return 1; }
    int d = atoi(argv[1]);
    if (d < 0 || d >= NUM_DIRECTIONS) { fprintf(stderr, "bad direction\n"); return 1; }

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);
    srand(time(NULL) ^ (getpid() << 8));

    int shmid = ipc_shm_attach_existing();
    int semid = ipc_sem_attach_existing();
    int qcmd  = ipc_q_attach_existing(IPC_ID_MQ_CMD);
    int qlog  = ipc_q_attach_existing(IPC_ID_MQ_LOG);
    int qevt  = ipc_q_attach_existing(IPC_ID_MQ_EVT);
    if (shmid < 0 || semid < 0 || qcmd < 0 || qlog < 0 || qevt < 0) {
        fprintf(stderr, "light[%s]: IPC attach failed\n", DIR_NAMES[d]);
        return 1;
    }
    shared_state_t *S = (shared_state_t *)shmat(shmid, NULL, 0);
    if (S == (void *)-1) { perror("shmat"); return 1; }

    send_log(qlog, DIR_NAMES[d], 0, "Light process started (pid=%d)", getpid());

    cmd_msg_t cmd;
    while (!stop_flag) {
        /* check running */
        sem_lock(semid);
        int running = S->running;
        sem_unlock(semid);
        if (!running) break;

        /* receive only commands for this direction */
        ssize_t r = msgrcv(qcmd, &cmd, sizeof(cmd) - sizeof(long),
                           MTYPE_CMD_BASE + d + 1, IPC_NOWAIT);
        if (r < 0) {
            if (errno == ENOMSG) { usleep(100 * 1000); continue; }
            if (errno == EINTR)  continue;
            perror("msgrcv"); break;
        }

        /* Simulate a tiny hardware fault probability                     */
        int fault = (rand() % 1000) < 2;   /* 0.2 % */
        light_color_t actual = cmd.target_color;
        if (fault) actual = LIGHT_RED;      /* fail safe */

        sem_lock(semid);
        light_color_t prev = S->light[d];
        S->light[d]        = actual;
        S->last_change[d]  = time(NULL);
        sem_unlock(semid);

        ack_msg_t a;
        memset(&a, 0, sizeof(a));
        a.mtype         = MTYPE_ACK_BASE + d + 1;
        a.direction     = d;
        a.current_color = actual;
        a.cmd_id        = cmd.cmd_id;
        a.success       = fault ? 0 : 1;
        a.timestamp     = time(NULL);
        msgsnd(qcmd, &a, sizeof(a) - sizeof(long), IPC_NOWAIT);

        if (fault) {
            evt_msg_t e;
            memset(&e, 0, sizeof(e));
            e.mtype     = MTYPE_EVT_FAULT;
            e.direction = d;
            e.timestamp = time(NULL);
            snprintf(e.info, sizeof(e.info), "lamp stuck — forced RED");
            msgsnd(qevt, &e, sizeof(e) - sizeof(long), IPC_NOWAIT);
            send_log(qlog, DIR_NAMES[d], 2, "FAULT — forced RED instead of %s",
                     COLOR_NAMES[cmd.target_color]);
        } else if (prev != actual) {
            send_log(qlog, DIR_NAMES[d], 0, "Light %s -> %s",
                     COLOR_NAMES[prev], COLOR_NAMES[actual]);
        }
    }

    send_log(qlog, DIR_NAMES[d], 0, "Light process exiting");
    shmdt(S);
    return 0;
}

/*****************************************************************************
 *  logger.c
 *  ---------------------------------------------------------------------------
 *  Drains the log message queue and writes every entry to a log file
 *  (logs/system.log) and optionally to stderr.
 *
 *  Each log entry contains:
 *      - Wall-clock timestamp (formatted HH:MM:SS)
 *      - Severity tag  (INFO | WARN | ERROR)
 *      - Source string (CTRL, NORTH, EMERG, ...)
 *      - Description text
 *
 *  This isolates I/O latency from the real-time loops.
 *****************************************************************************/
#include "common.h"
#include "ipc_init.h"

static volatile sig_atomic_t stop_flag = 0;
static void on_signal(int sig) { (void)sig; stop_flag = 1; }

static const char *SEV[3] = {"INFO ", "WARN ", "ERROR"};

int main(int argc, char **argv)
{
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    const char *path = (argc > 1) ? argv[1] : "logs/system.log";
    FILE *fp = fopen(path, "a");
    if (!fp) { perror("logger: open"); return 1; }
    setvbuf(fp, NULL, _IOLBF, 0);   /* line-buffered */

    int shmid = ipc_shm_attach_existing();
    int semid = ipc_sem_attach_existing();
    int qlog  = ipc_q_attach_existing(IPC_ID_MQ_LOG);
    if (shmid < 0 || semid < 0 || qlog < 0) {
        fprintf(stderr, "logger: IPC attach failed\n");
        return 1;
    }
    shared_state_t *S = (shared_state_t *)shmat(shmid, NULL, 0);
    if (S == (void *)-1) { perror("shmat"); return 1; }

    fprintf(fp, "\n===== System started =====\n");
    fflush(fp);

    log_msg_t m;
    while (!stop_flag) {
        sem_lock(semid);
        int running = S->running;
        sem_unlock(semid);

        ssize_t r = msgrcv(qlog, &m, sizeof(m) - sizeof(long), 0, IPC_NOWAIT);
        if (r > 0) {
            char ts[16]; fmt_time(m.timestamp, ts, sizeof(ts));
            int sev = (m.severity < 0 || m.severity > 2) ? 0 : m.severity;
            fprintf(fp, "[%s] %s %-7s %s\n",
                    ts, SEV[sev], m.source, m.text);
        } else {
            if (!running) {
                /* drain remaining and exit */
                while (msgrcv(qlog, &m, sizeof(m) - sizeof(long), 0,
                              IPC_NOWAIT) > 0) {
                    char ts[16]; fmt_time(m.timestamp, ts, sizeof(ts));
                    int sev = (m.severity < 0 || m.severity > 2) ? 0 : m.severity;
                    fprintf(fp, "[%s] %s %-7s %s\n",
                            ts, SEV[sev], m.source, m.text);
                }
                break;
            }
            usleep(50 * 1000);
        }
    }

    fprintf(fp, "===== System stopped =====\n\n");
    fclose(fp);
    shmdt(S);
    return 0;
}

/*****************************************************************************
 *  ipc_init.h
 *  Centralised IPC creation and teardown.  Every process uses these helpers
 *  to obtain the same identifiers based on ftok() of /tmp + a project id.
 *****************************************************************************/
#ifndef IPC_INIT_H
#define IPC_INIT_H

#include "common.h"

/* ----- key helpers --------------------------------------------------- */
static inline key_t k_shm(void)   { return ftok(IPC_KEY_PATH, IPC_ID_SHM);    }
static inline key_t k_sem(void)   { return ftok(IPC_KEY_PATH, IPC_ID_SEM);    }
static inline key_t k_qcmd(void)  { return ftok(IPC_KEY_PATH, IPC_ID_MQ_CMD); }
static inline key_t k_qevt(void)  { return ftok(IPC_KEY_PATH, IPC_ID_MQ_EVT); }
static inline key_t k_qlog(void)  { return ftok(IPC_KEY_PATH, IPC_ID_MQ_LOG); }

/* ----- create-or-get for each resource ------------------------------- */
static inline int ipc_shm_create(void)
{
    int id = shmget(k_shm(), sizeof(shared_state_t), IPC_CREAT | 0666);
    return id;
}
static inline int ipc_shm_attach_existing(void)
{
    return shmget(k_shm(), sizeof(shared_state_t), 0666);
}

static inline int ipc_sem_create(void)
{
    int id = semget(k_sem(), SEM_COUNT, IPC_CREAT | 0666);
    if (id < 0) return -1;
    /* Initialise to 1 (mutex unlocked). Only the creator does this. */
    union semun arg;
    arg.val = 1;
    if (semctl(id, SEM_MUTEX, SETVAL, arg) < 0) return -1;
    return id;
}
static inline int ipc_sem_attach_existing(void)
{
    return semget(k_sem(), SEM_COUNT, 0666);
}

static inline int ipc_q_create(int id_char)
{
    key_t k = ftok(IPC_KEY_PATH, id_char);
    return msgget(k, IPC_CREAT | 0666);
}
static inline int ipc_q_attach_existing(int id_char)
{
    key_t k = ftok(IPC_KEY_PATH, id_char);
    return msgget(k, 0666);
}

/* ----- teardown ------------------------------------------------------ */
static inline void ipc_destroy_all(int shmid, int semid,
                                   int qcmd, int qevt, int qlog)
{
    if (qcmd >= 0) msgctl(qcmd, IPC_RMID, NULL);
    if (qevt >= 0) msgctl(qevt, IPC_RMID, NULL);
    if (qlog >= 0) msgctl(qlog, IPC_RMID, NULL);
    if (semid >= 0) semctl(semid, 0, IPC_RMID);
    if (shmid >= 0) shmctl(shmid, IPC_RMID, NULL);
}

#endif /* IPC_INIT_H */

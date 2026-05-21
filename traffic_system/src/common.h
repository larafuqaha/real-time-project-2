/*****************************************************************************
 *  common.h
 *  ---------------------------------------------------------------------------
 *  Birzeit University - ENCS4330 - Project #2
 *  Real-Time Traffic Light Control System Using IPC
 *
 *  This header is shared by every process in the system. It defines:
 *      - The directions of the intersection (N, S, E, W)
 *      - The possible light colors and traffic phases
 *      - The structure stored in shared memory (the "blackboard")
 *      - The structures of every message exchanged via message queues
 *      - The IPC keys (ftok) used by all processes
 *      - Helper macros for logging and time
 *
 *  Why these IPC choices?
 *  ----------------------
 *  - SHARED MEMORY is used for the global "state of the world" (light colors,
 *    detected vehicles, pending pedestrians, emergency flag, timers). Every
 *    process needs to *read* this state frequently and cheaply; copying it
 *    around in messages would waste CPU and introduce staleness.
 *
 *  - A SEMAPHORE protects the shared memory against race conditions. Without
 *    it, two processes could read/modify the state simultaneously, producing
 *    inconsistent values (e.g. two greens at once).
 *
 *  - MESSAGE QUEUES are used for *events* and *commands*. Events (vehicle
 *    arrived, pedestrian pressed button, emergency detected) are produced
 *    asynchronously and must not be lost; queues give us a natural FIFO
 *    buffer. Commands from the controller to each light are also messages,
 *    because they are one-shot directives that should be acknowledged.
 *
 *  - A separate LOG QUEUE is used so that any process can fire-and-forget
 *    log entries to the logger without blocking on file I/O.
 *
 *  Author: ENCS4330 Project Team
 *****************************************************************************/
#ifndef COMMON_H
#define COMMON_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <sys/msg.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <fcntl.h>

/* ------------------------------------------------------------------ */
/*  Project-wide constants                                            */
/* ------------------------------------------------------------------ */
#define NUM_DIRECTIONS      4         /* N, S, E, W                   */
#define MAX_QUEUE_VEHICLES  99        /* per-lane logical max         */
#define MAX_LOG_LEN         256       /* per log line                 */
#define MAX_PATH_LEN        256

/* IPC keys: we generate them with ftok() in ipc_init.c, but we keep
 * symbolic project IDs here so that every process uses the same ones. */
#define IPC_KEY_PATH        "/tmp"    /* must exist on the file system */
#define IPC_ID_SHM          'S'       /* shared memory blackboard     */
#define IPC_ID_SEM          'M'       /* mutex semaphore set          */
#define IPC_ID_MQ_CMD       'C'       /* controller -> light commands */
#define IPC_ID_MQ_EVT       'E'       /* sensors    -> controller     */
#define IPC_ID_MQ_LOG       'L'       /* anyone     -> logger         */

/* Message type ranges (msgsnd/msgrcv mtype).  Using direction+offset
 * lets a light process receive only its own commands. */
#define MTYPE_CMD_BASE      100       /* +direction (1..4)            */
#define MTYPE_ACK_BASE      200       /* light -> controller ack      */
#define MTYPE_EVT_VEHICLE   300
#define MTYPE_EVT_PEDESTRIAN 301
#define MTYPE_EVT_EMERGENCY 302
#define MTYPE_EVT_FAULT     303
#define MTYPE_EVT_TICKET    304   /* GUI -> CTRL: red-light runner detected */
#define MTYPE_EVT_TICKET_DONE 305 /* GUI -> CTRL: police chase finished     */
#define MTYPE_EVT_FAULT_INJECT 306 /* GUI -> CTRL: force a light to fault   */
#define MTYPE_EVT_FAULT_CLEAR  307 /* GUI -> CTRL: clear a fault            */
#define MTYPE_LOG_ANY       400

/* ------------------------------------------------------------------ */
/*  Enumerations                                                      */
/* ------------------------------------------------------------------ */
typedef enum {
    DIR_NORTH = 0,
    DIR_SOUTH = 1,
    DIR_EAST  = 2,
    DIR_WEST  = 3
} direction_t;

static const char *DIR_NAMES[NUM_DIRECTIONS] __attribute__((unused)) =
    {"NORTH", "SOUTH", "EAST", "WEST"};

typedef enum {
    LIGHT_RED    = 0,
    LIGHT_YELLOW = 1,
    LIGHT_GREEN  = 2
} light_color_t;

static const char *COLOR_NAMES[3] __attribute__((unused)) = {"RED", "YELLOW", "GREEN"};

typedef enum {
    PHASE_NS_GREEN     = 0,    /* North/South go, East/West stop      */
    PHASE_NS_YELLOW    = 1,
    PHASE_ALL_RED_1    = 2,    /* clearance interval                  */
    PHASE_EW_GREEN     = 3,
    PHASE_EW_YELLOW    = 4,
    PHASE_ALL_RED_2    = 5,
    PHASE_PEDESTRIAN   = 6,    /* all vehicles red, pedestrians walk  */
    PHASE_EMERGENCY    = 7,    /* one direction green, others red     */
    PHASE_STARTUP      = 8     /* initial state                       */
} phase_t;

static const char *PHASE_NAMES[9] __attribute__((unused)) = {
    "NS-GREEN", "NS-YELLOW", "ALL-RED-1",
    "EW-GREEN", "EW-YELLOW", "ALL-RED-2",
    "PEDESTRIAN", "EMERGENCY", "STARTUP"
};

/* ------------------------------------------------------------------ */
/*  Shared-memory blackboard                                          */
/*                                                                    */
/*  All processes read this; only the controller and the light        */
/*  processes write to it (protected by SEM_MUTEX).                   */
/* ------------------------------------------------------------------ */
typedef struct {
    /* per-direction state */
    light_color_t  light[NUM_DIRECTIONS];
    int            waiting_vehicles[NUM_DIRECTIONS];
    time_t         last_change[NUM_DIRECTIONS];

    /* pedestrian state */
    int            pedestrian_pending[NUM_DIRECTIONS];
    time_t         pedestrian_request_time[NUM_DIRECTIONS];
    int            pedestrian_active;            /* 0 / 1            */
    int            pedestrian_remaining;         /* seconds left     */

    /* emergency state */
    int            emergency_active;
    direction_t    emergency_direction;
    time_t         emergency_started;

    /* ticket / red-light-runner state.  When ticket_active=1, the
     * controller suspends phase progression until the GUI clears it. */
    int            ticket_active;
    direction_t    ticket_direction;
    char           ticket_plate[16];

    /* Per-light fault state.  light_faulted[d]=1 means direction d's
     * light has been forced down by an injected fault and shows no
     * lamps until cleared.  Other directions keep operating normally. */
    int            light_faulted[NUM_DIRECTIONS];

    /* phase/cycle */
    phase_t        current_phase;
    time_t         phase_started;
    int            phase_remaining;              /* seconds left     */

    /* configuration loaded from file */
    int            t_green_min;
    int            t_green_max;
    int            t_yellow;
    int            t_all_red;
    int            t_pedestrian;
    int            t_pedestrian_max_wait;
    int            t_vehicle_max_wait;
    int            t_emergency_response;
    int t_emergency_max_hold;   /* max seconds to hold emergency green */
    int            vehicle_spawn_rate;          /* % per second/dir  */
    int            pedestrian_spawn_rate;
    int            emergency_spawn_rate;        /* per mille / s     */

    /* book-keeping */
    int            total_vehicles_served;
    int            total_pedestrians_served;
    int            total_emergencies;
    int            total_tickets;
    int            safety_violations;
    int            running;                     /* shutdown flag     */

} shared_state_t;

/* ------------------------------------------------------------------ */
/*  Semaphore set: one element for now (mutex)                        */
/* ------------------------------------------------------------------ */
#define SEM_MUTEX 0
#define SEM_COUNT 1

/* Linux requires us to define this union ourselves (semctl) */
#if defined(__GNU_LIBRARY__) && !defined(_SEM_SEMUN_UNDEFINED)
/* glibc provides it */
#else
union semun {
    int              val;
    struct semid_ds *buf;
    unsigned short  *array;
    struct seminfo  *__buf;
};
#endif

/* ------------------------------------------------------------------ */
/*  Message structures                                                */
/* ------------------------------------------------------------------ */

/* Command:  controller -> traffic light                              */
typedef struct {
    long          mtype;            /* MTYPE_CMD_BASE + direction    */
    direction_t   direction;
    light_color_t target_color;
    time_t        timestamp;
    int           cmd_id;           /* for ack matching              */
} cmd_msg_t;

/* Acknowledgement: traffic light -> controller                       */
typedef struct {
    long          mtype;            /* MTYPE_ACK_BASE + direction    */
    direction_t   direction;
    light_color_t current_color;
    int           cmd_id;
    int           success;          /* 1 ok, 0 fault                 */
    time_t        timestamp;
} ack_msg_t;

/* Event: sensor / pedestrian / emergency -> controller               */
typedef struct {
    long          mtype;            /* one of MTYPE_EVT_*            */
    direction_t   direction;
    int           count;            /* e.g. # vehicles arriving      */
    int           priority;         /* 0 normal, 1 high              */
    time_t        timestamp;
    char          info[64];
} evt_msg_t;

/* Log: anyone -> logger                                              */
typedef struct {
    long          mtype;            /* always MTYPE_LOG_ANY          */
    time_t        timestamp;
    int           severity;         /* 0 INFO  1 WARN  2 ERROR       */
    char          source[32];
    char          text[MAX_LOG_LEN];
} log_msg_t;

/* ------------------------------------------------------------------ */
/*  Semaphore helpers (P / V wrappers)                                */
/* ------------------------------------------------------------------ */
static inline int sem_lock(int semid)
{
    struct sembuf op = { SEM_MUTEX, -1, SEM_UNDO };
    while (semop(semid, &op, 1) == -1) {
        if (errno == EINTR) continue;
        return -1;
    }
    return 0;
}
static inline int sem_unlock(int semid)
{
    struct sembuf op = { SEM_MUTEX, +1, SEM_UNDO };
    while (semop(semid, &op, 1) == -1) {
        if (errno == EINTR) continue;
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Logging helper: build a log_msg and send it                       */
/* ------------------------------------------------------------------ */
static inline void send_log(int logq, const char *source, int severity,
                            const char *fmt, ...)
{
    if (logq < 0) return;
    log_msg_t m;
    memset(&m, 0, sizeof(m));
    m.mtype     = MTYPE_LOG_ANY;
    m.timestamp = time(NULL);
    m.severity  = severity;
    strncpy(m.source, source, sizeof(m.source) - 1);

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(m.text, sizeof(m.text), fmt, ap);
    va_end(ap);

    /* IPC_NOWAIT so the producer is never blocked by the logger */
    msgsnd(logq, &m, sizeof(m) - sizeof(long), IPC_NOWAIT);
}

/* ------------------------------------------------------------------ */
/*  Safety check: returns 1 if state is safe, 0 if not                */
/* ------------------------------------------------------------------ */
static inline int state_is_safe(const shared_state_t *s)
{
    /* Rule 1: NS green and EW green simultaneously is forbidden.    */
    int ns_green = (s->light[DIR_NORTH] == LIGHT_GREEN) ||
                   (s->light[DIR_SOUTH] == LIGHT_GREEN);
    int ew_green = (s->light[DIR_EAST]  == LIGHT_GREEN) ||
                   (s->light[DIR_WEST]  == LIGHT_GREEN);
    if (ns_green && ew_green) return 0;

    /* Rule 2 (synchronized crossing model):
     * Pedestrians on E/W crosswalks cross WHILE N/S vehicles are green,
     * and N/S pedestrians cross WHILE E/W vehicles are green. The old
     * "no green + ped_active" rule is therefore intentionally removed.
     * Safety is guaranteed by geometry: peds only cross roads whose
     * vehicle signals are RED (the perpendicular axis).             */

    /* Rule 3: pedestrians must never be active during an emergency. */
    if (s->pedestrian_active && s->emergency_active) return 0;

    return 1;
}

/* ------------------------------------------------------------------ */
/*  Pretty time helper                                                */
/* ------------------------------------------------------------------ */
static inline void fmt_time(time_t t, char *buf, size_t n)
{
    struct tm tmv;
    localtime_r(&t, &tmv);
    strftime(buf, n, "%H:%M:%S", &tmv);
}

#endif /* COMMON_H */
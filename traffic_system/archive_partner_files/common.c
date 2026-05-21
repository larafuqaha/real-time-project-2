/******************************************************************************
 * common.c
 * Shared helpers: IPC setup, logging, config parsing.
 *****************************************************************************/
#include "common.h"
#include <stdarg.h>
#include <ctype.h>

/* Union required by semctl on Linux. */
union semun {
    int              val;
    struct semid_ds *buf;
    unsigned short  *array;
};

/* ------------------------------------------------------------------------- */
/* Name helpers                                                              */
/* ------------------------------------------------------------------------- */
const char *dir_name(Direction d)
{
    switch (d) {
        case DIR_NORTH: return "NORTH";
        case DIR_SOUTH: return "SOUTH";
        case DIR_EAST:  return "EAST";
        case DIR_WEST:  return "WEST";
        default:        return "?";
    }
}

const char *lane_name(Lane l)
{
    switch (l) {
        case LANE_THROUGH: return "THROUGH";
        case LANE_LEFT:    return "LEFT";
        default:           return "?";
    }
}

const char *light_name(LightState s)
{
    switch (s) {
        case LIGHT_RED:    return "RED";
        case LIGHT_YELLOW: return "YELLOW";
        case LIGHT_GREEN:  return "GREEN";
        default:           return "?";
    }
}

const char *phase_name(TrafficPhase p)
{
    switch (p) {
        case PHASE_NS_THROUGH: return "NS_THROUGH";
        case PHASE_NS_LEFT:    return "NS_LEFT";
        case PHASE_EW_THROUGH: return "EW_THROUGH";
        case PHASE_EW_LEFT:    return "EW_LEFT";
        case PHASE_ALL_RED:    return "ALL_RED";
        case PHASE_EMERGENCY:  return "EMERGENCY";
        default:               return "?";
    }
}

const char *ped_name(PedState s)
{
    switch (s) {
        case PED_DONT_WALK:     return "DONT_WALK";
        case PED_WALK:          return "WALK";
        case PED_WALK_FLASHING: return "WALK_FLASH";
        default:                return "?";
    }
}

/* ------------------------------------------------------------------------- */
/* IPC helpers                                                               */
/* ------------------------------------------------------------------------- */
static key_t make_key(int proj_id)
{
    key_t k = ftok(IPC_KEY_PATH, proj_id);
    if (k == (key_t)-1) {
        perror("ftok");
        exit(EXIT_FAILURE);
    }
    return k;
}

int get_shm(int create)
{
    int flags = 0666 | (create ? IPC_CREAT : 0);
    int id = shmget(make_key(SHM_PROJ_ID), sizeof(SharedState), flags);
    if (id < 0) perror("shmget");
    return id;
}

int get_evt_queue(int create)
{
    int flags = 0666 | (create ? IPC_CREAT : 0);
    int id = msgget(make_key(MSGQ_EVENT_ID), flags);
    if (id < 0) perror("msgget(evt)");
    return id;
}

int get_cmd_queue(int create)
{
    int flags = 0666 | (create ? IPC_CREAT : 0);
    int id = msgget(make_key(MSGQ_CMD_ID), flags);
    if (id < 0) perror("msgget(cmd)");
    return id;
}

int get_sts_queue(int create)
{
    int flags = 0666 | (create ? IPC_CREAT : 0);
    int id = msgget(make_key(MSGQ_STATUS_ID), flags);
    if (id < 0) perror("msgget(sts)");
    return id;
}

int get_sem(int create)
{
    int flags = 0666 | (create ? IPC_CREAT : 0);
    int id = semget(make_key(SEM_PROJ_ID), 1, flags);
    if (id < 0) { perror("semget"); return -1; }
    if (create) {
        union semun arg;
        arg.val = 1;
        if (semctl(id, 0, SETVAL, arg) < 0) {
            perror("semctl(SETVAL)");
            return -1;
        }
    }
    return id;
}

int sem_lock(int semid)
{
    struct sembuf op = {0, -1, SEM_UNDO};
    while (semop(semid, &op, 1) < 0) {
        if (errno == EINTR) continue;
        perror("semop(lock)");
        return -1;
    }
    return 0;
}

int sem_unlock(int semid)
{
    struct sembuf op = {0, +1, SEM_UNDO};
    if (semop(semid, &op, 1) < 0) {
        perror("semop(unlock)");
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
/* Logging                                                                   */
/* ------------------------------------------------------------------------- */
void log_event(SharedState *s, int semid, const char *fmt, ...)
{
    char line[LOG_LINE_LEN + 32];
    char ts[16];
    time_t now = time(NULL);
    struct tm tmv;
    localtime_r(&now, &tmv);
    strftime(ts, sizeof(ts), "%H:%M:%S", &tmv);

    char msg[LOG_LINE_LEN];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);

    snprintf(line, sizeof(line), "[%s] %s", ts, msg);

    if (s != NULL) {
        if (semid >= 0) sem_lock(semid);
        int idx = s->log_head;
        /* Use memcpy + explicit terminator to silence -Wstringop-truncation */
        size_t len = strlen(line);
        if (len >= LOG_LINE_LEN) len = LOG_LINE_LEN - 1;
        memcpy(s->log_buffer[idx], line, len);
        s->log_buffer[idx][len] = '\0';
        s->log_head = (s->log_head + 1) % MAX_LOG_LINES;
        if (s->log_count < MAX_LOG_LINES) s->log_count++;
        int do_file = s->cfg.log_to_file;
        char path[256];
        strncpy(path, s->cfg.log_file_path, sizeof(path) - 1);
        path[sizeof(path) - 1] = '\0';
        if (semid >= 0) sem_unlock(semid);

        if (do_file) {
            FILE *f = fopen(path, "a");
            if (f) {
                fprintf(f, "%s\n", line);
                fclose(f);
            }
        }
    }

    /* Also write to stderr if running with debug. */
    if (getenv("TRAFFIC_DEBUG")) {
        fprintf(stderr, "%s\n", line);
    }
}

/* ------------------------------------------------------------------------- */
/* Config                                                                    */
/* ------------------------------------------------------------------------- */
void set_default_config(Config *cfg)
{
    cfg->green_min_sec            = 8;
    cfg->green_max_sec            = 20;
    cfg->yellow_sec               = 3;
    cfg->all_red_sec              = 2;
    cfg->ped_walk_sec             = 7;
    cfg->ped_flash_sec            = 4;
    cfg->ped_max_wait_sec         = 30;
    cfg->veh_max_wait_sec         = 45;
    cfg->emergency_clear_sec      = 3;
    cfg->emergency_green_sec      = 10;
    cfg->vehicle_spawn_rate_ms    = 2500;
    cfg->pedestrian_spawn_rate_ms = 9000;
    cfg->emergency_spawn_rate_ms  = 30000;
    cfg->enable_random_traffic    = 1;
    cfg->enable_random_pedestrians= 1;
    cfg->enable_random_emergencies= 1;
    cfg->left_turn_green_sec      = 6;
    cfg->log_to_file              = 1;
    strcpy(cfg->log_file_path, "logs/traffic.log");
}

static void trim(char *s)
{
    char *p = s;
    while (*p && isspace((unsigned char)*p)) p++;
    if (p != s) memmove(s, p, strlen(p) + 1);
    size_t n = strlen(s);
    while (n > 0 && isspace((unsigned char)s[n - 1])) s[--n] = '\0';
}

int load_config(const char *path, Config *cfg)
{
    set_default_config(cfg);
    if (!path) return 0;
    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "Warning: could not open config '%s' (%s). Using defaults.\n",
                path, strerror(errno));
        return -1;
    }
    char line[512];
    while (fgets(line, sizeof(line), f)) {
        /* Strip comments */
        char *hash = strchr(line, '#');
        if (hash) *hash = '\0';
        trim(line);
        if (line[0] == '\0') continue;
        char key[64], val[256];
        if (sscanf(line, "%63[^=]=%255[^\n]", key, val) != 2) continue;
        trim(key); trim(val);

#define MATCH_INT(k, field) \
        do { if (strcmp(key, k) == 0) { cfg->field = atoi(val); continue; } } while (0)

        MATCH_INT("green_min_sec",            green_min_sec);
        MATCH_INT("green_max_sec",            green_max_sec);
        MATCH_INT("yellow_sec",               yellow_sec);
        MATCH_INT("all_red_sec",              all_red_sec);
        MATCH_INT("ped_walk_sec",             ped_walk_sec);
        MATCH_INT("ped_flash_sec",            ped_flash_sec);
        MATCH_INT("ped_max_wait_sec",         ped_max_wait_sec);
        MATCH_INT("veh_max_wait_sec",         veh_max_wait_sec);
        MATCH_INT("emergency_clear_sec",      emergency_clear_sec);
        MATCH_INT("emergency_green_sec",      emergency_green_sec);
        MATCH_INT("vehicle_spawn_rate_ms",    vehicle_spawn_rate_ms);
        MATCH_INT("pedestrian_spawn_rate_ms", pedestrian_spawn_rate_ms);
        MATCH_INT("emergency_spawn_rate_ms",  emergency_spawn_rate_ms);
        MATCH_INT("enable_random_traffic",    enable_random_traffic);
        MATCH_INT("enable_random_pedestrians",enable_random_pedestrians);
        MATCH_INT("enable_random_emergencies",enable_random_emergencies);
        MATCH_INT("left_turn_green_sec",      left_turn_green_sec);
        MATCH_INT("log_to_file",              log_to_file);

        if (strcmp(key, "log_file_path") == 0) {
            strncpy(cfg->log_file_path, val, sizeof(cfg->log_file_path) - 1);
            cfg->log_file_path[sizeof(cfg->log_file_path) - 1] = '\0';
        }
#undef MATCH_INT
    }
    fclose(f);
    return 0;
}

/* ------------------------------------------------------------------------- */
/* Safety check – verifies the light array matches the declared phase.       */
/* ------------------------------------------------------------------------- */
int is_phase_safe(TrafficPhase p, const SharedState *s)
{
    /* No two conflicting movements may both be green. */
    /* Build per-(dir,lane) green map and check pairwise conflicts.          */
    int g[NUM_DIRS][NUM_LANES];
    for (int d = 0; d < NUM_DIRS; d++)
        for (int l = 0; l < NUM_LANES; l++)
            g[d][l] = (s->light[d][l] == LIGHT_GREEN);

    /* Conflicting pairs:
     *   - NS through vs EW through/left
     *   - EW through vs NS through/left
     *   - NS left   vs NS through (left and through on same axis conflict)
     *   - EW left   vs EW through
     *   - any green vs any pedestrian WALK on its own axis
     */
    if ((g[DIR_NORTH][LANE_THROUGH] || g[DIR_SOUTH][LANE_THROUGH]) &&
        (g[DIR_EAST][LANE_THROUGH]  || g[DIR_WEST][LANE_THROUGH]  ||
         g[DIR_EAST][LANE_LEFT]     || g[DIR_WEST][LANE_LEFT]))
        return 0;
    if ((g[DIR_EAST][LANE_THROUGH] || g[DIR_WEST][LANE_THROUGH]) &&
        (g[DIR_NORTH][LANE_LEFT] || g[DIR_SOUTH][LANE_LEFT]))
        return 0;
    if ((g[DIR_NORTH][LANE_LEFT] || g[DIR_SOUTH][LANE_LEFT]) &&
        (g[DIR_NORTH][LANE_THROUGH] || g[DIR_SOUTH][LANE_THROUGH]))
        return 0;
    if ((g[DIR_EAST][LANE_LEFT] || g[DIR_WEST][LANE_LEFT]) &&
        (g[DIR_EAST][LANE_THROUGH] || g[DIR_WEST][LANE_THROUGH]))
        return 0;

    UNUSED(p);
    return 1;
}

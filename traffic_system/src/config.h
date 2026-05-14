/*****************************************************************************
 *  config.h / config.c-equivalent inline parser
 *  Loads key=value pairs from a plain text file into the shared state.
 *  Lines starting with '#' or empty lines are ignored.
 *****************************************************************************/
#ifndef CONFIG_H
#define CONFIG_H

#include "common.h"

/* Apply a single key=value pair to the shared state.
 * Returns 1 if the key was recognised, 0 otherwise.            */
static int apply_kv(shared_state_t *s, const char *key, const char *val)
{
    int v = atoi(val);
    if      (!strcmp(key, "T_GREEN_MIN"))           s->t_green_min = v;
    else if (!strcmp(key, "T_GREEN_MAX"))           s->t_green_max = v;
    else if (!strcmp(key, "T_YELLOW"))              s->t_yellow = v;
    else if (!strcmp(key, "T_ALL_RED"))             s->t_all_red = v;
    else if (!strcmp(key, "T_PEDESTRIAN"))          s->t_pedestrian = v;
    else if (!strcmp(key, "T_PEDESTRIAN_MAX_WAIT")) s->t_pedestrian_max_wait = v;
    else if (!strcmp(key, "T_VEHICLE_MAX_WAIT"))    s->t_vehicle_max_wait = v;
    else if (!strcmp(key, "T_EMERGENCY_RESPONSE"))  s->t_emergency_response = v;
    else if (!strcmp(key, "VEHICLE_SPAWN_RATE"))    s->vehicle_spawn_rate = v;
    else if (!strcmp(key, "PEDESTRIAN_SPAWN_RATE")) s->pedestrian_spawn_rate = v;
    else if (!strcmp(key, "EMERGENCY_SPAWN_RATE"))  s->emergency_spawn_rate = v;
    else return 0;
    return 1;
}

/* Load defaults so the program still runs if a value is missing.   */
static void load_defaults(shared_state_t *s)
{
    s->t_green_min            = 8;
    s->t_green_max            = 15;
    s->t_yellow               = 3;
    s->t_all_red              = 2;
    s->t_pedestrian           = 10;
    s->t_pedestrian_max_wait  = 45;
    s->t_vehicle_max_wait     = 60;
    s->t_emergency_response   = 5;
    s->vehicle_spawn_rate     = 25;   /* 25% chance/sec/direction */
    s->pedestrian_spawn_rate  = 10;
    s->emergency_spawn_rate   = 5;    /* 5 in 1000 / s */
}

/* Returns 0 on success, -1 if the file cannot be opened.           */
static int load_config(const char *path, shared_state_t *s)
{
    load_defaults(s);
    FILE *fp = fopen(path, "r");
    if (!fp) return -1;

    char line[256];
    while (fgets(line, sizeof(line), fp)) {
        /* strip comments and whitespace */
        char *hash = strchr(line, '#'); if (hash) *hash = '\0';
        char *eq   = strchr(line, '='); if (!eq) continue;
        *eq = '\0';
        char *key = line, *val = eq + 1;

        /* trim */
        while (*key && (*key == ' ' || *key == '\t')) key++;
        char *end = key + strlen(key) - 1;
        while (end > key && (*end == ' ' || *end == '\t' || *end == '\n' || *end == '\r')) *end-- = '\0';
        while (*val && (*val == ' ' || *val == '\t')) val++;
        end = val + strlen(val) - 1;
        while (end > val && (*end == ' ' || *end == '\t' || *end == '\n' || *end == '\r')) *end-- = '\0';

        if (*key == '\0') continue;
        apply_kv(s, key, val);
    }
    fclose(fp);
    return 0;
}

#endif /* CONFIG_H */

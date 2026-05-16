/*****************************************************************************
 *  gui_opengl.c
 *  ---------------------------------------------------------------------------
 *  Realistic OpenGL visualisation of the intersection.
 *
 *  - Top-down/perspective view of a 4-way intersection.
 *  - Asphalt roads with lane markings and pedestrian zebra crossings.
 *  - 3-D traffic-light poles at each corner with red/yellow/green lamps that
 *    actually glow when active.
 *  - Cars are simple 3-D boxes that appear in lanes when vehicles are
 *    detected in shared memory and move during their green phase.
 *  - Pedestrians (small standing figures) appear on sidewalks and walk across
 *    when the pedestrian phase is active.
 *  - Emergency vehicle: a red car with flashing blue/red lights when active.
 *  - HUD overlay showing the current phase, countdown, queue lengths.
 *
 *  Dependencies: freeglut (libglut), GLU, GL.
 *  Build:        see Makefile (uses pkg-config or -lglut -lGLU -lGL).
 *
 *  The GUI is a pure consumer of the shared state — it never modifies it,
 *  so it doesn't need to hold the mutex for long; it takes a snapshot,
 *  releases the lock, then renders.
 *****************************************************************************/
#include "common.h"
#include "ipc_init.h"

#include <GL/glut.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* IPC */
static int            shmid = -1, semid = -1, qevt = -1, qlog = -1;
static shared_state_t *S = NULL;
static shared_state_t  snap;        /* local snapshot per frame    */

/* window */
static int   WIN_W = 1100, WIN_H = 760;
static float cam_angle   = 25.0f;      /* tilt above horizon */
static float cam_rotate  = 0.0f;       /* rotate around y    */
static float cam_dist    = 28.0f;
static int   show_help   = 1;

/* recent events ring buffer for HUD display */
#define MAX_RECENT_EVENTS 5
static char recent_events[MAX_RECENT_EVENTS][128];
static int  recent_count = 0;
static int  recent_head  = 0;

/* animation */
static float t_animation = 0.0f;       /* global anim time   */

/* ---------- low-level draw helpers ---------------------------------- */

static void set_color(float r, float g, float b)
{
    GLfloat m[] = {r, g, b, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, m);
    glColor3f(r, g, b);
}

static void set_emissive(float r, float g, float b)
{
    GLfloat e[] = {r, g, b, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, e);
}
static void clear_emissive(void)
{
    GLfloat e[] = {0, 0, 0, 1};
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, e);
}

/* draw an axis-aligned cuboid centred at origin with given size */
static void draw_box(float sx, float sy, float sz)
{
    glPushMatrix();
    glScalef(sx, sy, sz);
    glutSolidCube(1.0f);
    glPopMatrix();
}

static void draw_text(float x, float y, const char *s)
{
    glRasterPos2f(x, y);
    while (*s) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *s++);
}

static void draw_text_big(float x, float y, const char *s)
{
    glRasterPos2f(x, y);
    while (*s) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *s++);
}

/* ---------- world drawing -------------------------------------------- */

#define ROAD_HALF_WIDTH   3.0f   /* half-width of each road  */
#define ROAD_LENGTH       18.0f
#define INTER_HALF        3.0f   /* half-size of central square */

static void draw_ground(void)
{
    /* big grass plane */
    set_color(0.18f, 0.42f, 0.18f);
    glBegin(GL_QUADS);
        glNormal3f(0, 1, 0);
        glVertex3f(-25, 0, -25);
        glVertex3f( 25, 0, -25);
        glVertex3f( 25, 0,  25);
        glVertex3f(-25, 0,  25);
    glEnd();

    /* sidewalks (lighter quads around corners) */
    set_color(0.65f, 0.65f, 0.62f);
    for (int dx = -1; dx <= 1; dx += 2)
    for (int dz = -1; dz <= 1; dz += 2) {
        float x0 = dx * ROAD_HALF_WIDTH;
        float x1 = dx * (ROAD_HALF_WIDTH + 3.0f);
        float z0 = dz * ROAD_HALF_WIDTH;
        float z1 = dz * (ROAD_HALF_WIDTH + 3.0f);
        glBegin(GL_QUADS);
        glNormal3f(0, 1, 0);
        glVertex3f(x0, 0.02f, z0);
        glVertex3f(x1, 0.02f, z0);
        glVertex3f(x1, 0.02f, z1);
        glVertex3f(x0, 0.02f, z1);
        glEnd();
    }
}

static void draw_roads(void)
{
    /* asphalt */
    set_color(0.16f, 0.16f, 0.18f);

    /* horizontal road (East-West) */
    glBegin(GL_QUADS);
    glNormal3f(0, 1, 0);
    glVertex3f(-ROAD_LENGTH, 0.03f, -ROAD_HALF_WIDTH);
    glVertex3f( ROAD_LENGTH, 0.03f, -ROAD_HALF_WIDTH);
    glVertex3f( ROAD_LENGTH, 0.03f,  ROAD_HALF_WIDTH);
    glVertex3f(-ROAD_LENGTH, 0.03f,  ROAD_HALF_WIDTH);
    glEnd();

    /* vertical road (North-South) */
    glBegin(GL_QUADS);
    glNormal3f(0, 1, 0);
    glVertex3f(-ROAD_HALF_WIDTH, 0.03f, -ROAD_LENGTH);
    glVertex3f( ROAD_HALF_WIDTH, 0.03f, -ROAD_LENGTH);
    glVertex3f( ROAD_HALF_WIDTH, 0.03f,  ROAD_LENGTH);
    glVertex3f(-ROAD_HALF_WIDTH, 0.03f,  ROAD_LENGTH);
    glEnd();

    /* lane dividers (dashed yellow lines) */
    set_color(0.95f, 0.85f, 0.15f);
    /* east-west centre dashes */
    for (float x = -ROAD_LENGTH; x < ROAD_LENGTH; x += 1.5f) {
        if (fabsf(x) < INTER_HALF) continue;
        glBegin(GL_QUADS);
        glNormal3f(0, 1, 0);
        glVertex3f(x,      0.05f, -0.08f);
        glVertex3f(x+0.8f, 0.05f, -0.08f);
        glVertex3f(x+0.8f, 0.05f,  0.08f);
        glVertex3f(x,      0.05f,  0.08f);
        glEnd();
    }
    /* north-south centre dashes */
    for (float z = -ROAD_LENGTH; z < ROAD_LENGTH; z += 1.5f) {
        if (fabsf(z) < INTER_HALF) continue;
        glBegin(GL_QUADS);
        glNormal3f(0, 1, 0);
        glVertex3f(-0.08f, 0.05f, z);
        glVertex3f( 0.08f, 0.05f, z);
        glVertex3f( 0.08f, 0.05f, z+0.8f);
        glVertex3f(-0.08f, 0.05f, z+0.8f);
        glEnd();
    }

    /* stop lines (white) at each entry */
    set_color(0.92f, 0.92f, 0.92f);
    /* North-bound stop */
    glBegin(GL_QUADS); glNormal3f(0,1,0);
    glVertex3f( 0.1f, 0.05f, INTER_HALF + 0.2f);
    glVertex3f( ROAD_HALF_WIDTH, 0.05f, INTER_HALF + 0.2f);
    glVertex3f( ROAD_HALF_WIDTH, 0.05f, INTER_HALF + 0.5f);
    glVertex3f( 0.1f, 0.05f, INTER_HALF + 0.5f);
    glEnd();
    /* South-bound */
    glBegin(GL_QUADS); glNormal3f(0,1,0);
    glVertex3f(-ROAD_HALF_WIDTH, 0.05f, -INTER_HALF - 0.5f);
    glVertex3f(-0.1f,             0.05f, -INTER_HALF - 0.5f);
    glVertex3f(-0.1f,             0.05f, -INTER_HALF - 0.2f);
    glVertex3f(-ROAD_HALF_WIDTH, 0.05f, -INTER_HALF - 0.2f);
    glEnd();
    /* East-bound */
    glBegin(GL_QUADS); glNormal3f(0,1,0);
    glVertex3f(-INTER_HALF - 0.5f, 0.05f, 0.1f);
    glVertex3f(-INTER_HALF - 0.2f, 0.05f, 0.1f);
    glVertex3f(-INTER_HALF - 0.2f, 0.05f, ROAD_HALF_WIDTH);
    glVertex3f(-INTER_HALF - 0.5f, 0.05f, ROAD_HALF_WIDTH);
    glEnd();
    /* West-bound */
    glBegin(GL_QUADS); glNormal3f(0,1,0);
    glVertex3f( INTER_HALF + 0.2f, 0.05f, -ROAD_HALF_WIDTH);
    glVertex3f( INTER_HALF + 0.5f, 0.05f, -ROAD_HALF_WIDTH);
    glVertex3f( INTER_HALF + 0.5f, 0.05f, -0.1f);
    glVertex3f( INTER_HALF + 0.2f, 0.05f, -0.1f);
    glEnd();

    /* zebra crossings (white stripes) at each side */
    set_color(0.92f, 0.92f, 0.92f);
    /* North crossing */
    for (float x = -ROAD_HALF_WIDTH + 0.3f; x < ROAD_HALF_WIDTH; x += 0.7f) {
        glBegin(GL_QUADS); glNormal3f(0,1,0);
        glVertex3f(x,        0.06f, -INTER_HALF - 0.8f);
        glVertex3f(x + 0.4f, 0.06f, -INTER_HALF - 0.8f);
        glVertex3f(x + 0.4f, 0.06f, -INTER_HALF - 0.1f);
        glVertex3f(x,        0.06f, -INTER_HALF - 0.1f);
        glEnd();
    }
    /* South crossing */
    for (float x = -ROAD_HALF_WIDTH + 0.3f; x < ROAD_HALF_WIDTH; x += 0.7f) {
        glBegin(GL_QUADS); glNormal3f(0,1,0);
        glVertex3f(x,        0.06f,  INTER_HALF + 0.1f);
        glVertex3f(x + 0.4f, 0.06f,  INTER_HALF + 0.1f);
        glVertex3f(x + 0.4f, 0.06f,  INTER_HALF + 0.8f);
        glVertex3f(x,        0.06f,  INTER_HALF + 0.8f);
        glEnd();
    }
    /* East crossing */
    for (float z = -ROAD_HALF_WIDTH + 0.3f; z < ROAD_HALF_WIDTH; z += 0.7f) {
        glBegin(GL_QUADS); glNormal3f(0,1,0);
        glVertex3f( INTER_HALF + 0.1f, 0.06f, z);
        glVertex3f( INTER_HALF + 0.8f, 0.06f, z);
        glVertex3f( INTER_HALF + 0.8f, 0.06f, z + 0.4f);
        glVertex3f( INTER_HALF + 0.1f, 0.06f, z + 0.4f);
        glEnd();
    }
    /* West crossing */
    for (float z = -ROAD_HALF_WIDTH + 0.3f; z < ROAD_HALF_WIDTH; z += 0.7f) {
        glBegin(GL_QUADS); glNormal3f(0,1,0);
        glVertex3f(-INTER_HALF - 0.8f, 0.06f, z);
        glVertex3f(-INTER_HALF - 0.1f, 0.06f, z);
        glVertex3f(-INTER_HALF - 0.1f, 0.06f, z + 0.4f);
        glVertex3f(-INTER_HALF - 0.8f, 0.06f, z + 0.4f);
        glEnd();
    }
}

/* Draw a single traffic-light pole with three lamps. */
static void draw_traffic_light(float px, float pz, float yaw,
                               light_color_t color)
{
    glPushMatrix();
    glTranslatef(px, 0, pz);
    glRotatef(yaw, 0, 1, 0);

    /* Pole */
    set_color(0.20f, 0.20f, 0.20f);
    glPushMatrix();
        glTranslatef(0, 1.6f, 0);
        draw_box(0.18f, 3.2f, 0.18f);
    glPopMatrix();

    /* Housing */
    set_color(0.10f, 0.10f, 0.10f);
    glPushMatrix();
        glTranslatef(0, 3.6f, 0.35f);
        draw_box(0.55f, 1.5f, 0.45f);
    glPopMatrix();

    /* Lamps: red(top), yellow(mid), green(bottom) */
    GLfloat lamp_y[3] = { 4.15f, 3.65f, 3.15f };
    light_color_t order[3] = { LIGHT_RED, LIGHT_YELLOW, LIGHT_GREEN };
    GLfloat colors[3][3] = {
        {0.95f, 0.10f, 0.10f},
        {0.95f, 0.85f, 0.15f},
        {0.15f, 0.85f, 0.20f}
    };
    for (int i = 0; i < 3; ++i) {
        glPushMatrix();
        glTranslatef(0, lamp_y[i], 0.58f);
        if (order[i] == color) {
            set_color(colors[i][0], colors[i][1], colors[i][2]);
            set_emissive(colors[i][0] * 0.8f,
                         colors[i][1] * 0.8f,
                         colors[i][2] * 0.8f);
        } else {
            set_color(colors[i][0] * 0.25f,
                      colors[i][1] * 0.25f,
                      colors[i][2] * 0.25f);
            clear_emissive();
        }
        glutSolidSphere(0.18f, 16, 16);
        clear_emissive();
        glPopMatrix();
    }

    glPopMatrix();
}

/* ---------- vehicles ------------------------------------------------- */
static void draw_car(float r, float g, float b)
{
    /* body */
    set_color(r, g, b);
    glPushMatrix();
    glTranslatef(0, 0.30f, 0);
    draw_box(1.6f, 0.45f, 0.85f);
    glPopMatrix();

    /* cabin */
    set_color(r * 0.5f, g * 0.5f, b * 0.5f);
    glPushMatrix();
    glTranslatef(-0.05f, 0.65f, 0);
    draw_box(0.9f, 0.35f, 0.75f);
    glPopMatrix();

    /* wheels */
    set_color(0.05f, 0.05f, 0.05f);
    for (int sx = -1; sx <= 1; sx += 2)
    for (int sz = -1; sz <= 1; sz += 2) {
        glPushMatrix();
        glTranslatef(sx * 0.55f, 0.12f, sz * 0.42f);
        glutSolidSphere(0.16f, 10, 10);
        glPopMatrix();
    }
}

static void draw_emergency_car(int t_ms)
{
    draw_car(0.85f, 0.10f, 0.10f);
    /* roof lightbar — alternates red and blue */
    int blink = (t_ms / 250) & 1;
    glPushMatrix();
    glTranslatef(0, 1.0f, 0);
    if (blink) { set_color(1.0f, 0.1f, 0.1f); set_emissive(1.0f, 0.1f, 0.1f); }
    else       { set_color(0.1f, 0.3f, 1.0f); set_emissive(0.1f, 0.3f, 1.0f); }
    draw_box(0.45f, 0.10f, 0.55f);
    clear_emissive();
    glPopMatrix();
}

static void draw_vehicles_queue(direction_t d, int count)
{
    if (count <= 0) return;
    int shown = count > 6 ? 6 : count;
    float palette[][3] = {
        {0.2f, 0.5f, 0.9f}, {0.95f, 0.95f, 0.95f}, {0.15f, 0.55f, 0.25f},
        {0.85f, 0.6f, 0.15f}, {0.6f, 0.2f, 0.7f},  {0.3f, 0.3f, 0.35f}
    };

    int moving = 0;
    if (d == DIR_NORTH || d == DIR_SOUTH) {
        if (snap.current_phase == PHASE_NS_GREEN) moving = 1;
    } else {
        if (snap.current_phase == PHASE_EW_GREEN) moving = 1;
    }

    float cycle      = 10.0f;
    float t_in_cycle = fmodf(t_animation, cycle);
    float progress   = moving ? (t_in_cycle / cycle) : 0.0f;

    for (int i = 0; i < shown; ++i) {
        float spacing = 2.2f;

        float entry_delay = i * 1.5f;
        float effective_progress = progress - (entry_delay / cycle);
        if (effective_progress < 0.0f) continue;

        float dist = INTER_HALF + 1.5f + i * spacing
                     - effective_progress * (ROAD_LENGTH * 2.0f + INTER_HALF);

        if (dist < -ROAD_LENGTH) continue;

        glPushMatrix();
        switch (d) {
        case DIR_NORTH:
            glTranslatef(1.3f, 0, dist);
            glRotatef(90, 0, 1, 0);
            break;
        case DIR_SOUTH:
            glTranslatef(-1.3f, 0, -dist);
            glRotatef(-90, 0, 1, 0);
            break;
        case DIR_EAST:
            glTranslatef(-dist, 0, 1.3f);
            break;
        case DIR_WEST:
            glTranslatef(dist, 0, -1.3f);
            glRotatef(180, 0, 1, 0);
            break;
        }
        float *c = palette[i % 6];
        draw_car(c[0], c[1], c[2]);
        glPopMatrix();
    }
}

static void draw_emergency_vehicle(void)
{
    if (!snap.emergency_active) return;
    direction_t d = snap.emergency_direction;

    float total_travel = 12.0f;
    float t     = fmodf(t_animation, total_travel);
    float speed = (ROAD_LENGTH * 2.0f + INTER_HALF * 2.0f) / total_travel;
    float pos   = t * speed;
    float dist  = ROAD_LENGTH - pos;

    int t_ms = (int)(t_animation * 1000);

    glPushMatrix();
    switch (d) {
    case DIR_NORTH:
        if (dist > 0) glTranslatef(-1.3f, 0, dist);
        else          glTranslatef( 1.3f, 0, dist);
        glRotatef(90, 0, 1, 0);
        break;
    case DIR_SOUTH:
        if (dist > 0) glTranslatef( 1.3f, 0, -dist);
        else          glTranslatef(-1.3f, 0, -dist);
        glRotatef(-90, 0, 1, 0);
        break;
    case DIR_EAST:
        if (dist > 0) glTranslatef(-dist, 0, -1.3f);
        else          glTranslatef(-dist, 0,  1.3f);
        break;
    case DIR_WEST:
        if (dist > 0) glTranslatef( dist, 0,  1.3f);
        else          glTranslatef( dist, 0, -1.3f);
        glRotatef(180, 0, 1, 0);
        break;
    }
    draw_emergency_car(t_ms);
    glPopMatrix();
}

/* ---------- pedestrians --------------------------------------------- */
static void draw_pedestrian(float x, float z, float r, float g, float b)
{
    /* body */
    set_color(r, g, b);
    glPushMatrix();
    glTranslatef(x, 0.55f, z);
    draw_box(0.22f, 0.55f, 0.22f);
    glPopMatrix();
    /* head */
    set_color(0.95f, 0.82f, 0.68f);
    glPushMatrix();
    glTranslatef(x, 1.0f, z);
    glutSolidSphere(0.15f, 12, 12);
    glPopMatrix();
    /* legs */
    set_color(0.2f, 0.2f, 0.45f);
    glPushMatrix();
    glTranslatef(x - 0.07f, 0.22f, z);
    draw_box(0.10f, 0.40f, 0.10f);
    glPopMatrix();
    glPushMatrix();
    glTranslatef(x + 0.07f, 0.22f, z);
    draw_box(0.10f, 0.40f, 0.10f);
    glPopMatrix();
}

typedef struct {
    int   walking;          
    int   frozen_count;     
    float walk_start_t; 
    int   last_light; 
} dir_ped_state_t;

static dir_ped_state_t dps[4];
static int dps_inited = 0;

static void draw_pedestrians(void)
{
    float ped_colors[][3] = {
        {0.85f, 0.20f, 0.20f},
        {0.20f, 0.55f, 0.85f},
        {0.20f, 0.65f, 0.35f},
        {0.85f, 0.45f, 0.20f},
        {0.75f, 0.20f, 0.75f},
        {0.95f, 0.85f, 0.20f},
    };
    int ncolors = 6;

    float speed    = 2.5f;
    float gap      = 0.6f;
    float road_end = ROAD_HALF_WIDTH * 2.0f + ROAD_LENGTH;

    if (!dps_inited) {
        for (int i = 0; i < 4; ++i) {
            dps[i].walking      = 0;
            dps[i].frozen_count = 0;
            dps[i].walk_start_t = 0.0f;
            dps[i].last_light   = LIGHT_RED;
        }
        dps_inited = 1;
    }

    for (int i = 0; i < NUM_DIRECTIONS; ++i) {
        int cur_light   = snap.light[i];
        int has_pending = snap.pedestrian_pending[i] > 0;

       
        if (cur_light == LIGHT_RED && dps[i].last_light != LIGHT_RED) {
            if (has_pending) {
                dps[i].walking      = 1;
                dps[i].walk_start_t = t_animation;
                int cnt = snap.pedestrian_pending[i];
                if (cnt > 8) cnt = 8;
                dps[i].frozen_count = cnt;
            }
        }

        
        if (cur_light != LIGHT_RED && dps[i].last_light == LIGHT_RED) {
           
        }

        dps[i].last_light = cur_light;

       
        if (dps[i].walking) {
            float base_pos = (t_animation - dps[i].walk_start_t) * speed;
            float last_pos = base_pos - (dps[i].frozen_count - 1) * gap;
            if (last_pos >= road_end) {
                dps[i].walking = 0;
                if (cur_light == LIGHT_RED && snap.pedestrian_pending[i] > 0) {
                    dps[i].walking      = 1;
                    dps[i].walk_start_t = t_animation;
                    int cnt = snap.pedestrian_pending[i];
                    if (cnt > 8) cnt = 8;
                    dps[i].frozen_count = cnt;
                }
            }
        }

      
        if (!dps[i].walking && has_pending) {
            int waiting = snap.pedestrian_pending[i];
            if (waiting > 8) waiting = 8;

            for (int k = 0; k < waiting; ++k) {
                float off = (k - (waiting - 1) * 0.5f) * 0.55f;
                float px, pz;
                switch (i) {
                case DIR_NORTH: px =  ROAD_HALF_WIDTH + 1.5f;  pz = -INTER_HALF - 0.6f + off; break;
                case DIR_SOUTH: px = -ROAD_HALF_WIDTH - 1.5f;  pz =  INTER_HALF + 0.6f + off; break;
                case DIR_EAST:  px = -INTER_HALF - 0.6f + off; pz = -ROAD_HALF_WIDTH - 1.5f;  break;
                case DIR_WEST:  px =  INTER_HALF + 0.6f + off; pz =  ROAD_HALF_WIDTH + 1.5f;  break;
                default:        px = pz = 0; break;
                }
                float *c = ped_colors[k % ncolors];
                draw_pedestrian(px, pz, c[0], c[1], c[2]);
            }
        }

    
        if (dps[i].walking) {
            float base_pos = (t_animation - dps[i].walk_start_t) * speed;
            int   n        = dps[i].frozen_count;

            for (int k = 0; k < n; ++k) {
                float pos = base_pos - k * gap;
                if (pos <= 0.0f)     continue; 
                if (pos >= road_end) continue; 

                float *c = ped_colors[k % ncolors];

                switch (i) {
                case DIR_NORTH:
                    draw_pedestrian(-ROAD_HALF_WIDTH + pos, -INTER_HALF - 0.45f, c[0], c[1], c[2]);
                    break;
                case DIR_SOUTH:
                    draw_pedestrian( ROAD_HALF_WIDTH - pos,  INTER_HALF + 0.45f, c[0], c[1], c[2]);
                    break;
                case DIR_EAST:
                    draw_pedestrian( INTER_HALF + 0.45f, -ROAD_HALF_WIDTH + pos, c[0], c[1], c[2]);
                    break;
                case DIR_WEST:
                    draw_pedestrian(-INTER_HALF - 0.45f,  ROAD_HALF_WIDTH - pos, c[0], c[1], c[2]);
                    break;
                }
            }
        }
    }
}

static void add_recent_event(const char *msg)
{
    strncpy(recent_events[recent_head], msg, 127);
    recent_events[recent_head][127] = '\0';
    recent_head = (recent_head + 1) % MAX_RECENT_EVENTS;
    if (recent_count < MAX_RECENT_EVENTS) recent_count++;
}

static void poll_recent_events(void)
{
    /* detect phase changes */
    static phase_t last_phase = PHASE_STARTUP;
    if (snap.current_phase != last_phase) {
        char buf[128];
        snprintf(buf, sizeof(buf), "Phase: %s", PHASE_NAMES[snap.current_phase]);
        add_recent_event(buf);
        last_phase = snap.current_phase;
    }

    /* detect emergency */
    static int last_emerg = 0;
    if (snap.emergency_active && !last_emerg) {
        char buf[128];
        snprintf(buf, sizeof(buf), "EMERGENCY from %s",
                 DIR_NAMES[snap.emergency_direction]);
        add_recent_event(buf);
    }
    last_emerg = snap.emergency_active;

    /* detect pedestrian phase */
    static int last_ped_ev = 0;
    if (snap.pedestrian_active && !last_ped_ev)
        add_recent_event("Pedestrian crossing OPEN");
    if (!snap.pedestrian_active && last_ped_ev)
        add_recent_event("Pedestrian crossing CLOSED");
    last_ped_ev = snap.pedestrian_active;
}

/* ---------- HUD ------------------------------------------------------ */
static void draw_hud(void)
{
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
    gluOrtho2D(0, WIN_W, 0, WIN_H);
    glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();

    /* translucent black panel top-left */
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0, 0, 0, 0.55f);
    glBegin(GL_QUADS);
        glVertex2f(10,            WIN_H - 10);
        glVertex2f(360,           WIN_H - 10);
        glVertex2f(360,           WIN_H - 230); 
        glVertex2f(10,            WIN_H - 230);
    glEnd();
    glDisable(GL_BLEND);

    char buf[256];
    glColor3f(0.95f, 0.95f, 0.95f);
    snprintf(buf, sizeof(buf), "Phase: %s",
             PHASE_NAMES[snap.current_phase]);
    draw_text_big(20, WIN_H - 30, buf);

    snprintf(buf, sizeof(buf), "Countdown: %d s", snap.phase_remaining);
    draw_text(20, WIN_H - 55, buf);

    snprintf(buf, sizeof(buf), "Lights : N=%s  S=%s  E=%s  W=%s",
             COLOR_NAMES[snap.light[0]], COLOR_NAMES[snap.light[1]],
             COLOR_NAMES[snap.light[2]], COLOR_NAMES[snap.light[3]]);
    draw_text(20, WIN_H - 80, buf);

    snprintf(buf, sizeof(buf), "Queues : N=%d  S=%d  E=%d  W=%d",
             snap.waiting_vehicles[0], snap.waiting_vehicles[1],
             snap.waiting_vehicles[2], snap.waiting_vehicles[3]);
    draw_text(20, WIN_H - 100, buf);

    int total_ped_waiting = 0;
    for (int i = 0; i < NUM_DIRECTIONS; ++i)
        total_ped_waiting += snap.pedestrian_pending[i];

    snprintf(buf, sizeof(buf), "Ped waiting: N=%d  S=%d  E=%d  W=%d  (Total=%d)",
             snap.pedestrian_pending[0], snap.pedestrian_pending[1],
             snap.pedestrian_pending[2], snap.pedestrian_pending[3],
             total_ped_waiting);
    if (total_ped_waiting > 0 && !snap.pedestrian_active)
        glColor3f(1.0f, 0.85f, 0.20f);  
    else if (snap.pedestrian_active)
        glColor3f(0.20f, 0.90f, 0.20f); 
    else
        glColor3f(0.95f, 0.95f, 0.95f);
    draw_text(20, WIN_H - 120, buf);
    glColor3f(0.95f, 0.95f, 0.95f);

    if (snap.pedestrian_active) {
        snprintf(buf, sizeof(buf), "*** PEDESTRIAN CROSSING ACTIVE *** (%d s left)",
                 snap.pedestrian_remaining);
        glColor3f(0.20f, 1.0f, 0.20f);
        draw_text(20, WIN_H - 143, buf);
        glColor3f(0.95f, 0.95f, 0.95f);
    }

    if (snap.emergency_active) {
        glColor3f(1, 0.3f, 0.3f);
        snprintf(buf, sizeof(buf), "*** EMERGENCY from %s ***",
                 DIR_NAMES[snap.emergency_direction]);
        draw_text_big(20, WIN_H - 168, buf);
        glColor3f(0.95f, 0.95f, 0.95f);
    }

    snprintf(buf, sizeof(buf),
             "Served: %d veh   %d ped   |   Emergencies: %d   Faults: %d",
             snap.total_vehicles_served, snap.total_pedestrians_served,
             snap.total_emergencies, snap.safety_violations);
    draw_text(20, WIN_H - 200, buf);

    if (show_help) {
        /* bottom-right help panel */
        glColor4f(0, 0, 0, 0.55f);
        glEnable(GL_BLEND);
        glBegin(GL_QUADS);
            glVertex2f(WIN_W - 320, 10);
            glVertex2f(WIN_W - 10,  10);
            glVertex2f(WIN_W - 10,  170);
            glVertex2f(WIN_W - 320, 170);
        glEnd();
        glDisable(GL_BLEND);
        glColor3f(1, 1, 1);
        draw_text(WIN_W - 310, 150, "Keyboard controls:");
        draw_text(WIN_W - 310, 130, "  n / s / e / w : add a vehicle");
        draw_text(WIN_W - 310, 110, "  N / S / E / W : pedestrian button");
        draw_text(WIN_W - 310,  90, "  1 / 2 / 3 / 4 : emergency from N/S/E/W");
        draw_text(WIN_W - 310,  70, "  arrows : rotate camera");
        draw_text(WIN_W - 310,  50, "  +  /  - : zoom");
        draw_text(WIN_W - 310,  30, "  h : toggle help     q : quit");
    }

    /* recent events panel — bottom left */
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0, 0, 0, 0.50f);
    glBegin(GL_QUADS);
        glVertex2f(10,   10);
        glVertex2f(360,  10);
        glVertex2f(360,  10 + MAX_RECENT_EVENTS * 18 + 10);
        glVertex2f(10,   10 + MAX_RECENT_EVENTS * 18 + 10);
    glEnd();
    glDisable(GL_BLEND);

    glColor3f(0.75f, 0.85f, 1.0f);
    draw_text(20, 10 + MAX_RECENT_EVENTS * 18, "Recent events:");
    for (int i = 0; i < recent_count; i++) {
        int idx = (recent_head - recent_count + i + MAX_RECENT_EVENTS)
                  % MAX_RECENT_EVENTS;
        glColor3f(0.90f, 0.90f, 0.90f);
        draw_text(20, 10 + (MAX_RECENT_EVENTS - 1 - i) * 18, recent_events[idx]);
    }

    glMatrixMode(GL_PROJECTION); glPopMatrix();
    glMatrixMode(GL_MODELVIEW);  glPopMatrix();
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
}

/* ---------- snapshot ------------------------------------------------- */
static void take_snapshot(void)
{
    if (!S) return;
    if (sem_lock(semid) == 0) {
        snap = *S;
        sem_unlock(semid);
    }
}

/* ---------- display callback ---------------------------------------- */
static void on_display(void)
{
    take_snapshot();
    poll_recent_events();
    t_animation += 0.05f;

    glClearColor(0.55f, 0.75f, 0.9f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    /* camera */
    float cx = cam_dist * sinf(cam_rotate * M_PI / 180.0f) *
               cosf(cam_angle  * M_PI / 180.0f);
    float cz = cam_dist * cosf(cam_rotate * M_PI / 180.0f) *
               cosf(cam_angle  * M_PI / 180.0f);
    float cy = cam_dist * sinf(cam_angle  * M_PI / 180.0f);
    gluLookAt(cx, cy, cz,  0, 0, 0,  0, 1, 0);

    draw_ground();
    draw_roads();

    /* traffic lights */
    draw_traffic_light( INTER_HALF + 1.5f,  INTER_HALF + 1.5f,
                        180.0f, snap.light[DIR_NORTH]);
    draw_traffic_light(-INTER_HALF - 1.5f, -INTER_HALF - 1.5f,
                          0.0f, snap.light[DIR_SOUTH]);
    draw_traffic_light(-INTER_HALF - 1.5f,  INTER_HALF + 1.5f,
                        -90.0f, snap.light[DIR_EAST]);
    draw_traffic_light( INTER_HALF + 1.5f, -INTER_HALF - 1.5f,
                         90.0f, snap.light[DIR_WEST]);

    /* queued vehicles */
    for (int i = 0; i < NUM_DIRECTIONS; ++i)
        draw_vehicles_queue((direction_t)i, snap.waiting_vehicles[i]);

    draw_emergency_vehicle();
    draw_pedestrians();

    draw_hud();

    glutSwapBuffers();
}

static void on_reshape(int w, int h)
{
    WIN_W = w; WIN_H = h;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    gluPerspective(45.0, (double)w / (h > 0 ? h : 1), 0.5, 200.0);
    glMatrixMode(GL_MODELVIEW);
}

/* ---------- input ---------------------------------------------------- */
static void inject_event(long mtype, direction_t d)
{
    evt_msg_t e;
    memset(&e, 0, sizeof(e));
    e.mtype     = mtype;
    e.direction = d;
    e.count     = 1;
    e.timestamp = time(NULL);
    snprintf(e.info, sizeof(e.info), "from-gui");
    msgsnd(qevt, &e, sizeof(e) - sizeof(long), IPC_NOWAIT);
    send_log(qlog, "GUI", 0, "Injected %s event for %s",
             (mtype == MTYPE_EVT_VEHICLE)    ? "vehicle"    :
             (mtype == MTYPE_EVT_PEDESTRIAN) ? "pedestrian" :
             (mtype == MTYPE_EVT_EMERGENCY)  ? "emergency"  : "?",
             DIR_NAMES[d]);
}

static void on_key(unsigned char k, int x, int y)
{
    (void)x; (void)y;
    switch (k) {
    case 'n': inject_event(MTYPE_EVT_VEHICLE, DIR_NORTH); break;
    case 's': inject_event(MTYPE_EVT_VEHICLE, DIR_SOUTH); break;
    case 'e': inject_event(MTYPE_EVT_VEHICLE, DIR_EAST);  break;
    case 'w': inject_event(MTYPE_EVT_VEHICLE, DIR_WEST);  break;
    case 'N': inject_event(MTYPE_EVT_PEDESTRIAN, DIR_NORTH); break;
    case 'S': inject_event(MTYPE_EVT_PEDESTRIAN, DIR_SOUTH); break;
    case 'E': inject_event(MTYPE_EVT_PEDESTRIAN, DIR_EAST);  break;
    case 'W': inject_event(MTYPE_EVT_PEDESTRIAN, DIR_WEST);  break;
    case '1': inject_event(MTYPE_EVT_EMERGENCY, DIR_NORTH); break;
    case '2': inject_event(MTYPE_EVT_EMERGENCY, DIR_SOUTH); break;
    case '3': inject_event(MTYPE_EVT_EMERGENCY, DIR_EAST);  break;
    case '4': inject_event(MTYPE_EVT_EMERGENCY, DIR_WEST);  break;
    case '+': case '=': cam_dist -= 1.5f; if (cam_dist < 8)  cam_dist = 8;  break;
    case '-': case '_': cam_dist += 1.5f; if (cam_dist > 60) cam_dist = 60; break;
    case 'h': case 'H': show_help = !show_help; break;
    case 'q': case 'Q': case 27:
        if (S) { sem_lock(semid); S->running = 0; sem_unlock(semid); }
        exit(0);
    }
}

static void on_special(int k, int x, int y)
{
    (void)x; (void)y;
    switch (k) {
    case GLUT_KEY_LEFT:  cam_rotate -= 4.0f; break;
    case GLUT_KEY_RIGHT: cam_rotate += 4.0f; break;
    case GLUT_KEY_UP:    cam_angle  += 2.0f; if (cam_angle >  80) cam_angle =  80; break;
    case GLUT_KEY_DOWN:  cam_angle  -= 2.0f; if (cam_angle <  10) cam_angle =  10; break;
    }
}

/* ---------- timer (animation) --------------------------------------- */
static void on_timer(int v)
{
    (void)v;
    glutPostRedisplay();
    glutTimerFunc(60, on_timer, 0);
}

/* ---------- lighting ------------------------------------------------- */
static void init_gl(void)
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);

    GLfloat lpos[]  = { 10.0f, 20.0f, 10.0f, 0.0f };
    GLfloat lamb[]  = { 0.3f, 0.3f, 0.35f, 1.0f };
    GLfloat ldiff[] = { 0.9f, 0.9f, 0.85f, 1.0f };
    GLfloat lspec[] = { 0.5f, 0.5f, 0.5f, 1.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, lpos);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  lamb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  ldiff);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lspec);

    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glShadeModel(GL_SMOOTH);
    glClearDepth(1.0);
}

int main(int argc, char **argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(WIN_W, WIN_H);
    glutCreateWindow("ENCS4330 — Real-Time Traffic Light Control System");

    /* attach IPC */
    shmid = ipc_shm_attach_existing();
    semid = ipc_sem_attach_existing();
    qevt  = ipc_q_attach_existing(IPC_ID_MQ_EVT);
    qlog  = ipc_q_attach_existing(IPC_ID_MQ_LOG);
    if (shmid < 0 || semid < 0 || qevt < 0 || qlog < 0) {
        fprintf(stderr, "gui: IPC attach failed — start main first\n");
        return 1;
    }
    S = (shared_state_t *)shmat(shmid, NULL, 0);
    if (S == (void *)-1) { perror("shmat"); return 1; }
    send_log(qlog, "GUI", 0, "OpenGL GUI attached");

    init_gl();
    glutDisplayFunc(on_display);
    glutReshapeFunc(on_reshape);
    glutKeyboardFunc(on_key);
    glutSpecialFunc(on_special);
    glutTimerFunc(60, on_timer, 0);
    glutMainLoop();
    return 0;
}

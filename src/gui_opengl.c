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
#define MAX_VEHICLES_PER_DIR 16

static int vehicle_types[NUM_DIRECTIONS][MAX_VEHICLES_PER_DIR];
static int vehicle_colors[NUM_DIRECTIONS][MAX_VEHICLES_PER_DIR];
static int vehicle_initialized[NUM_DIRECTIONS][MAX_VEHICLES_PER_DIR];
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

/* ---------- ticket / red-light-runner state ------------------------- */

static int   ticket_local_active = 0;
static char  ticket_local_plate[16];
static direction_t ticket_local_dir = DIR_NORTH;
static float ticket_runner_pos  = 0.0f;
static float ticket_police_pos  = 0.0f;
static float ticket_runner_speed = 0.18f;
static float ticket_police_speed = 0.28f;

static void gen_random_plate(char *out, size_t n)
{
    if (n < 8) { if (n) out[0] = '\0'; return; }
    out[0] = 'A' + (rand() % 26);
    out[1] = 'A' + (rand() % 26);
    out[2] = '-';
    out[3] = '0' + (rand() % 10);
    out[4] = '0' + (rand() % 10);
    out[5] = '0' + (rand() % 10);
    out[6] = '0' + (rand() % 10);
    out[7] = '\0';
}

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

/* ---------- vehicle variety ------------------------------------------ */

typedef enum {
    VEHICLE_SEDAN = 0,
    VEHICLE_SUV,
    VEHICLE_TRUCK,
    VEHICLE_BUS,
    VEHICLE_BIKE
} vehicle_type_t;

static void draw_sedan(float r, float g, float b)
{
    set_color(r, g, b);
    glPushMatrix();
    glTranslatef(0, 0.28f, 0);
    draw_box(1.5f, 0.40f, 0.80f);
    glPopMatrix();

    set_color(r * 0.4f, g * 0.4f, b * 0.4f);
    glPushMatrix();
    glTranslatef(-0.05f, 0.60f, 0);
    draw_box(0.85f, 0.32f, 0.70f);
    glPopMatrix();

    set_color(0.05f, 0.05f, 0.05f);
    for (int sx = -1; sx <= 1; sx += 2)
    for (int sz = -1; sz <= 1; sz += 2) {
        glPushMatrix();
        glTranslatef(sx * 0.50f, 0.12f, sz * 0.38f);
        glutSolidSphere(0.14f, 10, 10);
        glPopMatrix();
    }
}

static void draw_suv(float r, float g, float b)
{
    set_color(r, g, b);
    glPushMatrix();
    glTranslatef(0, 0.40f, 0);
    draw_box(1.7f, 0.60f, 0.90f);
    glPopMatrix();

    set_color(r * 0.3f, g * 0.3f, b * 0.3f);
    glPushMatrix();
    glTranslatef(-0.10f, 0.85f, 0);
    draw_box(1.0f, 0.45f, 0.80f);
    glPopMatrix();

    set_color(0.05f, 0.05f, 0.05f);
    for (int sx = -1; sx <= 1; sx += 2)
    for (int sz = -1; sz <= 1; sz += 2) {
        glPushMatrix();
        glTranslatef(sx * 0.60f, 0.16f, sz * 0.42f);
        glutSolidSphere(0.18f, 10, 10);
        glPopMatrix();
    }
}

static void draw_truck(float r, float g, float b)
{
    set_color(r, g, b);
    glPushMatrix();
    glTranslatef(0.60f, 0.45f, 0);
    draw_box(0.90f, 0.70f, 0.90f);
    glPopMatrix();

    set_color(r * 0.7f, g * 0.7f, b * 0.7f);
    glPushMatrix();
    glTranslatef(-0.60f, 0.45f, 0);
    draw_box(1.50f, 0.65f, 0.92f);
    glPopMatrix();

    set_color(0.05f, 0.05f, 0.05f);
    for (int sx = -1; sx <= 1; sx += 2) {
        glPushMatrix(); glTranslatef(sx * 0.95f, 0.16f, -0.42f); glutSolidSphere(0.18f, 10, 10); glPopMatrix();
        glPushMatrix(); glTranslatef(sx * 0.95f, 0.16f,  0.42f); glutSolidSphere(0.18f, 10, 10); glPopMatrix();
        glPushMatrix(); glTranslatef(sx * 0.25f, 0.16f, -0.42f); glutSolidSphere(0.18f, 10, 10); glPopMatrix();
        glPushMatrix(); glTranslatef(sx * 0.25f, 0.16f,  0.42f); glutSolidSphere(0.18f, 10, 10); glPopMatrix();
    }
}

static void draw_bus(float r, float g, float b)
{
    set_color(r, g, b);
    glPushMatrix();
    glTranslatef(0, 0.70f, 0);
    draw_box(2.80f, 1.10f, 1.00f);
    glPopMatrix();

    set_color(r * 0.2f, g * 0.2f, b * 0.2f);
    glPushMatrix();
    glTranslatef(0, 1.0f, 0);
    draw_box(2.85f, 0.45f, 0.95f);
    glPopMatrix();

    set_color(0.05f, 0.05f, 0.05f);
    float wheel_positions[] = {-1.1f, -0.4f, 0.4f, 1.1f};
    for (int i = 0; i < 4; i++) {
        for (int sz = -1; sz <= 1; sz += 2) {
            glPushMatrix();
            glTranslatef(wheel_positions[i], 0.18f, sz * 0.48f);
            glutSolidSphere(0.20f, 10, 10);
            glPopMatrix();
        }
    }
}

static void draw_bicycle(float r, float g, float b)
{
    set_color(r, g, b);
    glPushMatrix();
    glTranslatef(0, 0.35f, 0);
    glScalef(0.5f, 0.02f, 0.02f);
    glutSolidCube(1.0f);
    glPopMatrix();

    set_color(0.10f, 0.10f, 0.10f);
    glPushMatrix();
    glTranslatef(0.22f, 0.15f, 0);
    glScalef(0.02f, 1.0f, 1.0f);
    glutSolidSphere(0.16f, 8, 8);
    glPopMatrix();
    glPushMatrix();
    glTranslatef(-0.22f, 0.15f, 0);
    glScalef(0.02f, 1.0f, 1.0f);
    glutSolidSphere(0.16f, 8, 8);
    glPopMatrix();

    set_color(0.20f, 0.35f, 0.60f);
    glPushMatrix();
    glTranslatef(0, 0.60f, 0);
    glScalef(0.18f, 0.50f, 0.18f);
    glutSolidCube(1.0f);
    glPopMatrix();

    set_color(0.85f, 0.75f, 0.65f);
    glPushMatrix();
    glTranslatef(0, 0.95f, 0);
    glutSolidSphere(0.12f, 8, 8);
    glPopMatrix();
}

static void draw_vehicle_type(int type_index, float r, float g, float b)
{
    switch (type_index % 5) {
        case VEHICLE_SEDAN: draw_sedan(r, g, b); break;
        case VEHICLE_SUV:   draw_suv(r, g, b);   break;
        case VEHICLE_TRUCK: draw_truck(r, g, b); break;
        case VEHICLE_BUS:   draw_bus(r, g, b);   break;
        case VEHICLE_BIKE:  draw_bicycle(r, g, b); break;
    }
}

/* ---------- world drawing -------------------------------------------- */

#define ROAD_HALF_WIDTH   3.0f
#define ROAD_LENGTH       18.0f
#define INTER_HALF        3.0f

static void draw_ground(void)
{
    set_color(0.18f, 0.42f, 0.18f);
    glBegin(GL_QUADS);
        glNormal3f(0, 1, 0);
        glVertex3f(-25, 0, -25);
        glVertex3f( 25, 0, -25);
        glVertex3f( 25, 0,  25);
        glVertex3f(-25, 0,  25);
    glEnd();

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
    set_color(0.14f, 0.14f, 0.16f);

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

    /* lane dividers (dashed yellow) */
    set_color(0.95f, 0.85f, 0.15f);
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

    /* stop lines (white) */
    set_color(0.92f, 0.92f, 0.92f);
    glBegin(GL_QUADS); glNormal3f(0,1,0);
    glVertex3f( 0.1f,            0.05f, INTER_HALF + 0.2f);
    glVertex3f( ROAD_HALF_WIDTH, 0.05f, INTER_HALF + 0.2f);
    glVertex3f( ROAD_HALF_WIDTH, 0.05f, INTER_HALF + 0.5f);
    glVertex3f( 0.1f,            0.05f, INTER_HALF + 0.5f);
    glEnd();
    glBegin(GL_QUADS); glNormal3f(0,1,0);
    glVertex3f(-ROAD_HALF_WIDTH, 0.05f, -INTER_HALF - 0.5f);
    glVertex3f(-0.1f,            0.05f, -INTER_HALF - 0.5f);
    glVertex3f(-0.1f,            0.05f, -INTER_HALF - 0.2f);
    glVertex3f(-ROAD_HALF_WIDTH, 0.05f, -INTER_HALF - 0.2f);
    glEnd();
    glBegin(GL_QUADS); glNormal3f(0,1,0);
    glVertex3f(-INTER_HALF - 0.5f, 0.05f, 0.1f);
    glVertex3f(-INTER_HALF - 0.2f, 0.05f, 0.1f);
    glVertex3f(-INTER_HALF - 0.2f, 0.05f, ROAD_HALF_WIDTH);
    glVertex3f(-INTER_HALF - 0.5f, 0.05f, ROAD_HALF_WIDTH);
    glEnd();
    glBegin(GL_QUADS); glNormal3f(0,1,0);
    glVertex3f( INTER_HALF + 0.2f, 0.05f, -ROAD_HALF_WIDTH);
    glVertex3f( INTER_HALF + 0.5f, 0.05f, -ROAD_HALF_WIDTH);
    glVertex3f( INTER_HALF + 0.5f, 0.05f, -0.1f);
    glVertex3f( INTER_HALF + 0.2f, 0.05f, -0.1f);
    glEnd();

    /* zebra crossings */
    set_color(0.92f, 0.92f, 0.92f);
    for (float x = -ROAD_HALF_WIDTH + 0.3f; x < ROAD_HALF_WIDTH; x += 0.7f) {
        glBegin(GL_QUADS); glNormal3f(0,1,0);
        glVertex3f(x,        0.06f, -INTER_HALF - 0.8f);
        glVertex3f(x + 0.4f, 0.06f, -INTER_HALF - 0.8f);
        glVertex3f(x + 0.4f, 0.06f, -INTER_HALF - 0.1f);
        glVertex3f(x,        0.06f, -INTER_HALF - 0.1f);
        glEnd();
    }
    for (float x = -ROAD_HALF_WIDTH + 0.3f; x < ROAD_HALF_WIDTH; x += 0.7f) {
        glBegin(GL_QUADS); glNormal3f(0,1,0);
        glVertex3f(x,        0.06f,  INTER_HALF + 0.1f);
        glVertex3f(x + 0.4f, 0.06f,  INTER_HALF + 0.1f);
        glVertex3f(x + 0.4f, 0.06f,  INTER_HALF + 0.8f);
        glVertex3f(x,        0.06f,  INTER_HALF + 0.8f);
        glEnd();
    }
    for (float z = -ROAD_HALF_WIDTH + 0.3f; z < ROAD_HALF_WIDTH; z += 0.7f) {
        glBegin(GL_QUADS); glNormal3f(0,1,0);
        glVertex3f( INTER_HALF + 0.1f, 0.06f, z);
        glVertex3f( INTER_HALF + 0.8f, 0.06f, z);
        glVertex3f( INTER_HALF + 0.8f, 0.06f, z + 0.4f);
        glVertex3f( INTER_HALF + 0.1f, 0.06f, z + 0.4f);
        glEnd();
    }
    for (float z = -ROAD_HALF_WIDTH + 0.3f; z < ROAD_HALF_WIDTH; z += 0.7f) {
        glBegin(GL_QUADS); glNormal3f(0,1,0);
        glVertex3f(-INTER_HALF - 0.8f, 0.06f, z);
        glVertex3f(-INTER_HALF - 0.1f, 0.06f, z);
        glVertex3f(-INTER_HALF - 0.1f, 0.06f, z + 0.4f);
        glVertex3f(-INTER_HALF - 0.8f, 0.06f, z + 0.4f);
        glEnd();
    }

    /* bike lanes: painted green strips */
    set_color(0.25f, 0.55f, 0.35f);
    float bike_width = 0.75f;

    glBegin(GL_QUADS); glNormal3f(0,1,0);
    glVertex3f( ROAD_HALF_WIDTH - bike_width, 0.04f, -ROAD_LENGTH);
    glVertex3f( ROAD_HALF_WIDTH,              0.04f, -ROAD_LENGTH);
    glVertex3f( ROAD_HALF_WIDTH,              0.04f,  ROAD_LENGTH);
    glVertex3f( ROAD_HALF_WIDTH - bike_width, 0.04f,  ROAD_LENGTH);
    glEnd();
    glBegin(GL_QUADS); glNormal3f(0,1,0);
    glVertex3f(-ROAD_HALF_WIDTH,              0.04f, -ROAD_LENGTH);
    glVertex3f(-ROAD_HALF_WIDTH + bike_width, 0.04f, -ROAD_LENGTH);
    glVertex3f(-ROAD_HALF_WIDTH + bike_width, 0.04f,  ROAD_LENGTH);
    glVertex3f(-ROAD_HALF_WIDTH,              0.04f,  ROAD_LENGTH);
    glEnd();
    glBegin(GL_QUADS); glNormal3f(0,1,0);
    glVertex3f(-ROAD_LENGTH, 0.04f,  ROAD_HALF_WIDTH - bike_width);
    glVertex3f( ROAD_LENGTH, 0.04f,  ROAD_HALF_WIDTH - bike_width);
    glVertex3f( ROAD_LENGTH, 0.04f,  ROAD_HALF_WIDTH);
    glVertex3f(-ROAD_LENGTH, 0.04f,  ROAD_HALF_WIDTH);
    glEnd();
    glBegin(GL_QUADS); glNormal3f(0,1,0);
    glVertex3f(-ROAD_LENGTH, 0.04f, -ROAD_HALF_WIDTH);
    glVertex3f( ROAD_LENGTH, 0.04f, -ROAD_HALF_WIDTH);
    glVertex3f( ROAD_LENGTH, 0.04f, -ROAD_HALF_WIDTH + bike_width);
    glVertex3f(-ROAD_LENGTH, 0.04f, -ROAD_HALF_WIDTH + bike_width);
    glEnd();

    /* white bicycle symbols on bike lanes */
    set_color(0.95f, 0.95f, 0.95f);
    float bike_symbol_positions[] = {-12.0f, -6.0f, 6.0f, 12.0f};
    for (int i = 0; i < 4; i++) {
        float pos = bike_symbol_positions[i];
        float symbol_scale = 0.25f;

        glPushMatrix();
        glTranslatef(ROAD_HALF_WIDTH - 0.37f, 0.05f, pos);
        glRotatef(90, 0, 1, 0);
        glScalef(symbol_scale, 1.0f, symbol_scale);
        glBegin(GL_LINE_LOOP);
        for (int a = 0; a < 16; a++) { float angle = a * 3.14159f * 2.0f / 16.0f; glVertex3f(0.8f + 0.5f * cosf(angle), 0, 0.5f * sinf(angle)); }
        glEnd();
        glBegin(GL_LINE_LOOP);
        for (int a = 0; a < 16; a++) { float angle = a * 3.14159f * 2.0f / 16.0f; glVertex3f(-0.8f + 0.5f * cosf(angle), 0, 0.5f * sinf(angle)); }
        glEnd();
        glBegin(GL_LINES); glVertex3f(-0.8f, 0, 0); glVertex3f(0, 0, 0); glVertex3f(0, 0, 0); glVertex3f(0.8f, 0, 0); glEnd();
        glPopMatrix();

        glPushMatrix();
        glTranslatef(-ROAD_HALF_WIDTH + 0.37f, 0.05f, pos);
        glRotatef(-90, 0, 1, 0);
        glScalef(symbol_scale, 1.0f, symbol_scale);
        glBegin(GL_LINE_LOOP);
        for (int a = 0; a < 16; a++) { float angle = a * 3.14159f * 2.0f / 16.0f; glVertex3f(0.8f + 0.5f * cosf(angle), 0, 0.5f * sinf(angle)); }
        glEnd();
        glBegin(GL_LINE_LOOP);
        for (int a = 0; a < 16; a++) { float angle = a * 3.14159f * 2.0f / 16.0f; glVertex3f(-0.8f + 0.5f * cosf(angle), 0, 0.5f * sinf(angle)); }
        glEnd();
        glBegin(GL_LINES); glVertex3f(-0.8f, 0, 0); glVertex3f(0, 0, 0); glVertex3f(0, 0, 0); glVertex3f(0.8f, 0, 0); glEnd();
        glPopMatrix();

        glPushMatrix();
        glTranslatef(pos, 0.05f, ROAD_HALF_WIDTH - 0.37f);
        glRotatef(180, 0, 1, 0);
        glScalef(symbol_scale, 1.0f, symbol_scale);
        glBegin(GL_LINE_LOOP);
        for (int a = 0; a < 16; a++) { float angle = a * 3.14159f * 2.0f / 16.0f; glVertex3f(0.8f + 0.5f * cosf(angle), 0, 0.5f * sinf(angle)); }
        glEnd();
        glBegin(GL_LINE_LOOP);
        for (int a = 0; a < 16; a++) { float angle = a * 3.14159f * 2.0f / 16.0f; glVertex3f(-0.8f + 0.5f * cosf(angle), 0, 0.5f * sinf(angle)); }
        glEnd();
        glBegin(GL_LINES); glVertex3f(-0.8f, 0, 0); glVertex3f(0, 0, 0); glVertex3f(0, 0, 0); glVertex3f(0.8f, 0, 0); glEnd();
        glPopMatrix();

        glPushMatrix();
        glTranslatef(pos, 0.05f, -ROAD_HALF_WIDTH + 0.37f);
        glScalef(symbol_scale, 1.0f, symbol_scale);
        glBegin(GL_LINE_LOOP);
        for (int a = 0; a < 16; a++) { float angle = a * 3.14159f * 2.0f / 16.0f; glVertex3f(0.8f + 0.5f * cosf(angle), 0, 0.5f * sinf(angle)); }
        glEnd();
        glBegin(GL_LINE_LOOP);
        for (int a = 0; a < 16; a++) { float angle = a * 3.14159f * 2.0f / 16.0f; glVertex3f(-0.8f + 0.5f * cosf(angle), 0, 0.5f * sinf(angle)); }
        glEnd();
        glBegin(GL_LINES); glVertex3f(-0.8f, 0, 0); glVertex3f(0, 0, 0); glVertex3f(0, 0, 0); glVertex3f(0.8f, 0, 0); glEnd();
        glPopMatrix();
    }
}

static void draw_traffic_light(float px, float pz, float yaw,
                               light_color_t color)
{
    glPushMatrix();
    glTranslatef(px, 0, pz);
    glRotatef(yaw, 0, 1, 0);

    set_color(0.20f, 0.20f, 0.20f);
    glPushMatrix();
        glTranslatef(0, 1.6f, 0);
        draw_box(0.18f, 3.2f, 0.18f);
    glPopMatrix();

    set_color(0.10f, 0.10f, 0.10f);
    glPushMatrix();
        glTranslatef(0, 3.6f, 0.35f);
        draw_box(0.55f, 1.5f, 0.45f);
    glPopMatrix();

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
            set_emissive(colors[i][0] * 0.8f, colors[i][1] * 0.8f, colors[i][2] * 0.8f);
        } else {
            set_color(colors[i][0] * 0.25f, colors[i][1] * 0.25f, colors[i][2] * 0.25f);
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
    set_color(r, g, b);
    glPushMatrix();
    glTranslatef(0, 0.30f, 0);
    draw_box(1.6f, 0.45f, 0.85f);
    glPopMatrix();

    set_color(r * 0.5f, g * 0.5f, b * 0.5f);
    glPushMatrix();
    glTranslatef(-0.05f, 0.65f, 0);
    draw_box(0.9f, 0.35f, 0.75f);
    glPopMatrix();

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
    int blink = (t_ms / 250) & 1;
    glPushMatrix();
    glTranslatef(0, 1.0f, 0);
    if (blink) { set_color(1.0f, 0.1f, 0.1f); set_emissive(1.0f, 0.1f, 0.1f); }
    else       { set_color(0.1f, 0.3f, 1.0f); set_emissive(0.1f, 0.3f, 1.0f); }
    draw_box(0.45f, 0.10f, 0.55f);
    clear_emissive();
    glPopMatrix();
}

/* ======================================================================
 * Per-direction car animation state.
 *
 * travel_t advances monotonically while the phase is green and resets to
 * zero when the light goes red.  Car i waits until travel_t > i*spacing
 * before it starts moving, guaranteeing perfectly constant-speed,
 * gap-free flow with zero pauses or cycle resets.
 * ====================================================================== */
typedef struct {
    float travel_t;
    int   was_moving;
} car_anim_t;

static car_anim_t car_anim[NUM_DIRECTIONS];
static int car_anim_inited = 0;

static void update_car_anims(void)
{
    if (!car_anim_inited) {
        for (int i = 0; i < NUM_DIRECTIONS; ++i) {
            car_anim[i].travel_t   = 0.0f;
            car_anim[i].was_moving = 0;
        }
        car_anim_inited = 1;
    }

    const float CAR_SPEED = 0.22f;   /* world-units per frame at ~60 fps */

    for (int i = 0; i < NUM_DIRECTIONS; ++i) {
        int moving = 0;
        if (i == DIR_NORTH || i == DIR_SOUTH)
            moving = (snap.current_phase == PHASE_NS_GREEN);
        else
            moving = (snap.current_phase == PHASE_EW_GREEN);

        if (moving)
            car_anim[i].travel_t += CAR_SPEED;
        else
            car_anim[i].travel_t = 0.0f;   /* red: snap queue back to stop line */

        car_anim[i].was_moving = moving;
    }
}

static void draw_vehicles_queue(direction_t d, int count)
{
    if (count <= 0) return;
    int shown = count > 6 ? 6 : count;

    float palette[][3] = {
        {0.2f, 0.5f, 0.9f},  {0.95f, 0.95f, 0.95f}, {0.15f, 0.55f, 0.25f},
        {0.85f, 0.6f, 0.15f},{0.6f,  0.2f,  0.7f},  {0.3f,  0.3f,  0.35f}
    };

    int   moving  = car_anim[d].was_moving;
    float spacing = 2.2f;

    for (int i = 0; i < shown; ++i) {
       if (!vehicle_initialized[d][i]) {
            vehicle_types[d][i]  = rand() % 5;
            vehicle_colors[d][i] = rand() % 6;
            vehicle_initialized[d][i] = 1;
        }

    int vehicle_type = vehicle_types[d][i];

    /* Bikes belong in the dedicated lane */
    if (vehicle_type == VEHICLE_BIKE) continue;

        float base_dist = INTER_HALF + 1.5f + i * spacing;

        float dist;
        if (!moving) {
            /* Red: hold at queue position */
            dist = base_dist;
        } else {
            /* Car i departs as soon as travel_t passes i*spacing,
             * keeping constant gaps with no pauses between cars.  */
            float car_travel = car_anim[d].travel_t - (float)i * spacing;
            if (car_travel <= 0.0f) {
                dist = base_dist;           /* hasn't started yet */
            } else {
                dist = base_dist - car_travel;
                if (dist < -ROAD_LENGTH) continue;  /* exited road */
            }
        }

        glPushMatrix();
        switch (d) {
        case DIR_NORTH: glTranslatef( 1.3f, 0,  dist); glRotatef( 90, 0,1,0); break;
        case DIR_SOUTH: glTranslatef(-1.3f, 0, -dist); glRotatef(-90, 0,1,0); break;
        case DIR_EAST:  glTranslatef(-dist,  0,  1.3f);                        break;
        case DIR_WEST:  glTranslatef( dist,  0, -1.3f); glRotatef(180, 0,1,0); break;
        }
        float *c = palette[vehicle_colors[d][i]];
        draw_vehicle_type(vehicle_type, c[0], c[1], c[2]);
        glPopMatrix();
    }
}

/* ======================================================================
 * Per-direction bike animation state.
 *
 * Mirrors the car logic exactly.  NS bikes (DIR_NORTH / DIR_SOUTH) obey
 * the NS green phase; EW bikes obey the EW green phase.  On red they
 * queue before the intersection stop line and wait.
 * ====================================================================== */
typedef struct {
    float travel_t;
    int   was_moving;
} bike_anim_t;

static bike_anim_t bike_anim[NUM_DIRECTIONS];
static int bike_anim_inited = 0;

static void update_bike_anims(void)
{
    if (!bike_anim_inited) {
        for (int i = 0; i < NUM_DIRECTIONS; ++i) {
            bike_anim[i].travel_t   = 0.0f;
            bike_anim[i].was_moving = 0;
        }
        bike_anim_inited = 1;
    }

    const float BIKE_SPEED = 0.14f;   /* slower than cars */

    for (int i = 0; i < NUM_DIRECTIONS; ++i) {
        int moving = 0;
        if (i == DIR_NORTH || i == DIR_SOUTH)
            moving = (snap.current_phase == PHASE_NS_GREEN);
        else
            moving = (snap.current_phase == PHASE_EW_GREEN);

        if (moving)
            bike_anim[i].travel_t += BIKE_SPEED;
        else
            bike_anim[i].travel_t = 0.0f;

        bike_anim[i].was_moving = moving;
    }
}

static void draw_bike_lane_traffic(void)
{
    update_bike_anims();

    const int   N_BIKES  = 3;          /* bikes per direction           */
    const float SPACING  = 5.0f;       /* distance between consecutive  */
    const float LANE_OFF = 2.4f;       /* lateral offset into bike lane */

    /* One distinct colour per direction */
    float colors[NUM_DIRECTIONS][3] = {
        {0.25f, 0.45f, 0.75f},  /* NORTH: blue   */
        {0.75f, 0.25f, 0.45f},  /* SOUTH: pink   */
        {0.45f, 0.75f, 0.25f},  /* EAST:  green  */
        {0.65f, 0.35f, 0.75f},  /* WEST:  purple */
    };

    for (int d = 0; d < NUM_DIRECTIONS; ++d) {
        int   moving = bike_anim[d].was_moving;
        float *col   = colors[d];

        for (int i = 0; i < N_BIKES; ++i) {
            float base_dist = INTER_HALF + 2.0f + i * SPACING;

            float dist;
            if (!moving) {
                /* Red: queue before the intersection */
                dist = base_dist;
            } else {
                float bike_travel = bike_anim[d].travel_t - (float)i * SPACING;
                if (bike_travel <= 0.0f) {
                    dist = base_dist;       /* hasn't departed yet */
                } else {
                    dist = base_dist - bike_travel;
                    if (dist < -ROAD_LENGTH) continue;  /* off screen */
                }
            }

            if (dist > ROAD_LENGTH) continue;   /* not spawned yet */

            glPushMatrix();
            switch (d) {
            case DIR_NORTH:
                glTranslatef( LANE_OFF, 0,  dist);
                glRotatef(90, 0, 1, 0);
                break;
            case DIR_SOUTH:
                glTranslatef(-LANE_OFF, 0, -dist);
                glRotatef(-90, 0, 1, 0);
                break;
            case DIR_EAST:
                glTranslatef(-dist, 0,  LANE_OFF);
                break;
            case DIR_WEST:
                glTranslatef( dist, 0, -LANE_OFF);
                glRotatef(180, 0, 1, 0);
                break;
            }
            draw_bicycle(col[0], col[1], col[2]);
            glPopMatrix();
        }
    }
}

static void draw_emergency_vehicle(void)
{
    if (!snap.emergency_active) return;
    direction_t d = snap.emergency_direction;

    static float emerg_start_t = -1.0f;
    static int   last_emerg    = 0;
    static direction_t last_dir = DIR_NORTH;

    if (!last_emerg && snap.emergency_active) {
        emerg_start_t = t_animation;
        last_dir = d;
    }
    if (snap.emergency_active && d != last_dir) {
        emerg_start_t = t_animation;
        last_dir = d;
    }
    last_emerg = snap.emergency_active;

    float total_travel = 14.0f;
    float t     = fmodf(t_animation - emerg_start_t, total_travel);
    float speed = (ROAD_LENGTH * 2.0f + INTER_HALF * 2.0f) / total_travel;
    float pos   = t * speed;
    float dist  = ROAD_LENGTH - pos;

    int t_ms = (int)(t_animation * 1000);

    glPushMatrix();
    switch (d) {
    case DIR_NORTH:
        if (dist > 0) glTranslatef(-1.3f, 0,  dist);
        else          glTranslatef( 1.3f, 0,  dist);
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

/* ---------- ticket chase visuals ------------------------------------ */

static void draw_police_car(int t_ms)
{
    set_color(0.05f, 0.05f, 0.05f);
    glPushMatrix();
    glTranslatef(0, 0.30f, 0);
    draw_box(1.6f, 0.45f, 0.85f);
    glPopMatrix();

    set_color(0.95f, 0.95f, 0.95f);
    glPushMatrix();
    glTranslatef(0, 0.35f, 0);
    draw_box(1.65f, 0.15f, 0.90f);
    glPopMatrix();

    set_color(0.10f, 0.10f, 0.15f);
    glPushMatrix();
    glTranslatef(-0.05f, 0.65f, 0);
    draw_box(0.9f, 0.35f, 0.75f);
    glPopMatrix();

    set_color(0.05f, 0.05f, 0.05f);
    for (int sx = -1; sx <= 1; sx += 2)
    for (int sz = -1; sz <= 1; sz += 2) {
        glPushMatrix();
        glTranslatef(sx * 0.55f, 0.12f, sz * 0.42f);
        glutSolidSphere(0.16f, 10, 10);
        glPopMatrix();
    }

    int blink = (t_ms / 180) & 1;
    glPushMatrix();
    glTranslatef(0, 1.05f, 0);
    if (blink) { set_color(1.0f, 0.1f, 0.1f); set_emissive(1.0f, 0.1f, 0.1f); }
    else       { set_color(0.1f, 0.3f, 1.0f); set_emissive(0.1f, 0.3f, 1.0f); }
    draw_box(0.60f, 0.12f, 0.60f);
    clear_emissive();
    glPopMatrix();
}

static void draw_on_ticket_path(direction_t d, float axis_pos)
{
    float spawn_dist = INTER_HALF + 1.5f;
    float dist_from_center = spawn_dist - axis_pos;

    switch (d) {
    case DIR_NORTH:
        if (dist_from_center > 0) glTranslatef(-1.3f, 0,  dist_from_center);
        else                      glTranslatef( 1.3f, 0,  dist_from_center);
        glRotatef(90, 0, 1, 0);
        break;
    case DIR_SOUTH:
        if (dist_from_center > 0) glTranslatef( 1.3f, 0, -dist_from_center);
        else                      glTranslatef(-1.3f, 0, -dist_from_center);
        glRotatef(-90, 0, 1, 0);
        break;
    case DIR_EAST:
        if (dist_from_center > 0) glTranslatef(-dist_from_center, 0, -1.3f);
        else                      glTranslatef(-dist_from_center, 0,  1.3f);
        break;
    case DIR_WEST:
        if (dist_from_center > 0) glTranslatef( dist_from_center, 0,  1.3f);
        else                      glTranslatef( dist_from_center, 0, -1.3f);
        glRotatef(180, 0, 1, 0);
        break;
    }
}

static void draw_ticket_chase(void)
{
    if (!ticket_local_active) return;
    int t_ms = (int)(t_animation * 1000);

    glPushMatrix();
    draw_on_ticket_path(ticket_local_dir, ticket_runner_pos);
    draw_car(0.95f, 0.85f, 0.10f);
    glPopMatrix();

    glPushMatrix();
    draw_on_ticket_path(ticket_local_dir, ticket_police_pos);
    draw_police_car(t_ms);
    glPopMatrix();
}

static void draw_ticket_modal(void)
{
    if (!ticket_local_active) return;

    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
    gluOrtho2D(0, WIN_W, 0, WIN_H);
    glMatrixMode(GL_MODELVIEW);   glPushMatrix(); glLoadIdentity();

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0, 0, 0, 0.30f);
    glBegin(GL_QUADS);
        glVertex2f(0,     0);
        glVertex2f(WIN_W, 0);
        glVertex2f(WIN_W, WIN_H);
        glVertex2f(0,     WIN_H);
    glEnd();

    float cw = 380, ch = 180;
    float cx = WIN_W - cw - 20;
    float cy = WIN_H - ch - 20;
    glColor4f(0.10f, 0.10f, 0.15f, 0.95f);
    glBegin(GL_QUADS);
        glVertex2f(cx,      cy);
        glVertex2f(cx + cw, cy);
        glVertex2f(cx + cw, cy + ch);
        glVertex2f(cx,      cy + ch);
    glEnd();
    glColor4f(0.95f, 0.20f, 0.20f, 1.0f);
    glLineWidth(3);
    glBegin(GL_LINE_LOOP);
        glVertex2f(cx,      cy);
        glVertex2f(cx + cw, cy);
        glVertex2f(cx + cw, cy + ch);
        glVertex2f(cx,      cy + ch);
    glEnd();
    glLineWidth(1);
    glDisable(GL_BLEND);

    glColor3f(1.0f, 0.30f, 0.30f);
    draw_text_big(cx + 15, cy + ch - 30, "*** TRAFFIC VIOLATION ***");
    glColor3f(0.95f, 0.95f, 0.95f);
    draw_text(cx + 15, cy + ch - 55, "Red-light violation detected.");

    char buf[128];
    snprintf(buf, sizeof(buf), "Plate:   %s", ticket_local_plate);
    glColor3f(1.0f, 0.95f, 0.40f);
    draw_text_big(cx + 15, cy + ch - 90, buf);

    snprintf(buf, sizeof(buf), "From:    %s", DIR_NAMES[ticket_local_dir]);
    glColor3f(0.95f, 0.95f, 0.95f);
    draw_text(cx + 15, cy + ch - 115, buf);

    glColor3f(0.80f, 0.85f, 1.0f);
    draw_text(cx + 15, cy + 20, "Police in pursuit...");

    glMatrixMode(GL_PROJECTION); glPopMatrix();
    glMatrixMode(GL_MODELVIEW);  glPopMatrix();
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
}

/* ---------- pedestrians --------------------------------------------- */

static void draw_pedestrian(float x, float z, float r, float g, float b)
{
    set_color(r, g, b);
    glPushMatrix();
    glTranslatef(x, 0.55f, z);
    draw_box(0.22f, 0.55f, 0.22f);
    glPopMatrix();
    set_color(0.95f, 0.82f, 0.68f);
    glPushMatrix();
    glTranslatef(x, 1.0f, z);
    glutSolidSphere(0.15f, 12, 12);
    glPopMatrix();
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

    int axis_active[NUM_DIRECTIONS] = {0, 0, 0, 0};
    if (!snap.emergency_active && snap.pedestrian_active) {
        int ns_green = (snap.light[DIR_NORTH] == LIGHT_GREEN) ||
                       (snap.light[DIR_SOUTH] == LIGHT_GREEN);
        int ew_green = (snap.light[DIR_EAST]  == LIGHT_GREEN) ||
                       (snap.light[DIR_WEST]  == LIGHT_GREEN);
        if (ns_green) { axis_active[DIR_EAST]  = 1; axis_active[DIR_WEST]  = 1; }
        if (ew_green) { axis_active[DIR_NORTH] = 1; axis_active[DIR_SOUTH] = 1; }
    }

    for (int i = 0; i < NUM_DIRECTIONS; ++i) {
        int cur_light   = snap.light[i];
        int has_pending = snap.pedestrian_pending[i] > 0;

        static int last_axis_active[NUM_DIRECTIONS] = {0, 0, 0, 0};
        if (axis_active[i] && !last_axis_active[i]) {
            int cnt = has_pending ? snap.pedestrian_pending[i] : 1;
            if (cnt > 8) cnt = 8;
            dps[i].walking      = 1;
            dps[i].frozen_count = cnt;
            dps[i].walk_start_t = t_animation;
        }
        last_axis_active[i] = axis_active[i];

        if (!axis_active[i]) {
            dps[i].walking = 0;
        }

        dps[i].last_light = cur_light;

        if (dps[i].walking) {
            float base_pos = (t_animation - dps[i].walk_start_t) * speed;
            float last_pos = base_pos - (dps[i].frozen_count - 1) * gap;
            if (last_pos >= road_end) {
                dps[i].walking = 0;
                if (axis_active[i]) {
                    int cnt = has_pending ? snap.pedestrian_pending[i] : 1;
                    if (cnt > 8) cnt = 8;
                    dps[i].walking      = 1;
                    dps[i].frozen_count = cnt;
                    dps[i].walk_start_t = t_animation;
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
    static phase_t last_phase = PHASE_STARTUP;
    if (snap.current_phase != last_phase) {
        char buf[128];
        snprintf(buf, sizeof(buf), "Phase: %s", PHASE_NAMES[snap.current_phase]);
        add_recent_event(buf);
        last_phase = snap.current_phase;
    }

    static int last_emerg = 0;
    if (snap.emergency_active && !last_emerg) {
        char buf[128];
        snprintf(buf, sizeof(buf), "EMERGENCY from %s", DIR_NAMES[snap.emergency_direction]);
        add_recent_event(buf);
    }
    last_emerg = snap.emergency_active;

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

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0, 0, 0, 0.55f);
    glBegin(GL_QUADS);
        glVertex2f(10,  WIN_H - 10);
        glVertex2f(360, WIN_H - 10);
        glVertex2f(360, WIN_H - 230);
        glVertex2f(10,  WIN_H - 230);
    glEnd();
    glDisable(GL_BLEND);

    char buf[256];
    glColor3f(0.95f, 0.95f, 0.95f);
    snprintf(buf, sizeof(buf), "Phase: %s", PHASE_NAMES[snap.current_phase]);
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
             "Served: %d veh   %d ped   |   Emerg: %d   Tickets: %d   Faults: %d",
             snap.total_vehicles_served, snap.total_pedestrians_served,
             snap.total_emergencies, snap.total_tickets, snap.safety_violations);
    draw_text(20, WIN_H - 200, buf);

    if (show_help) {
        glColor4f(0, 0, 0, 0.55f);
        glEnable(GL_BLEND);
        glBegin(GL_QUADS);
            glVertex2f(WIN_W - 320, 10);
            glVertex2f(WIN_W - 10,  10);
            glVertex2f(WIN_W - 10,  190);
            glVertex2f(WIN_W - 320, 190);
        glEnd();
        glDisable(GL_BLEND);
        glColor3f(1, 1, 1);
        draw_text(WIN_W - 310, 170, "Keyboard controls:");
        draw_text(WIN_W - 310, 150, "  n / s / e / w : add a vehicle");
        draw_text(WIN_W - 310, 130, "  N / S / E / W : pedestrian button");
        draw_text(WIN_W - 310, 110, "  1 / 2 / 3 / 4 : emergency from N/S/E/W");
        draw_text(WIN_W - 310,  90, "  t : spawn red-light runner (ticket)");
        draw_text(WIN_W - 310,  70, "  arrows : rotate camera");
        draw_text(WIN_W - 310,  50, "  +  /  - : zoom");
        draw_text(WIN_W - 310,  30, "  h : toggle help     q : quit");
    }

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0, 0, 0, 0.50f);
    glBegin(GL_QUADS);
        glVertex2f(10,  10);
        glVertex2f(360, 10);
        glVertex2f(360, 10 + MAX_RECENT_EVENTS * 18 + 10);
        glVertex2f(10,  10 + MAX_RECENT_EVENTS * 18 + 10);
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
        for (int d = 0; d < NUM_DIRECTIONS; d++) {
            for (int i = snap.waiting_vehicles[d]; i < MAX_VEHICLES_PER_DIR; i++) {
                vehicle_initialized[d][i] = 0;
            }
}
        sem_unlock(semid);
    }
}

/* ---------- display callback ---------------------------------------- */

static void on_display(void)
{
    take_snapshot();
    poll_recent_events();

    if (!ticket_local_active) {
        t_animation += 0.05f;
        /* Advance per-direction animation clocks each frame */
        update_car_anims();
    } else {
        /* Chase mode: advance runner + police, freeze world animation */
        ticket_runner_pos += ticket_runner_speed;
        ticket_police_pos += ticket_police_speed;

        float gap = ticket_runner_pos - ticket_police_pos;
        if (gap <= 1.0f) {
            evt_msg_t e;
            memset(&e, 0, sizeof(e));
            e.mtype     = MTYPE_EVT_TICKET_DONE;
            e.direction = ticket_local_dir;
            e.timestamp = time(NULL);
            msgsnd(qevt, &e, sizeof(e) - sizeof(long), IPC_NOWAIT);
            send_log(qlog, "GUI", 0, "TICKET: police caught plate %s",
                     ticket_local_plate);
            ticket_local_active = 0;
        }
    }

    glClearColor(0.55f, 0.75f, 0.9f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    float cx = cam_dist * sinf(cam_rotate * M_PI / 180.0f) *
               cosf(cam_angle  * M_PI / 180.0f);
    float cz = cam_dist * cosf(cam_rotate * M_PI / 180.0f) *
               cosf(cam_angle  * M_PI / 180.0f);
    float cy = cam_dist * sinf(cam_angle  * M_PI / 180.0f);
    gluLookAt(cx, cy, cz,  0, 0, 0,  0, 1, 0);

    draw_ground();
    draw_roads();

    draw_traffic_light( INTER_HALF + 1.5f,  INTER_HALF + 1.5f,
                        180.0f, snap.light[DIR_NORTH]);
    draw_traffic_light(-INTER_HALF - 1.5f, -INTER_HALF - 1.5f,
                          0.0f, snap.light[DIR_SOUTH]);
    draw_traffic_light(-INTER_HALF - 1.5f,  INTER_HALF + 1.5f,
                        -90.0f, snap.light[DIR_EAST]);
    draw_traffic_light( INTER_HALF + 1.5f, -INTER_HALF - 1.5f,
                         90.0f, snap.light[DIR_WEST]);

    for (int i = 0; i < NUM_DIRECTIONS; ++i)
        draw_vehicles_queue((direction_t)i, snap.waiting_vehicles[i]);

    draw_bike_lane_traffic();

    draw_emergency_vehicle();
    draw_pedestrians();

    /* Ticket chase drawn last so it appears on top */
    draw_ticket_chase();

    draw_hud();
    draw_ticket_modal();   /* always on top */

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
    case 't': case 'T': {
        if (ticket_local_active) break;

        direction_t candidates[NUM_DIRECTIONS];
        int n_candidates = 0;
        for (int i = 0; i < NUM_DIRECTIONS; ++i) {
            if (snap.waiting_vehicles[i] > 0 && snap.light[i] == LIGHT_RED)
                candidates[n_candidates++] = (direction_t)i;
        }
        if (n_candidates == 0) break;

        ticket_local_dir = candidates[rand() % n_candidates];
        gen_random_plate(ticket_local_plate, sizeof(ticket_local_plate));

        ticket_runner_pos   = ROAD_LENGTH - (INTER_HALF + 1.5f);
        ticket_police_pos   = ticket_runner_pos - 12.0f;
        ticket_local_active = 1;

        if (snap.waiting_vehicles[ticket_local_dir] > 0)
            snap.waiting_vehicles[ticket_local_dir]--;
        sem_lock(semid);
        if (S->waiting_vehicles[ticket_local_dir] > 0)
            S->waiting_vehicles[ticket_local_dir]--;
        sem_unlock(semid);

        ticket_police_speed = 0.22f;

        evt_msg_t e;
        memset(&e, 0, sizeof(e));
        e.mtype     = MTYPE_EVT_TICKET;
        e.direction = ticket_local_dir;
        e.timestamp = time(NULL);
        strncpy(e.info, ticket_local_plate, sizeof(e.info) - 1);
        msgsnd(qevt, &e, sizeof(e) - sizeof(long), IPC_NOWAIT);
        send_log(qlog, "GUI", 1,
                 "TICKET: red-light runner from %s (was queued at RED), plate %s",
                 DIR_NAMES[ticket_local_dir], ticket_local_plate);
        break;
    }
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

/* ---------- timer ---------------------------------------------------- */

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
    GLfloat lspec[] = { 0.5f, 0.5f, 0.5f,  1.0f };
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
    srand((unsigned)(time(NULL) ^ getpid()));
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(WIN_W, WIN_H);
    glutCreateWindow("ENCS4330 — Real-Time Traffic Light Control System");

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
/******************************************************************************
 * gui.c
 *
 * OpenGL / GLUT real-time visualisation of the intersection. Reads state
 * from shared memory; never writes a light directly (it can only post events
 * to the event queue – just like a real sensor or button). Renders:
 *   - 4-way intersection with marked lanes, stop bars, crosswalks
 *   - Per-lane traffic lights for THROUGH and LEFT lanes
 *   - Pedestrian WALK / DONT-WALK signals at each crosswalk
 *   - Smoothly moving cars (one slot per queued vehicle, animated through
 *     the intersection when their lane is green)
 *   - A dedicated emergency corridor strip down the middle of the active
 *     emergency direction so the ambulance/police can pass safely
 *   - HUD with current phase, countdown, statistics, and recent log
 *   - Keyboard controls to spawn events manually
 *****************************************************************************/
#include "common.h"
#include <GL/freeglut.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- Window ------------------------------------------------------------- */
static int   WIN_W = 1280, WIN_H = 800;
static int   shmid, evtid, semid;
static SharedState *S;

/* ---- Geometry ----------------------------------------------------------- */
/* Use a 100x100 world centred on (0,0). The intersection is at the origin. */
static const float ROAD_HALF_WIDTH = 14.0f;   /* half of total road width */
static const float LANE_WIDTH      = 3.5f;    /* one lane */
/* lanes per direction: 1 LEFT (inner) + 1 THROUGH (outer) per direction
 * of travel. We use 2 lanes per side -> ROAD_HALF = 2 * 3.5 = 7. The road
 * is 14 wide each side of the centreline -> 28 total. */

/* ---- Car animation state ----------------------------------------------- */
#define MAX_CARS_PER_LANE 8
typedef struct {
    int   active;
    float pos;          /* position along approach: 0..1 (1 = at stop line) */
    float speed;
    float r, g, b;      /* car colour */
    int   has_passed;   /* once it crosses the intersection */
    float exit_t;       /* parametric position once in/past the box */
} Car;
static Car cars[NUM_DIRS][NUM_LANES][MAX_CARS_PER_LANE];

/* Emergency vehicle moving along the corridor */
typedef struct {
    int   active;
    Direction dir;
    float t;            /* 0..1 along the corridor from spawn to exit */
} EmergencyVeh;
static EmergencyVeh g_emerg = {0};

/* Pedestrian animation */
typedef struct {
    int   active;
    Direction crosswalk;
    float t;            /* 0..1 across the road */
    int   forward;      /* 1: this side -> other; 0: reverse */
} Pedestrian;
#define MAX_PEDS 8
static Pedestrian peds[MAX_PEDS];

/* ---- Misc state for animation ----------------------------------------- */
static float g_yellow_blink = 0.0f;
static float g_anim_clock   = 0.0f;
static int   g_paused       = 0;

/* ---- Helpers ----------------------------------------------------------- */
static void post_event(long type, int direction, int lane)
{
    EventMsg e = {0};
    e.mtype = type;
    e.direction = direction;
    e.lane = lane;
    e.timestamp = time(NULL);
    msgsnd(evtid, &e, sizeof(e) - sizeof(long), IPC_NOWAIT);
}

static void color3(float r, float g, float b) { glColor3f(r, g, b); }

static void draw_quad(float x0, float y0, float x1, float y1)
{
    glBegin(GL_QUADS);
    glVertex2f(x0, y0); glVertex2f(x1, y0);
    glVertex2f(x1, y1); glVertex2f(x0, y1);
    glEnd();
}

static void draw_circle(float cx, float cy, float r, int seg)
{
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(cx, cy);
    for (int i = 0; i <= seg; i++) {
        float a = 2.0f * (float)M_PI * i / seg;
        glVertex2f(cx + r * cosf(a), cy + r * sinf(a));
    }
    glEnd();
}

static void draw_text(float x, float y, void *font, const char *text,
                      float r, float g, float bl)
{
    color3(r, g, bl);
    glRasterPos2f(x, y);
    for (const char *c = text; *c; c++) glutBitmapCharacter(font, *c);
}

/* ---- Road rendering ---------------------------------------------------- */
static void draw_grass_and_road(void)
{
    /* Grass background */
    color3(0.18f, 0.42f, 0.18f);
    draw_quad(-60, -60, 60, 60);

    /* Subtle texture: small darker patches */
    color3(0.14f, 0.36f, 0.14f);
    srand(42);
    for (int i = 0; i < 80; i++) {
        float x = ((rand() % 1200) / 10.0f) - 60.0f;
        float y = ((rand() % 1200) / 10.0f) - 60.0f;
        if (fabsf(x) < ROAD_HALF_WIDTH + 1 || fabsf(y) < ROAD_HALF_WIDTH + 1) continue;
        draw_circle(x, y, 1.2f, 8);
    }

    /* Asphalt: two crossing strips */
    color3(0.18f, 0.18f, 0.20f);
    draw_quad(-60, -ROAD_HALF_WIDTH, 60, ROAD_HALF_WIDTH);
    draw_quad(-ROAD_HALF_WIDTH, -60, ROAD_HALF_WIDTH, 60);

    /* Lane markings ----------------------------------------------------- */
    /* Centre double-yellow lines on each road */
    color3(0.95f, 0.78f, 0.18f);
    glLineWidth(2.0f);
    /* Horizontal road */
    glBegin(GL_LINES);
    glVertex2f(-60, -0.25f); glVertex2f(-ROAD_HALF_WIDTH, -0.25f);
    glVertex2f(-60,  0.25f); glVertex2f(-ROAD_HALF_WIDTH,  0.25f);
    glVertex2f( ROAD_HALF_WIDTH, -0.25f); glVertex2f(60, -0.25f);
    glVertex2f( ROAD_HALF_WIDTH,  0.25f); glVertex2f(60,  0.25f);
    /* Vertical road */
    glVertex2f(-0.25f, -60); glVertex2f(-0.25f, -ROAD_HALF_WIDTH);
    glVertex2f( 0.25f, -60); glVertex2f( 0.25f, -ROAD_HALF_WIDTH);
    glVertex2f(-0.25f,  ROAD_HALF_WIDTH); glVertex2f(-0.25f, 60);
    glVertex2f( 0.25f,  ROAD_HALF_WIDTH); glVertex2f( 0.25f, 60);
    glEnd();

    /* Dashed lane dividers between LEFT and THROUGH lanes in each direction. */
    color3(0.92f, 0.92f, 0.92f);
    glLineWidth(1.5f);
    glEnable(GL_LINE_STIPPLE);
    glLineStipple(2, 0x00FF);
    glBegin(GL_LINES);
    /* horizontal road: dividers at y = +/- LANE_WIDTH (between left & through
     * for each travel direction) */
    glVertex2f(-60,  LANE_WIDTH); glVertex2f(-ROAD_HALF_WIDTH,  LANE_WIDTH);
    glVertex2f( ROAD_HALF_WIDTH,  LANE_WIDTH); glVertex2f(60,  LANE_WIDTH);
    glVertex2f(-60, -LANE_WIDTH); glVertex2f(-ROAD_HALF_WIDTH, -LANE_WIDTH);
    glVertex2f( ROAD_HALF_WIDTH, -LANE_WIDTH); glVertex2f(60, -LANE_WIDTH);
    /* vertical road */
    glVertex2f( LANE_WIDTH, -60); glVertex2f( LANE_WIDTH, -ROAD_HALF_WIDTH);
    glVertex2f( LANE_WIDTH,  ROAD_HALF_WIDTH); glVertex2f( LANE_WIDTH, 60);
    glVertex2f(-LANE_WIDTH, -60); glVertex2f(-LANE_WIDTH, -ROAD_HALF_WIDTH);
    glVertex2f(-LANE_WIDTH,  ROAD_HALF_WIDTH); glVertex2f(-LANE_WIDTH, 60);
    glEnd();
    glDisable(GL_LINE_STIPPLE);

    /* Stop bars (white thick lines just before the crosswalk) */
    color3(1.0f, 1.0f, 1.0f);
    glLineWidth(4.0f);
    glBegin(GL_LINES);
    /* North approach (cars going south) – stop bar at y = +ROAD_HALF_WIDTH */
    glVertex2f(0, ROAD_HALF_WIDTH);  glVertex2f( ROAD_HALF_WIDTH * 0.5f, ROAD_HALF_WIDTH);
    /* South approach */
    glVertex2f(-ROAD_HALF_WIDTH * 0.5f, -ROAD_HALF_WIDTH); glVertex2f(0, -ROAD_HALF_WIDTH);
    /* East approach */
    glVertex2f(ROAD_HALF_WIDTH, -ROAD_HALF_WIDTH * 0.5f); glVertex2f(ROAD_HALF_WIDTH, 0);
    /* West approach */
    glVertex2f(-ROAD_HALF_WIDTH, 0); glVertex2f(-ROAD_HALF_WIDTH, ROAD_HALF_WIDTH * 0.5f);
    glEnd();
    glLineWidth(1.0f);

    /* Crosswalks: zebra stripes on each leg, just outside the box. */
    color3(0.95f, 0.95f, 0.95f);
    /* North crosswalk: y in [ROAD_HALF_WIDTH, ROAD_HALF_WIDTH+2.5], stripes vertical */
    for (int i = 0; i < 7; i++) {
        float x0 = -ROAD_HALF_WIDTH + 1 + i * 4.0f;
        draw_quad(x0, ROAD_HALF_WIDTH + 0.2f, x0 + 1.8f, ROAD_HALF_WIDTH + 2.7f);
    }
    /* South crosswalk */
    for (int i = 0; i < 7; i++) {
        float x0 = -ROAD_HALF_WIDTH + 1 + i * 4.0f;
        draw_quad(x0, -ROAD_HALF_WIDTH - 2.7f, x0 + 1.8f, -ROAD_HALF_WIDTH - 0.2f);
    }
    /* East crosswalk */
    for (int i = 0; i < 7; i++) {
        float y0 = -ROAD_HALF_WIDTH + 1 + i * 4.0f;
        draw_quad(ROAD_HALF_WIDTH + 0.2f, y0, ROAD_HALF_WIDTH + 2.7f, y0 + 1.8f);
    }
    /* West crosswalk */
    for (int i = 0; i < 7; i++) {
        float y0 = -ROAD_HALF_WIDTH + 1 + i * 4.0f;
        draw_quad(-ROAD_HALF_WIDTH - 2.7f, y0, -ROAD_HALF_WIDTH - 0.2f, y0 + 1.8f);
    }

    /* Re-cover the intersection box (so stop-bars and lane lines don't bleed
     * into the centre). */
    color3(0.20f, 0.20f, 0.22f);
    draw_quad(-ROAD_HALF_WIDTH, -ROAD_HALF_WIDTH,
               ROAD_HALF_WIDTH,  ROAD_HALF_WIDTH);
}

/* ---- Traffic-light gantry ---------------------------------------------- */
/* Each direction has TWO lights drawn near its stop bar: one for THROUGH
 * (with arrows up / right) and one for LEFT (with a left-turn arrow). */
static void draw_signal_head(float cx, float cy, LightState state, int blink,
                             const char *label, float labelx, float labely)
{
    /* Backplate */
    color3(0.10f, 0.10f, 0.12f);
    draw_quad(cx - 0.9f, cy - 2.6f, cx + 0.9f, cy + 2.6f);
    color3(0.04f, 0.04f, 0.06f);
    glLineWidth(1.0f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(cx - 0.9f, cy - 2.6f); glVertex2f(cx + 0.9f, cy - 2.6f);
    glVertex2f(cx + 0.9f, cy + 2.6f); glVertex2f(cx - 0.9f, cy + 2.6f);
    glEnd();

    /* Bulbs: red top, yellow middle, green bottom */
    float dim = 0.22f;
    float on  = 1.0f;
    int yellow_on = (state == LIGHT_YELLOW) && (!blink || ((int)(g_yellow_blink*2)&1));

    color3(state == LIGHT_RED    ? on  : dim, 0, 0);
    draw_circle(cx, cy + 1.7f, 0.55f, 18);
    color3(yellow_on ? on : dim, yellow_on ? on*0.85f : dim*0.85f, 0);
    draw_circle(cx, cy,        0.55f, 18);
    color3(0, state == LIGHT_GREEN  ? on  : dim, 0);
    draw_circle(cx, cy - 1.7f, 0.55f, 18);

    /* Label */
    draw_text(labelx, labely, GLUT_BITMAP_HELVETICA_10, label,
              1.0f, 1.0f, 1.0f);
}

static void draw_all_signals(void)
{
    sem_lock(semid);
    LightState L[NUM_DIRS][NUM_LANES];
    memcpy(L, S->light, sizeof(L));
    sem_unlock(semid);

    /* North approach: cars come from +y going down. Signals visible to
     * approaching cars are placed on the far side (at +y, on the right of
     * the approaching driver -> east side of north road). */
    /* North through (right & straight) at (ROAD_HALF_WIDTH+2, +ROAD_HALF_WIDTH+5) */
    draw_signal_head( ROAD_HALF_WIDTH + 2.0f, ROAD_HALF_WIDTH + 5.0f,
                      L[DIR_NORTH][LANE_THROUGH], 1,
                      "N-THRU", ROAD_HALF_WIDTH + 0.5f, ROAD_HALF_WIDTH + 8.0f);
    /* North left  */
    draw_signal_head( ROAD_HALF_WIDTH - 2.0f, ROAD_HALF_WIDTH + 5.0f,
                      L[DIR_NORTH][LANE_LEFT], 1,
                      "N-LEFT", ROAD_HALF_WIDTH - 4.0f, ROAD_HALF_WIDTH + 8.0f);

    /* South approach: cars from -y going up. Signals on +ROAD_HALF (south road) */
    draw_signal_head(-ROAD_HALF_WIDTH - 2.0f, -ROAD_HALF_WIDTH - 5.0f,
                      L[DIR_SOUTH][LANE_THROUGH], 1,
                      "S-THRU", -ROAD_HALF_WIDTH - 4.5f, -ROAD_HALF_WIDTH - 8.5f);
    draw_signal_head(-ROAD_HALF_WIDTH + 2.0f, -ROAD_HALF_WIDTH - 5.0f,
                      L[DIR_SOUTH][LANE_LEFT], 1,
                      "S-LEFT", -ROAD_HALF_WIDTH + 0.5f, -ROAD_HALF_WIDTH - 8.5f);

    /* East approach: cars from +x going left. Place signals on right side
     * (south of east road) */
    draw_signal_head( ROAD_HALF_WIDTH + 5.0f, -ROAD_HALF_WIDTH - 2.0f,
                      L[DIR_EAST][LANE_THROUGH], 1,
                      "E-THRU", ROAD_HALF_WIDTH + 3.5f, -ROAD_HALF_WIDTH - 4.5f);
    draw_signal_head( ROAD_HALF_WIDTH + 5.0f, -ROAD_HALF_WIDTH + 2.0f,
                      L[DIR_EAST][LANE_LEFT], 1,
                      "E-LEFT", ROAD_HALF_WIDTH + 3.5f, -ROAD_HALF_WIDTH + 0.5f);

    /* West approach */
    draw_signal_head(-ROAD_HALF_WIDTH - 5.0f,  ROAD_HALF_WIDTH + 2.0f,
                      L[DIR_WEST][LANE_THROUGH], 1,
                      "W-THRU", -ROAD_HALF_WIDTH - 8.5f, ROAD_HALF_WIDTH + 3.5f);
    draw_signal_head(-ROAD_HALF_WIDTH - 5.0f,  ROAD_HALF_WIDTH - 2.0f,
                      L[DIR_WEST][LANE_LEFT], 1,
                      "W-LEFT", -ROAD_HALF_WIDTH - 8.5f, ROAD_HALF_WIDTH - 0.5f);
}

/* ---- Pedestrian signals (small WALK / DONT-WALK boxes) ----------------- */
static void draw_ped_signal(float cx, float cy, PedState st)
{
    color3(0.05f, 0.05f, 0.07f);
    draw_quad(cx - 0.9f, cy - 0.9f, cx + 0.9f, cy + 0.9f);
    int show_walk = 0;
    if (st == PED_WALK) show_walk = 1;
    else if (st == PED_WALK_FLASHING) show_walk = ((int)(g_yellow_blink * 2) & 1);

    if (show_walk) {
        /* white walking figure */
        color3(1.0f, 1.0f, 1.0f);
        /* head */
        draw_circle(cx, cy + 0.3f, 0.18f, 12);
        /* body */
        glLineWidth(2.5f);
        glBegin(GL_LINES);
        glVertex2f(cx, cy + 0.1f); glVertex2f(cx, cy - 0.3f);
        /* legs */
        glVertex2f(cx, cy - 0.3f); glVertex2f(cx - 0.2f, cy - 0.6f);
        glVertex2f(cx, cy - 0.3f); glVertex2f(cx + 0.2f, cy - 0.6f);
        /* arms */
        glVertex2f(cx, cy - 0.05f); glVertex2f(cx + 0.25f, cy - 0.2f);
        glVertex2f(cx, cy - 0.05f); glVertex2f(cx - 0.25f, cy - 0.2f);
        glEnd();
        glLineWidth(1.0f);
    } else {
        /* orange hand */
        color3(1.0f, 0.45f, 0.05f);
        glBegin(GL_POLYGON);
        glVertex2f(cx - 0.25f, cy - 0.5f);
        glVertex2f(cx + 0.25f, cy - 0.5f);
        glVertex2f(cx + 0.35f, cy + 0.4f);
        glVertex2f(cx - 0.35f, cy + 0.4f);
        glEnd();
    }
}

static void draw_all_ped_signals(void)
{
    sem_lock(semid);
    PedState P[NUM_CROSSWALKS];
    memcpy(P, S->ped_signal, sizeof(P));
    sem_unlock(semid);

    draw_ped_signal( ROAD_HALF_WIDTH + 1.5f,  ROAD_HALF_WIDTH + 1.5f, P[DIR_NORTH]);
    draw_ped_signal(-ROAD_HALF_WIDTH - 1.5f, -ROAD_HALF_WIDTH - 1.5f, P[DIR_SOUTH]);
    draw_ped_signal( ROAD_HALF_WIDTH + 1.5f, -ROAD_HALF_WIDTH - 1.5f, P[DIR_EAST]);
    draw_ped_signal(-ROAD_HALF_WIDTH - 1.5f,  ROAD_HALF_WIDTH + 1.5f, P[DIR_WEST]);
}

/* ---- Car drawing ------------------------------------------------------- */
/* Lane offsets (perpendicular from centreline of road) for each (dir, lane).
 * Drivers drive on the right (US/EU style). So for north-bound (DIR_NORTH
 * means cars coming FROM the north), they travel in the -y direction, on the
 * right side of the y axis when facing south -> negative x.
 *
 * We define: the approach line for each direction, with two lane slots
 * (LEFT inner / THROUGH outer). */

typedef struct {
    float sx, sy;    /* spawn (far end of approach) */
    float qx, qy;    /* stop-line position for the FRONT car in this lane */
    float dx, dy;    /* unit vector from spawn toward stop line */
    float perp_x, perp_y; /* perpendicular (for sliding between cars) */
} LaneGeom;

static LaneGeom lane_geom(Direction d, Lane l)
{
    LaneGeom g;
    /* The right-hand side of the road (from the driver's POV) is the OUTER
     * lane (THROUGH); LEFT is the INNER lane (next to the centreline). */
    /* For DIR_NORTH (cars from north, moving south, -y):
     *   - through (outer) is at x = -1.75 - 3.5 = -5.25 (further from centre
     *     in -x because their right side is +x... wait, facing -y, right is -x)
     *   - Driver heading south, right side is +x? No: if heading -y, your
     *     right hand points to -x. So the right-hand outer lane is at -x.
     *   - Hmm, let's redo: standard right-hand traffic, driver going south
     *     stays on the WEST side of the road, i.e., x < 0.
     *     -> THROUGH lane is at x ≈ -LANE_WIDTH*1.5 (outer) and LEFT is at
     *        x ≈ -LANE_WIDTH*0.5 (next to centre).
     */
    switch (d) {
    case DIR_NORTH:
        g.sx = (l == LANE_THROUGH) ? -LANE_WIDTH * 1.5f : -LANE_WIDTH * 0.5f;
        g.sy = 55.0f;
        g.qx = g.sx;
        g.qy = ROAD_HALF_WIDTH + 3.5f; /* just before crosswalk */
        g.dx = 0; g.dy = -1;
        g.perp_x = 1; g.perp_y = 0;
        break;
    case DIR_SOUTH:
        /* Heading +y. Right hand -> +x. So south-bound (DIR_SOUTH) cars stay
         * on the east side: x > 0. */
        g.sx = (l == LANE_THROUGH) ?  LANE_WIDTH * 1.5f :  LANE_WIDTH * 0.5f;
        g.sy = -55.0f;
        g.qx = g.sx;
        g.qy = -ROAD_HALF_WIDTH - 3.5f;
        g.dx = 0; g.dy = 1;
        g.perp_x = 1; g.perp_y = 0;
        break;
    case DIR_EAST:
        /* From east, heading -x. Right hand -> -y. So they stay y<0. */
        g.sx = 55.0f;
        g.sy = (l == LANE_THROUGH) ? -LANE_WIDTH * 1.5f : -LANE_WIDTH * 0.5f;
        g.qx = ROAD_HALF_WIDTH + 3.5f;
        g.qy = g.sy;
        g.dx = -1; g.dy = 0;
        g.perp_x = 0; g.perp_y = 1;
        break;
    case DIR_WEST:
        /* From west, heading +x. Right hand -> +y. y>0. */
        g.sx = -55.0f;
        g.sy = (l == LANE_THROUGH) ?  LANE_WIDTH * 1.5f :  LANE_WIDTH * 0.5f;
        g.qx = -ROAD_HALF_WIDTH - 3.5f;
        g.qy = g.sy;
        g.dx = 1; g.dy = 0;
        g.perp_x = 0; g.perp_y = 1;
        break;
    default:
        memset(&g, 0, sizeof(g));
        break;
    }
    return g;
}

static void draw_car_box(float cx, float cy, float dx, float dy,
                         float r, float g, float b)
{
    /* car: 3.0 x 1.6 oriented along (dx,dy) */
    float L = 1.5f, W = 0.8f;
    float ex = -dy, ey = dx;
    float p1x = cx + dx*L + ex*W, p1y = cy + dy*L + ey*W;
    float p2x = cx + dx*L - ex*W, p2y = cy + dy*L - ey*W;
    float p3x = cx - dx*L - ex*W, p3y = cy - dy*L - ey*W;
    float p4x = cx - dx*L + ex*W, p4y = cy - dy*L + ey*W;
    color3(r, g, b);
    glBegin(GL_QUADS);
    glVertex2f(p1x, p1y); glVertex2f(p2x, p2y);
    glVertex2f(p3x, p3y); glVertex2f(p4x, p4y);
    glEnd();
    /* windshield (slightly lighter) */
    color3(r * 0.4f + 0.3f, g * 0.4f + 0.3f, b * 0.4f + 0.4f);
    float fx = cx + dx*L*0.4f, fy = cy + dy*L*0.4f;
    float bx = cx - dx*L*0.1f, by = cy - dy*L*0.1f;
    glBegin(GL_QUADS);
    glVertex2f(fx + ex*W*0.8f, fy + ey*W*0.8f);
    glVertex2f(fx - ex*W*0.8f, fy - ey*W*0.8f);
    glVertex2f(bx - ex*W*0.8f, by - ey*W*0.8f);
    glVertex2f(bx + ex*W*0.8f, by + ey*W*0.8f);
    glEnd();
    /* outline */
    color3(0.05f, 0.05f, 0.05f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(p1x, p1y); glVertex2f(p2x, p2y);
    glVertex2f(p3x, p3y); glVertex2f(p4x, p4y);
    glEnd();
}

static void update_and_draw_cars(float dt)
{
    sem_lock(semid);
    int queue_count[NUM_DIRS][NUM_LANES];
    LightState lights[NUM_DIRS][NUM_LANES];
    for (int d = 0; d < NUM_DIRS; d++)
        for (int l = 0; l < NUM_LANES; l++) {
            queue_count[d][l] = S->queue[d][l].count;
            lights[d][l]      = S->light[d][l];
        }
    sem_unlock(semid);

    for (int d = 0; d < NUM_DIRS; d++) {
        for (int l = 0; l < NUM_LANES; l++) {
            LaneGeom lg = lane_geom((Direction)d, (Lane)l);
            int want = queue_count[d][l];
            if (want > MAX_CARS_PER_LANE) want = MAX_CARS_PER_LANE;

            /* Ensure number of active cars matches the queue length. We add
             * new cars at the spawn point and remove cars that have driven
             * past the box. */
            int active = 0;
            for (int i = 0; i < MAX_CARS_PER_LANE; i++)
                if (cars[d][l][i].active && !cars[d][l][i].has_passed) active++;

            while (active < want) {
                /* spawn one car in a free slot */
                for (int i = 0; i < MAX_CARS_PER_LANE; i++) {
                    if (!cars[d][l][i].active) {
                        cars[d][l][i].active = 1;
                        cars[d][l][i].pos = 0.0f;
                        cars[d][l][i].speed = 0.05f + (rand() % 30) * 0.001f;
                        /* random car colour */
                        int palette = rand() % 6;
                        float pr[6] = {0.85f, 0.20f, 0.95f, 0.10f, 0.85f, 0.55f};
                        float pg[6] = {0.20f, 0.55f, 0.85f, 0.50f, 0.85f, 0.55f};
                        float pb[6] = {0.20f, 0.85f, 0.20f, 0.85f, 0.20f, 0.55f};
                        cars[d][l][i].r = pr[palette];
                        cars[d][l][i].g = pg[palette];
                        cars[d][l][i].b = pb[palette];
                        cars[d][l][i].has_passed = 0;
                        cars[d][l][i].exit_t = 0.0f;
                        active++;
                        break;
                    }
                }
            }

            /* Update positions: cars move from spawn toward queue line. When
             * the front car reaches the stop bar:
             *   - if light is GREEN, it crosses the box and exits
             *   - else it waits, and cars behind queue up. */

            /* Sort by pos descending so the front car is first. We'll just
             * find front by max pos. */
            int order[MAX_CARS_PER_LANE];
            int n = 0;
            for (int i = 0; i < MAX_CARS_PER_LANE; i++)
                if (cars[d][l][i].active && !cars[d][l][i].has_passed)
                    order[n++] = i;
            /* simple insertion sort by pos descending */
            for (int a = 1; a < n; a++) {
                int key = order[a]; int b = a - 1;
                while (b >= 0 && cars[d][l][order[b]].pos < cars[d][l][key].pos) {
                    order[b+1] = order[b]; b--;
                }
                order[b+1] = key;
            }

            /* Each car has a "target pos" based on its rank in queue:
             *   front car (rank 0) target = 1.0 (at stop bar)
             *   rank k        target = 1.0 - k * spacing
             * where spacing is enough room for one car body. */
            float spacing = 0.07f;   /* in normalised approach length */
            for (int k = 0; k < n; k++) {
                Car *c = &cars[d][l][order[k]];
                float target = 1.0f - k * spacing;
                if (target < 0) target = 0;

                /* If at the stop bar (front) and light is green, leap to
                 * exit and beyond. */
                int is_front = (k == 0);
                if (is_front && c->pos >= 0.99f && lights[d][l] == LIGHT_GREEN) {
                    c->has_passed = 1;
                    c->exit_t = 0.0f;
                    continue;
                }
                if (c->pos < target) {
                    c->pos += c->speed * dt;
                    if (c->pos > target) c->pos = target;
                }
                if (is_front && lights[d][l] == LIGHT_GREEN && c->pos >= 0.99f) {
                    c->has_passed = 1;
                    c->exit_t = 0.0f;
                }
            }

            /* Animate has_passed cars across and out of intersection */
            for (int i = 0; i < MAX_CARS_PER_LANE; i++) {
                Car *c = &cars[d][l][i];
                if (c->active && c->has_passed) {
                    c->exit_t += c->speed * dt * 1.5f;
                    if (c->exit_t > 1.5f) {
                        c->active = 0;
                    }
                }
            }

            /* Draw */
            for (int i = 0; i < MAX_CARS_PER_LANE; i++) {
                Car *c = &cars[d][l][i];
                if (!c->active) continue;
                float cx, cy;
                if (!c->has_passed) {
                    /* interpolate between spawn and stop bar by pos */
                    cx = lg.sx + (lg.qx - lg.sx) * c->pos;
                    cy = lg.sy + (lg.qy - lg.sy) * c->pos;
                } else {
                    /* continue past stop bar in same direction */
                    cx = lg.qx + lg.dx * c->exit_t * 50.0f;
                    cy = lg.qy + lg.dy * c->exit_t * 50.0f;
                }
                draw_car_box(cx, cy, lg.dx, lg.dy, c->r, c->g, c->b);
            }
        }
    }
}

/* ---- Emergency corridor & vehicle -------------------------------------- */
static void update_and_draw_emergency(float dt)
{
    sem_lock(semid);
    int active = S->emergency_active;
    Direction edir = S->emergency_direction;
    sem_unlock(semid);

    if (active && !g_emerg.active) {
        g_emerg.active = 1;
        g_emerg.dir = edir;
        g_emerg.t = 0.0f;
    } else if (!active && g_emerg.active && g_emerg.t > 1.2f) {
        g_emerg.active = 0;
    }

    if (!g_emerg.active) return;

    /* Draw the corridor: a faint yellow-orange highlighted strip along the
     * emergency direction's approach, marking the "small line" the user
     * requested for the emergency vehicle to pass. */
    LaneGeom lg = lane_geom(g_emerg.dir, LANE_THROUGH);
    /* Strip from spawn to the far exit side */
    float ax = lg.sx, ay = lg.sy;
    float bx = -lg.dx * 60.0f, by = -lg.dy * 60.0f;

    /* draw a translucent overlay */
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(1.0f, 0.55f, 0.10f, 0.22f);
    /* corridor width = 1 lane */
    float w = LANE_WIDTH * 0.5f;
    float ex = -lg.dy * w, ey = lg.dx * w;
    glBegin(GL_QUADS);
    glVertex2f(ax + ex, ay + ey);
    glVertex2f(ax - ex, ay - ey);
    glVertex2f(bx - ex, by - ey);
    glVertex2f(bx + ex, by + ey);
    glEnd();
    glDisable(GL_BLEND);

    /* Advance vehicle */
    g_emerg.t += 0.10f * dt;
    if (g_emerg.t > 1.5f) {
        g_emerg.active = 0;
        return;
    }
    /* Position interpolates from spawn through the intersection */
    float cx = ax + (bx - ax) * (g_emerg.t / 1.5f);
    float cy = ay + (by - ay) * (g_emerg.t / 1.5f);

    /* Body of ambulance: white with red cross */
    color3(1.0f, 1.0f, 1.0f);
    float L = 2.0f, W = 0.9f;
    float dx = lg.dx, dy = lg.dy;
    float ex2 = -dy, ey2 = dx;
    glBegin(GL_QUADS);
    glVertex2f(cx + dx*L + ex2*W, cy + dy*L + ey2*W);
    glVertex2f(cx + dx*L - ex2*W, cy + dy*L - ey2*W);
    glVertex2f(cx - dx*L - ex2*W, cy - dy*L - ey2*W);
    glVertex2f(cx - dx*L + ex2*W, cy - dy*L + ey2*W);
    glEnd();
    /* red cross */
    color3(0.95f, 0.10f, 0.10f);
    draw_quad(cx - 0.5f, cy - 0.15f, cx + 0.5f, cy + 0.15f);
    draw_quad(cx - 0.15f, cy - 0.5f, cx + 0.15f, cy + 0.5f);
    /* flashing lights (blue/red strobes) */
    int blink = ((int)(g_anim_clock * 6)) & 1;
    color3(blink ? 1.0f : 0.0f, 0.0f, blink ? 0.0f : 1.0f);
    draw_circle(cx + dx*L*0.6f + ex2*W*0.6f, cy + dy*L*0.6f + ey2*W*0.6f,
                0.25f, 12);
    color3(blink ? 0.0f : 1.0f, 0.0f, blink ? 1.0f : 0.0f);
    draw_circle(cx + dx*L*0.6f - ex2*W*0.6f, cy + dy*L*0.6f - ey2*W*0.6f,
                0.25f, 12);
    /* outline */
    color3(0,0,0);
    glBegin(GL_LINE_LOOP);
    glVertex2f(cx + dx*L + ex2*W, cy + dy*L + ey2*W);
    glVertex2f(cx + dx*L - ex2*W, cy + dy*L - ey2*W);
    glVertex2f(cx - dx*L - ex2*W, cy - dy*L - ey2*W);
    glVertex2f(cx - dx*L + ex2*W, cy - dy*L + ey2*W);
    glEnd();
}

/* ---- Pedestrian animation ---------------------------------------------- */
static void spawn_ped_if_walk(Direction d, PedState st)
{
    if (st != PED_WALK) return;
    /* Spawn a new pedestrian occasionally */
    for (int i = 0; i < MAX_PEDS; i++) {
        if (peds[i].active && peds[i].crosswalk == d) return; /* already one */
    }
    if (rand() % 100 > 40) return;
    for (int i = 0; i < MAX_PEDS; i++) {
        if (!peds[i].active) {
            peds[i].active = 1;
            peds[i].crosswalk = d;
            peds[i].t = 0.0f;
            peds[i].forward = rand() & 1;
            return;
        }
    }
}

static void update_and_draw_peds(float dt)
{
    sem_lock(semid);
    PedState P[NUM_CROSSWALKS];
    memcpy(P, S->ped_signal, sizeof(P));
    sem_unlock(semid);

    for (int d = 0; d < NUM_DIRS; d++) spawn_ped_if_walk((Direction)d, P[d]);

    for (int i = 0; i < MAX_PEDS; i++) {
        if (!peds[i].active) continue;
        peds[i].t += 0.20f * dt;
        if (peds[i].t > 1.0f) { peds[i].active = 0; continue; }

        /* The N crosswalk crosses the EW road; pedestrians walk along the y
         * axis at x = 0 (over the N approach crosswalk strip). Their start
         * and end depend on which side they start. */
        float x, y;
        switch (peds[i].crosswalk) {
        case DIR_NORTH:
            /* crosswalk at top: y ≈ ROAD_HALF + 1.5; walk across road in x */
            if (peds[i].forward) {
                x = -ROAD_HALF_WIDTH + peds[i].t * (2 * ROAD_HALF_WIDTH);
            } else {
                x =  ROAD_HALF_WIDTH - peds[i].t * (2 * ROAD_HALF_WIDTH);
            }
            y = ROAD_HALF_WIDTH + 1.4f;
            break;
        case DIR_SOUTH:
            if (peds[i].forward) {
                x =  ROAD_HALF_WIDTH - peds[i].t * (2 * ROAD_HALF_WIDTH);
            } else {
                x = -ROAD_HALF_WIDTH + peds[i].t * (2 * ROAD_HALF_WIDTH);
            }
            y = -ROAD_HALF_WIDTH - 1.4f;
            break;
        case DIR_EAST:
            if (peds[i].forward) {
                y = -ROAD_HALF_WIDTH + peds[i].t * (2 * ROAD_HALF_WIDTH);
            } else {
                y =  ROAD_HALF_WIDTH - peds[i].t * (2 * ROAD_HALF_WIDTH);
            }
            x = ROAD_HALF_WIDTH + 1.4f;
            break;
        case DIR_WEST:
            if (peds[i].forward) {
                y =  ROAD_HALF_WIDTH - peds[i].t * (2 * ROAD_HALF_WIDTH);
            } else {
                y = -ROAD_HALF_WIDTH + peds[i].t * (2 * ROAD_HALF_WIDTH);
            }
            x = -ROAD_HALF_WIDTH - 1.4f;
            break;
        default: continue;
        }

        /* Draw pedestrian as small figure */
        color3(0.95f, 0.85f, 0.55f);
        draw_circle(x, y + 0.4f, 0.18f, 12);
        color3(0.20f, 0.30f, 0.85f);
        draw_quad(x - 0.18f, y - 0.4f, x + 0.18f, y + 0.2f);
        color3(0.25f, 0.18f, 0.10f);
        draw_quad(x - 0.18f, y - 0.7f, x - 0.02f, y - 0.4f);
        draw_quad(x + 0.02f, y - 0.7f, x + 0.18f, y - 0.4f);
    }
}

/* ---- HUD --------------------------------------------------------------- */
static void draw_hud(void)
{
    /* Switch to overlay coordinates (pixels). */
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, WIN_W, 0, WIN_H);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    /* Right side panel background */
    int pw = 360;
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0.06f, 0.07f, 0.10f, 0.88f);
    glBegin(GL_QUADS);
    glVertex2i(WIN_W - pw, 0); glVertex2i(WIN_W, 0);
    glVertex2i(WIN_W, WIN_H);   glVertex2i(WIN_W - pw, WIN_H);
    glEnd();
    glDisable(GL_BLEND);

    /* Border */
    glColor3f(0.30f, 0.55f, 0.85f);
    glLineWidth(1.5f);
    glBegin(GL_LINE_LOOP);
    glVertex2i(WIN_W - pw + 1, 1);
    glVertex2i(WIN_W - 1,      1);
    glVertex2i(WIN_W - 1,      WIN_H - 1);
    glVertex2i(WIN_W - pw + 1, WIN_H - 1);
    glEnd();

    char buf[256];
    int x = WIN_W - pw + 14;
    int y = WIN_H - 26;

    /* Title */
    glColor3f(0.95f, 0.95f, 0.95f);
    glRasterPos2i(x, y);
    const char *title = "TRAFFIC CONTROL  -  BIRZEIT ENCS4330";
    for (const char *c = title; *c; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
    y -= 14;
    glColor3f(0.55f, 0.65f, 0.80f);
    glRasterPos2i(x, y);
    const char *sub = "Project #2 - IPC Real-Time System";
    for (const char *c = sub; *c; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, *c);
    y -= 24;

    /* Phase */
    sem_lock(semid);
    TrafficPhase ph = S->current_phase;
    PhaseSubState sb = S->phase_sub;
    int rem = S->phase_remaining_sec;
    int emerg = S->emergency_active;
    Direction edir = S->emergency_direction;
    unsigned long sv = S->total_vehicles_served;
    unsigned long sp = S->total_pedestrians_served;
    unsigned long se = S->total_emergencies_handled;
    unsigned long viol = S->safety_violations;
    int q[NUM_DIRS][NUM_LANES];
    memcpy(q, S->queue, sizeof(q));
    int peds_p[NUM_CROSSWALKS];
    for (int i = 0; i < NUM_CROSSWALKS; i++)
        peds_p[i] = S->ped_request_pending[i];
    sem_unlock(semid);

    glColor3f(0.95f, 0.90f, 0.55f);
    glRasterPos2i(x, y);
    snprintf(buf, sizeof(buf), "PHASE: %s [%s]  T-%ds",
             phase_name(ph),
             sb == SUB_GREEN  ? "GREEN" :
             sb == SUB_YELLOW ? "YELLOW" : "RED",
             rem > 0 ? rem : 0);
    for (const char *c = buf; *c; c++) glutBitmapCharacter(GLUT_BITMAP_9_BY_15, *c);
    y -= 20;

    if (emerg) {
        glColor3f(1.0f, 0.35f, 0.20f);
        glRasterPos2i(x, y);
        snprintf(buf, sizeof(buf), "!! EMERGENCY ACTIVE: %s !!", dir_name(edir));
        for (const char *c = buf; *c; c++) glutBitmapCharacter(GLUT_BITMAP_9_BY_15, *c);
        y -= 18;
    }
    y -= 6;

    /* Queues per direction */
    glColor3f(0.85f, 0.85f, 0.90f);
    glRasterPos2i(x, y);
    const char *qh = "Queues (THRU + LEFT):";
    for (const char *c = qh; *c; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
    y -= 16;
    for (int d = 0; d < NUM_DIRS; d++) {
        glRasterPos2i(x + 8, y);
        snprintf(buf, sizeof(buf), "%-6s  %2d  +  %2d   ped:%s",
                 dir_name(d), q[d][LANE_THROUGH], q[d][LANE_LEFT],
                 peds_p[d] ? "WAIT" : "-");
        for (const char *c = buf; *c; c++) glutBitmapCharacter(GLUT_BITMAP_8_BY_13, *c);
        y -= 14;
    }
    y -= 10;

    /* Stats */
    glColor3f(0.65f, 0.95f, 0.75f);
    glRasterPos2i(x, y);
    snprintf(buf, sizeof(buf), "Served vehicles:  %lu", sv);
    for (const char *c = buf; *c; c++) glutBitmapCharacter(GLUT_BITMAP_8_BY_13, *c);
    y -= 14;
    glRasterPos2i(x, y);
    snprintf(buf, sizeof(buf), "Served peds:      %lu", sp);
    for (const char *c = buf; *c; c++) glutBitmapCharacter(GLUT_BITMAP_8_BY_13, *c);
    y -= 14;
    glRasterPos2i(x, y);
    snprintf(buf, sizeof(buf), "Emergencies:      %lu", se);
    for (const char *c = buf; *c; c++) glutBitmapCharacter(GLUT_BITMAP_8_BY_13, *c);
    y -= 14;
    glColor3f(1.0f, 0.45f, 0.45f);
    glRasterPos2i(x, y);
    snprintf(buf, sizeof(buf), "Safety violations: %lu", viol);
    for (const char *c = buf; *c; c++) glutBitmapCharacter(GLUT_BITMAP_8_BY_13, *c);
    y -= 22;

    /* Controls */
    glColor3f(0.85f, 0.85f, 0.90f);
    glRasterPos2i(x, y);
    const char *ch = "CONTROLS:";
    for (const char *c = ch; *c; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
    y -= 16;
    const char *keys[] = {
        " N/S/E/W: vehicle arrival (THRU)",
        " Shift+N/S/E/W: vehicle (LEFT)",
        " 1/2/3/4: pedestrian @ N/S/E/W",
        " F1..F4: emergency from N/S/E/W",
        " SPACE: pause animation",
        " Q / ESC: quit cleanly",
        NULL
    };
    glColor3f(0.75f, 0.78f, 0.85f);
    for (int i = 0; keys[i]; i++) {
        glRasterPos2i(x + 4, y);
        for (const char *c = keys[i]; *c; c++)
            glutBitmapCharacter(GLUT_BITMAP_8_BY_13, *c);
        y -= 13;
    }
    y -= 10;

    /* Recent log */
    glColor3f(0.85f, 0.85f, 0.90f);
    glRasterPos2i(x, y);
    const char *lh = "RECENT EVENTS:";
    for (const char *c = lh; *c; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
    y -= 14;

    sem_lock(semid);
    int count = S->log_count;
    int head  = S->log_head;
    char lines[MAX_LOG_LINES][LOG_LINE_LEN];
    memcpy(lines, S->log_buffer, sizeof(lines));
    sem_unlock(semid);

    int show = count < 18 ? count : 18;
    glColor3f(0.75f, 0.82f, 0.95f);
    for (int i = 0; i < show && y > 20; i++) {
        int idx = (head - 1 - i + MAX_LOG_LINES) % MAX_LOG_LINES;
        glRasterPos2i(x + 2, y);
        for (const char *c = lines[idx]; *c; c++)
            glutBitmapCharacter(GLUT_BITMAP_8_BY_13, *c);
        y -= 12;
    }

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

/* ---- Display & timer --------------------------------------------------- */
static void display(void)
{
    glClearColor(0.05f, 0.07f, 0.10f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    /* World viewport: left two-thirds of window */
    int viewport_w = WIN_W - 360;
    glViewport(0, 0, viewport_w, WIN_H);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspect = (float)viewport_w / (float)WIN_H;
    float scale = 35.0f;
    gluOrtho2D(-scale * aspect, scale * aspect, -scale, scale);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    draw_grass_and_road();
    draw_all_signals();
    draw_all_ped_signals();

    float dt = g_paused ? 0.0f : 1.0f;
    g_anim_clock  += dt * 0.05f;
    g_yellow_blink += dt * 0.05f;

    update_and_draw_cars(dt);
    update_and_draw_peds(dt);
    update_and_draw_emergency(dt);

    /* Full-window viewport again for HUD */
    glViewport(0, 0, WIN_W, WIN_H);
    draw_hud();

    glutSwapBuffers();
}

static void timer_cb(int v)
{
    UNUSED(v);
    glutPostRedisplay();
    glutTimerFunc(33, timer_cb, 0); /* ~30 fps */
}

static void reshape(int w, int h)
{
    WIN_W = w; WIN_H = h;
}

/* ---- Keyboard ---------------------------------------------------------- */
static void on_quit(void)
{
    EventMsg e = {0};
    e.mtype = EVT_SHUTDOWN;
    e.timestamp = time(NULL);
    msgsnd(evtid, &e, sizeof(e) - sizeof(long), IPC_NOWAIT);
    /* Also flip system_running flag so processes notice quickly */
    if (S) {
        sem_lock(semid);
        S->system_running = 0;
        sem_unlock(semid);
    }
}

static void keyboard(unsigned char key, int x, int y)
{
    UNUSED(x); UNUSED(y);
    Direction d = -1;
    int shift = glutGetModifiers() & GLUT_ACTIVE_SHIFT;

    switch (key) {
    case 'n': d = DIR_NORTH; break;
    case 's': d = DIR_SOUTH; break;
    case 'e': d = DIR_EAST;  break;
    case 'w': d = DIR_WEST;  break;
    case 'N': d = DIR_NORTH; break;
    case 'S': d = DIR_SOUTH; break;
    case 'E': d = DIR_EAST;  break;
    case 'W': d = DIR_WEST;  break;
    case '1': post_event(EVT_PED_REQUEST, DIR_NORTH, 0); return;
    case '2': post_event(EVT_PED_REQUEST, DIR_SOUTH, 0); return;
    case '3': post_event(EVT_PED_REQUEST, DIR_EAST,  0); return;
    case '4': post_event(EVT_PED_REQUEST, DIR_WEST,  0); return;
    case ' ': g_paused = !g_paused; return;
    case 'q':
    case 27:
        on_quit();
        glutLeaveMainLoop();
        return;
    default: return;
    }
    if (d >= 0) {
        Lane l = shift ? LANE_LEFT : LANE_THROUGH;
        post_event(EVT_VEHICLE_ARRIVE, d, l);
    }
}

static void special(int key, int x, int y)
{
    UNUSED(x); UNUSED(y);
    switch (key) {
    case GLUT_KEY_F1: post_event(EVT_EMERGENCY, DIR_NORTH, 0); break;
    case GLUT_KEY_F2: post_event(EVT_EMERGENCY, DIR_SOUTH, 0); break;
    case GLUT_KEY_F3: post_event(EVT_EMERGENCY, DIR_EAST,  0); break;
    case GLUT_KEY_F4: post_event(EVT_EMERGENCY, DIR_WEST,  0); break;
    }
}

int main(int argc, char **argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(WIN_W, WIN_H);
    glutCreateWindow("Real-Time Traffic Control - Birzeit ENCS4330");

    shmid = get_shm(0);
    evtid = get_evt_queue(0);
    semid = get_sem(0);
    if (shmid < 0 || evtid < 0 || semid < 0) {
        fprintf(stderr, "GUI: IPC attach failed (is main running?)\n");
        return EXIT_FAILURE;
    }
    S = (SharedState *)shmat(shmid, NULL, 0);
    if (S == (void *)-1) { perror("shmat"); return EXIT_FAILURE; }

    memset(cars, 0, sizeof(cars));
    memset(peds, 0, sizeof(peds));
    srand((unsigned)time(NULL));

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(special);
    glutTimerFunc(33, timer_cb, 0);

    log_event(S, semid, "GUI started (PID %d)", getpid());

    glutMainLoop();

    log_event(S, semid, "GUI exiting");
    shmdt(S);
    return EXIT_SUCCESS;
}

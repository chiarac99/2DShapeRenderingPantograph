
// ============================================================
// ME327 Team 10 - Pantograph Single-Board Firmware
// ============================================================
// ARCHITECTURE: only U1 (right Hapkit) runs this firmware.
// U1 reads BOTH MR sensors via analog pins (its own on A2, the
// LEFT board's MR sensor jumper-wired into A3), and drives BOTH
// motors (right motor on M1 channel, left motor on M2 channel).
//
// U2 (left // ============================================================
// ME327 Team 10 - Pantograph Single-Board Firmware
// ============================================================
// ARCHITECTURE: only U1 (right Hapkit) runs this firmware.
// U1 reads BOTH MR sensors via analog pins (its own on A2, the
// LEFT board's MR sensor jumper-wired into A3), and drives BOTH
// motors (right motor on M1 channel, left motor on M2 channel).
//
// U2 (left Hapkit) just provides 5V/GND to its own MR sensor and
// jumpers the sensor's analog output to U1's A3. U2 runs whatever
// firmware it has — doesn't matter, no inter-board communication.
//
// This eliminates SoftwareSerial entirely. Both encoders are read
// synchronously every loop on the same MCU.
//
// Pin/channel naming convention (be careful):
//   "M1" / "M5"  — our math convention (theta1 = LEFT, theta5 = RIGHT)
//   "Motor 1" / "Motor 2" — Hapkit board's two H-bridge channels
// We deliberately route them as:
//   LEFT  motor (= our M1) → Hapkit's "Motor 2" channel (D6 PWM, D7 DIR)
//   RIGHT motor (= our M5) → Hapkit's "Motor 1" channel (D5 PWM, D8 DIR)
// ============================================================

#include <math.h>
#include <avr/pgmspace.h>

struct EncoderState {
  int  rawPos, lastRawPos, lastLastRawPos;
  int  rawDiff, lastRawDiff;
  int  rawOffset, lastRawOffset;
  int  flipNumber, tempOffset;
  bool flipped;
  int  updatedPos;
};

// ============================================================
// GLOBAL VARIABLES
// ============================================================

float proxy_x = 0.0f;
float proxy_y = 0.0f;
float Fx_filt = 0.0f;
float Fy_filt = 0.0f;
int   proxy_seg_idx = 0;
float proxy_seg_t   = 0.0f;

bool was_inside_shape = false;

// ============================================================
// LINK LENGTHS [meters] — unchanged from previous firmware
// ============================================================
const float LINK_A1 = 0.10f;
const float LINK_A2 = 0.10f;
const float LINK_A3 = 0.10f;
const float LINK_A4 = 0.10f;
const float LINK_A5 = 0.00f;  // motor pivots are coincident

// ============================================================
// PIN ASSIGNMENTS
// ============================================================
// Sensors
const int sensorPos_M1 = A3;   // LEFT MR sensor (jumper from U2's A2 → U1's A3)
const int sensorPos_M5 = A2;   // RIGHT MR sensor (this board's own A2)

// Motors — note the Hapkit-channel vs our-naming mismatch documented above
const int PWM_PIN_M5   = 5;    // RIGHT motor PWM, Hapkit "Motor 1" channel
const int DIR_PIN_M5   = 8;    // RIGHT motor DIR, Hapkit "Motor 1" channel
const int PWM_PIN_M1   = 6;    // LEFT motor PWM,  Hapkit "Motor 2" channel
const int DIR_PIN_M1   = 7;    // LEFT motor DIR,  Hapkit "Motor 2" channel

// ============================================================
// ENCODER CALIBRATION (per-sensor)
// ============================================================
// Same values as in the old two-board firmware.
// M1 (LEFT):  theta=0°  at updatedPos=1,    theta=90°  at updatedPos=-4058
// M5 (RIGHT): theta=90° at updatedPos=258,  theta=180° at updatedPos=-3708
const float ENC_M_M1 = -0.0222f;
const float ENC_B_M1 =  0.0f;
const float ENC_M_M5 = -0.0227f;
const float ENC_B_M5 =  95.5f;

const float SECTOR_GEAR_REDUCTION = 1.0f;

// ============================================================
// VELOCITY TRACKING 
// ============================================================
float xh_prev = 0.0f, yh_prev = 0.0f;
float vx_filt = 0.0f, vy_filt = 0.0f;
const float DT_LOOP = 0.001f;

// ============================================================
// MOTOR TEST MODE (legacy, leave disabled)
// ============================================================
const bool MOTOR_TEST_ENABLED      = false;
const int  MOTOR_TEST_PWM          = 100;
const unsigned long MOTOR_TEST_PERIOD_MS = 64000;   // ~1s real time (millis() is 64x-fast)
const unsigned long MOTOR_TEST_ON_MS     = 6400;    // ~100ms real time
const int  MOTOR_TEST_MAX_FLIPS    = 4;
bool motor_killed = false;
bool motor_test_started = false;
int  motor_flip_count = 0;
// Kill switch: any USB byte → permanently disable force output
bool force_killed = false;

// ============================================================
// FORCE MODEL (unchanged)
// ============================================================
float K_WALL = 30.0f;
float B_WALL = 0.0f;

// ============================================================
// MOTOR DRIVE CONSTANTS (unchanged)
// ============================================================
const float TORQUE_TO_DUTY_K = 0.0183f;
const int   PWM_OUTPUT_CAP   = 153;     // 60% of 255, Hapkit thermal limit
const float JAC_PERTURB      = 1e-4f;
const bool  FORCE_OUTPUT_ENABLED = true;

// ============================================================
// SHAPE — 6 cm × 6 cm square centered at (0, 0.10)
// ============================================================
// ----- duck, 50 pts, CCW -----
const int DUCK_N = 20;
const float DUCK_PTS[DUCK_N][2] PROGMEM = {
    { -0.00800f,  0.04611f},
    { -0.05000f,  0.02110f},
    { -0.04673f, -0.02139f},
    { -0.04344f, -0.02426f},
    { -0.03949f, -0.02062f},
    { -0.00964f, -0.00535f},
    {  0.00176f, -0.00880f},
    {  0.00143f, -0.01145f},
    { -0.00922f, -0.02893f},
    { -0.00278f, -0.04611f},
    {  0.02506f, -0.04539f},
    {  0.03691f, -0.03146f},
    {  0.04921f, -0.03059f},
    {  0.05000f, -0.02927f},
    {  0.04529f, -0.01986f},
    {  0.03294f, -0.01619f},
    {  0.03059f, -0.01162f},
    {  0.04502f,  0.00585f},
    {  0.04302f,  0.02963f},
    { -0.00800f,  0.04611f}
};
const float DUCK_K_WALL = 50.0f;
const float DUCK_B_WALL = 0.03f;

// ----- bell, 20 pts, CCW -----
const int BELL_N = 20;
const float BELL_PTS[BELL_N][2] PROGMEM = {
    { -0.00000f, -0.05000f},
    {  0.00902f, -0.04267f},
    {  0.00730f, -0.03493f},
    {  0.00733f, -0.03477f},
    {  0.02088f, -0.02680f},
    {  0.03676f,  0.01834f},
    {  0.04516f,  0.03429f},
    {  0.03288f,  0.04239f},
    {  0.01120f,  0.04253f},
    {  0.00755f,  0.05000f},
    { -0.00755f,  0.05000f},
    { -0.01120f,  0.04253f},
    { -0.03288f,  0.04239f},
    { -0.04516f,  0.03429f},
    { -0.03676f,  0.01834f},
    { -0.02088f, -0.02680f},
    { -0.00733f, -0.03477f},
    { -0.00730f, -0.03493f},
    { -0.00902f, -0.04267f},
    { -0.00000f, -0.05000f}
};
const float BELL_K_WALL = 50.0f;
const float BELL_B_WALL = 0.0f;

// //----- banana, 20 pts, CCW -----
const int BANANA_N = 20;
const float BANANA_PTS[BANANA_N][2] PROGMEM = {
    { -0.04365f, -0.03519f},
    { -0.03862f, -0.03620f},
    { -0.03224f, -0.03317f},
    { -0.03160f, -0.03108f},
    { -0.03320f, -0.02556f},
    { -0.02664f, -0.00755f},
    { -0.01193f,  0.00329f},
    {  0.04736f,  0.00977f},
    {  0.05000f,  0.01311f},
    {  0.04948f,  0.01979f},
    {  0.04044f,  0.02971f},
    { -0.00056f,  0.03620f},
    { -0.03360f,  0.02368f},
    { -0.05000f, -0.00258f},
    { -0.04891f, -0.01417f},
    { -0.04293f, -0.01954f},
    { -0.04253f, -0.02025f},
    { -0.04168f, -0.03119f},
    { -0.04389f, -0.03340f},
    { -0.04365f, -0.03519f}
};
const float BANANA_K_WALL = 30.0f;
const float BANANA_B_WALL = 0.03f;

// ----- fish, 20 pts, CCW -----
const int FISH_N = 20;
const float FISH_PTS[FISH_N][2] PROGMEM = {
    {  0.05000f, -0.03438f},
    {  0.04406f, -0.02006f},
    {  0.04119f,  0.00626f},
    {  0.04952f,  0.03438f},
    {  0.03319f,  0.02838f},
    {  0.02185f,  0.02001f},
    {  0.01531f,  0.01174f},
    {  0.00365f,  0.01618f},
    { -0.01638f,  0.02101f},
    { -0.03055f,  0.01845f},
    { -0.04274f,  0.01037f},
    { -0.05000f,  0.00102f},
    { -0.04410f, -0.00654f},
    { -0.03870f, -0.01131f},
    { -0.02690f, -0.01672f},
    { -0.01073f, -0.01688f},
    {  0.01364f, -0.00754f},
    {  0.02034f, -0.01432f},
    {  0.02487f, -0.01882f},
    {  0.05000f, -0.03438f}
};
const float FISH_K_WALL = 30.0f;
const float FISH_B_WALL = 0.0f;

// ----- horseshoe, 20 pts, CCW -----
const int HORSESHOE_N = 20;
const float HORSESHOE_PTS[HORSESHOE_N][2] PROGMEM = {
    { -0.03866f, -0.05000f},
    { -0.01951f, -0.03805f},
    { -0.02731f, -0.01591f},
    { -0.02640f,  0.01370f},
    { -0.01117f,  0.02920f},
    {  0.01194f,  0.02885f},
    {  0.02687f,  0.01236f},
    {  0.02894f, -0.00644f},
    {  0.01957f, -0.03840f},
    {  0.03930f, -0.04981f},
    {  0.04570f, -0.03838f},
    {  0.03778f, -0.03189f},
    {  0.04503f,  0.00317f},
    {  0.03209f,  0.03730f},
    { -0.00075f,  0.05000f},
    { -0.03283f,  0.03654f},
    { -0.04514f,  0.00098f},
    { -0.03797f, -0.03290f},
    { -0.04570f, -0.03958f},
    { -0.03866f, -0.05000f}
};
const float HORSESHOE_K_WALL = 50.0f;   // same as bell
const float HORSESHOE_B_WALL = 0.0f;    // same as bell

// ----- mushroom, 20 pts, CCW -----
const int MUSHROOM_N = 20;
const float MUSHROOM_PTS[MUSHROOM_N][2] PROGMEM = {
    { -0.00091f, -0.05000f},
    {  0.01890f, -0.04632f},
    {  0.04305f, -0.02557f},
    {  0.04945f, -0.00487f},
    {  0.04694f,  0.00190f},
    {  0.03936f,  0.00570f},
    {  0.01419f,  0.00718f},
    {  0.01960f,  0.03896f},
    {  0.01795f,  0.04535f},
    {  0.01305f,  0.04936f},
    { -0.01419f,  0.05000f},
    { -0.01969f,  0.04692f},
    { -0.02173f,  0.04047f},
    { -0.01678f,  0.00618f},
    { -0.04219f,  0.00539f},
    { -0.04753f,  0.00173f},
    { -0.04945f, -0.00484f},
    { -0.04439f, -0.02310f},
    { -0.02049f, -0.04558f},
    { -0.00091f, -0.05000f}
};
const float MUSHROOM_K_WALL = 40.0f;    // same as banana
const float MUSHROOM_B_WALL = 0.03f;    // same as banana

// ----- square, 4 corners, CCW (6 cm × 6 cm) -----
const int SQUARE_N = 5;
const float SQUARE_PTS[SQUARE_N][2] PROGMEM = {
    { -0.03000f, -0.03000f},
    {  0.03000f, -0.03000f},
    {  0.03000f,  0.03000f},
    { -0.03000f,  0.03000f},
    { -0.03000f, -0.03000f}
};
const float SQUARE_K_WALL = 50.0f;   // high stiffness
const float SQUARE_B_WALL = 0.0f;    // no damping

const float (*activeShape)[2] = FISH_PTS;
int activeShapeN = FISH_N;

inline float shapeX(int i) {
  return pgm_read_float(&activeShape[i][0]);
}

inline float shapeY(int i) {
  return pgm_read_float(&activeShape[i][1]) + 0.10f;
}

// ============================================================
// ENCODER STATE — TWO COPIES, one per sensor
// ============================================================

EncoderState enc_M1 = {0,0,0,0,0,0,0,0,0,false,0};
EncoderState enc_M5 = {0,0,0,0,0,0,0,0,0,false,0};
const int flipThresh = 700;

// Last valid pen-tip position (used when FK fails)
float xh_persist = 0.0f, yh_persist = 0.0f;

// ============================================================
// POLYGON GEOMETRY HELPERS (unchanged from previous firmware)
// ============================================================
float distSqToSegment(float px, float py,
                      float ax, float ay,
                      float bx, float by,
                      float &nx, float &ny) {
  float vx = bx - ax;
  float vy = by - ay;
  float wx = px - ax;
  float wy = py - ay;

  float seg_len2 = vx*vx + vy*vy;
  if (seg_len2 < 1e-12f) {
    nx = ax; ny = ay;
    float dx = px - ax, dy = py - ay;
    return dx*dx + dy*dy;
  }

  float t = (wx*vx + wy*vy) / seg_len2;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;

  nx = ax + t * vx;
  ny = ay + t * vy;
  float dx = px - nx, dy = py - ny;
  return dx*dx + dy*dy;
}

// Find the TWO nearest points on the polygon boundary.
// Used by computeForce to blend forces from the two nearest edges,
// which smooths out direction discontinuities at corner bisectors.
//
// Outputs:
//   nx1, ny1, d2_1 = nearest point and its squared distance
//   nx2, ny2, d2_2 = second-nearest point and its squared distance
// If the polygon has fewer than 2 edges (shouldn't happen), the second
// is set equal to the first.
void findTwoNearestOnPolygon(float px, float py,
                              float &nx1, float &ny1, float &d2_1,
                              float &nx2, float &ny2, float &d2_2) {
  d2_1 = 1e30f;  d2_2 = 1e30f;
  nx1 = px;      ny1 = py;
  nx2 = px;      ny2 = py;

  for (int i = 0; i < activeShapeN; i++) {
    int j = (i + 1) % activeShapeN;
    float seg_nx, seg_ny;
  float d2 = distSqToSegment(px, py,
                           shapeX(i), shapeY(i),
                           shapeX(j), shapeY(j),
                           seg_nx, seg_ny);
    if (d2 < d2_1) {
      // Demote previous best to second, then update best.
      d2_2 = d2_1; nx2 = nx1; ny2 = ny1;
      d2_1 = d2;   nx1 = seg_nx; ny1 = seg_ny;
    } else if (d2 < d2_2) {
      d2_2 = d2;   nx2 = seg_nx; ny2 = seg_ny;
    }
  }
}

// Convenience wrapper for code that only needs ONE nearest point.
// (Kept for any other callers; not used by computeForce anymore.)
float findNearestPointOnPolygon(float px, float py, float &nx, float &ny) {
  float nx2, ny2, d2_1, d2_2;
  findTwoNearestOnPolygon(px, py, nx, ny, d2_1, nx2, ny2, d2_2);
  return d2_1;
}

bool pointInPolygon(float px, float py) {
  bool inside = false;
  for (int i = 0, j = activeShapeN - 1; i < activeShapeN; j = i++) {
    float xi = shapeX(i), yi = shapeY(i);
    float xj = shapeX(j), yj = shapeY(j);

    bool crosses = ((yi > py) != (yj > py)) &&
                   (px < (xj - xi) * (py - yi) / (yj - yi) + xi);
    if (crosses) inside = !inside;
  }
  return inside;
}

// ============================================================
// FORCE COMPUTATION (OG)
// ============================================================

// void computeForce(float xh, float yh, float &Fx, float &Fy) {
//   // Inside the polygon? No force (this is the "interior" — pen moves freely).
//   if (pointInPolygon(xh, yh)) {
//     Fx = 0.0f;
//     Fy = 0.0f;
//     return;
//   }

//   // Outside: find the nearest point on the polygon boundary,
//   // then apply spring force pulling pen back toward it.
//   float nx, ny;
//   findNearestPointOnPolygon(xh, yh, nx, ny);
//   Fx = K_WALL * (nx - xh);
//   Fy = K_WALL * (ny - yh);

//   Fx -= B_WALL * vx_filt;
//   Fy -= B_WALL * vy_filt;
// }

// ============================================================
// SEGMENTS FORCE COMPUTATION 
// ============================================================

// void computeForce(float xh, float yh, float &Fx, float &Fy) {
//   // Outside the polygon = free space, no force.
//   if (!pointInPolygon(xh, yh)) {
//     Fx = 0.0f;
//     Fy = 0.0f;
//     return;
//   }

//   // Inside: find the two nearest edges and blend their restoring forces
//   // by inverse-square weighting. This smooths the 90° direction snap
//   // that would otherwise occur on a corner's bisector.
//   float nx1, ny1, d2_1, nx2, ny2, d2_2;
//   findTwoNearestOnPolygon(xh, yh, nx1, ny1, d2_1, nx2, ny2, d2_2);

//   // Per-edge restoring force vectors (each pushes toward its nearest pt).
//   float F1x = K_WALL * (nx1 - xh) ;
//   float F1y = K_WALL * (ny1 - yh) ;
//   float F2x = K_WALL * (nx2 - xh) ;
//   float F2y = K_WALL * (ny2 - yh) ;

//   // Inverse-square weights. Add a tiny epsilon to avoid divide-by-zero
//   // when the pen is exactly on a boundary point.
//   const float EPS = 1e-9f;
//   float w1 = 1.0f / (d2_1 + EPS);
//   float w2 = 1.0f / (d2_2 + EPS);
//   float wsum = w1 + w2;

// //   Fx = ((w1 * F1x + w2 * F2x) / wsum);
// //   Fy = ((w1 * F1y + w2 * F2y) / wsum);

// //uncomment for damping
//   Fx = ((w1 * F1x + w2 * F2x) / wsum) - B_WALL * vx_filt;
//   Fy = ((w1 * F1y + w2 * F2y) / wsum) - B_WALL * vy_filt;
// }

// ============================================================
// PROXY-BASED FORCE COMPUTATION
// ============================================================
// The proxy is a virtual point that:
//   - Follows the pen freely when INSIDE the shape
//   - Gets stuck on the nearest boundary point when OUTSIDE
// Force = spring between proxy and actual pen position
// This prevents "jumping through" thin features like the handle

// // old Proxy
// void computeForce(float xh, float yh, float &Fx, float &Fy) {
//   if (!pointInPolygon(xh, yh)) {
//     proxy_x = xh;
//     proxy_y = yh;
//     Fx = 0.0f;
//     Fy = 0.0f;
//     return;
//   }

//   float nx1, ny1, d2_1, nx2, ny2, d2_2;
//   findTwoNearestOnPolygon(xh, yh, nx1, ny1, d2_1, nx2, ny2, d2_2);

//   proxy_x = nx1;
//   proxy_y = ny1;

//   // Spring toward wall + damping opposing motion
//   // Damping only when moving INTO the wall (v_normal > 0)
//   float spring_x = K_WALL * (proxy_x - xh);
//   float spring_y = K_WALL * (proxy_y - yh);

//   Fx = spring_x - B_WALL * vx_filt;
//   Fy = spring_y - B_WALL * vy_filt;
// }

// void computeForce(float xh, float yh, float &Fx, float &Fy) {

//   if (!pointInPolygon(xh, yh)) {
//     // OUTSIDE: proxy follows pen freely
//     proxy_x = xh;
//     proxy_y = yh;
//     Fx = 0.0f;
//     Fy = 0.0f;
//     return;
//   }

//   // INSIDE: move proxy toward pen but only along the boundary
//   // Never let proxy jump — slide it toward pen incrementally
  
//   // Direction from proxy toward pen
//   float dx = xh - proxy_x;
//   float dy = yh - proxy_y;
//   float dist = sqrtf(dx*dx + dy*dy);
  
//   if (dist > 1e-6f) {
//     // Try moving proxy a small step toward pen
//     float step = 0.002f;  // 2mm step max per loop
//     float new_px = proxy_x + step * (dx/dist);
//     float new_py = proxy_y + step * (dy/dist);
    
//     // Only accept the move if new position stays OUTSIDE the shape
//     // (proxy must stay on or outside the boundary)
//     if (!pointInPolygon(new_px, new_py)) {
//       proxy_x = new_px;
//       proxy_y = new_py;
//     } else {
//       // New position is inside — project back to boundary
//       float nx, ny;
//       findNearestPointOnPolygon(new_px, new_py, nx, ny);
//       proxy_x = nx;
//       proxy_y = ny;
//     }
//   }

//   // Force pulls pen back toward proxy
//   Fx = K_WALL * (proxy_x - xh) - B_WALL * vx_filt;
//   Fy = K_WALL * (proxy_y - yh) - B_WALL * vy_filt;
// }

// void computeForce(float xh, float yh, float &Fx, float &Fy) {
//   if (!pointInPolygon(xh, yh)) {
//     proxy_x = xh;
//     proxy_y = yh;
//     Fx = 0.0f;
//     Fy = 0.0f;
//     return;
//   }

//   // Move proxy ALL THE WAY toward pen in one step
//   // but project back to boundary if it crosses
//   float nx, ny;
//   findNearestPointOnPolygon(xh, yh, nx, ny);
//   proxy_x = nx;
//   proxy_y = ny;

//   // Now compute blended force from proxy position
//   float nx1, ny1, d2_1, nx2, ny2, d2_2;
//   findTwoNearestOnPolygon(xh, yh, nx1, ny1, d2_1, nx2, ny2, d2_2);

//   float F1x = K_WALL * (nx1 - xh);
//   float F1y = K_WALL * (ny1 - yh);
//   float F2x = K_WALL * (nx2 - xh);
//   float F2y = K_WALL * (ny2 - yh);

//   const float EPS = 1e-9f;
//   float w1 = 1.0f / (d2_1 + EPS);
//   float w2 = 1.0f / (d2_2 + EPS);
//   float wsum = w1 + w2;

//   // Spring force + damping opposing motion
//   Fx = (w1 * F1x + w2 * F2x) / wsum - B_WALL * vx_filt;
//   Fy = (w1 * F1y + w2 * F2y) / wsum - B_WALL * vy_filt;
// }

// ============================================================
// PROXY + ROUNDED CORNERS
// ============================================================
// void computeForce(float xh, float yh, float &Fx, float &Fy) {
//   if (!pointInPolygon(xh, yh)) {
//     Fx = 0.0f;
//     Fy = 0.0f;
//     return;
//   }

//   // Just find nearest two points and blend — no proxy sliding
//   float nx1, ny1, d2_1, nx2, ny2, d2_2;
//   findTwoNearestOnPolygon(xh, yh, nx1, ny1, d2_1, nx2, ny2, d2_2);

//   float F1x = K_WALL * (nx1 - xh);
//   float F1y = K_WALL * (ny1 - yh);
//   float F2x = K_WALL * (nx2 - xh);
//   float F2y = K_WALL * (ny2 - yh);

//   const float EPS = 1e-9f;
//   float w1 = 1.0f / (d2_1 + EPS);
//   float w2 = 1.0f / (d2_2 + EPS);
//   float wsum = w1 + w2;

//   Fx = (w1 * F1x + w2 * F2x) / wsum;
//   Fy = (w1 * F1y + w2 * F2y) / wsum;
  
// }

// ============================================================
// TRADITIONAL PROXY
// ============================================================
// Line segment intersection helper
// Returns true if pen path (P→Q) crosses polygon segment (A→B)
// t_pen = how far along pen path [0,1], t_seg = where on polygon segment [0,1]

bool segmentCross(float px, float py, float qx, float qy,
                  float ax, float ay, float bx, float by,
                  float &t_pen, float &t_seg) {
  float dqx = qx-px, dqy = qy-py;
  float dbx = bx-ax, dby = by-ay;
  float denom = dqx*dby - dqy*dbx;
  if (fabsf(denom) < 1e-10f) return false;
  float dpx = ax-px, dpy = ay-py;
  t_pen = (dpx*dby - dpy*dbx) / denom;
  t_seg = (dpx*dqy - dpy*dqx) / denom;
  return (t_pen >= -0.01f && t_pen <= 1.01f &&
          t_seg >=  0.0f  && t_seg <= 1.0f);
}

void proxyGetPos(int seg, float t, float &px, float &py) {
  int j = (seg + 1) % activeShapeN;
  px = shapeX(seg) + t * (shapeX(j) - shapeX(seg));
  py = shapeY(seg) + t * (shapeY(j) - shapeY(seg));
}

  void computeForce(float xh, float yh, float &Fx, float &Fy) {
  bool currently_inside = pointInPolygon(xh, yh);

  if (!currently_inside) {
    was_inside_shape = false;

    // Track nearest segment so we know where pen will re-enter
    float best_d2 = 1e30f;
    for (int i = 0; i < activeShapeN; i++) {
      int j = (i + 1) % activeShapeN;
      float ax = shapeX(i), ay = shapeY(i);
      float bx = shapeX(j), by = shapeY(j);
      float vx = bx-ax, vy = by-ay;
      float len2 = vx*vx + vy*vy;
      if (len2 < 1e-12f) continue;
      float t = ((xh-ax)*vx + (yh-ay)*vy) / len2;
      t = constrain(t, 0.0f, 1.0f);
      float nx = ax+t*vx, ny = ay+t*vy;
      float d2 = (xh-nx)*(xh-nx) + (yh-ny)*(yh-ny);
      if (d2 < best_d2) {
        best_d2 = d2;
        proxy_seg_idx = i;
        proxy_seg_t   = t;
      }
    }
    proxyGetPos(proxy_seg_idx, proxy_seg_t, proxy_x, proxy_y);
    Fx = 0.0f;
    Fy = 0.0f;
    return;
  }

  // Currently inside
  if (!was_inside_shape) {
    // Just entered — find the crossing segment by intersecting
    // pen's movement path (xh_prev→xh) with each polygon segment.
    // Place proxy exactly at the crossing point, no jumping.
    float best_t_pen = 1e30f;
    for (int i = 0; i < activeShapeN; i++) {
      int j = (i + 1) % activeShapeN;
      float t_pen, t_seg;
      if (segmentCross(xh_prev, yh_prev, xh, yh,
                       shapeX(i), shapeY(i),
                       shapeX(j), shapeY(j),
                       t_pen, t_seg)) {
        if (t_pen < best_t_pen) {
          best_t_pen  = t_pen;
          proxy_seg_idx = i;
          proxy_seg_t   = t_seg;
        }
      }
    }
    // If no crossing found (fast motion missed it), use previously
    // tracked nearest segment — already set while outside
    proxyGetPos(proxy_seg_idx, proxy_seg_t, proxy_x, proxy_y);
    was_inside_shape = true;
  }

  // Slide proxy along boundary — ADJACENT segments only, never teleport
  float dx = xh - proxy_x;
  float dy = yh - proxy_y;

  int j = (proxy_seg_idx + 1) % activeShapeN;
  float ax = shapeX(proxy_seg_idx), ay = shapeY(proxy_seg_idx);
  float bx = shapeX(j),             by = shapeY(j);
  float tx = bx-ax, ty = by-ay;
  float seg_len2 = tx*tx + ty*ty;

  if (seg_len2 > 1e-12f) {
    float delta_t = (dx*tx + dy*ty) / seg_len2;
    // Cap per-loop movement to prevent skipping segments
    delta_t = constrain(delta_t, -0.5f, 0.5f);
    float new_t = proxy_seg_t + delta_t;

    if (new_t >= 1.0f) {
      // Step to next adjacent segment
      proxy_seg_idx = (proxy_seg_idx + 1) % activeShapeN;
      proxy_seg_t   = 0.0f;
    } else if (new_t <= 0.0f) {
      // Step to previous adjacent segment
      proxy_seg_idx = (proxy_seg_idx - 1 + activeShapeN) % activeShapeN;
      proxy_seg_t   = 1.0f;
    } else {
      proxy_seg_t = new_t;
    }
  }

  proxyGetPos(proxy_seg_idx, proxy_seg_t, proxy_x, proxy_y);

  Fx = K_WALL * (proxy_x - xh) - B_WALL * vx_filt;
  Fy = K_WALL * (proxy_y - yh) - B_WALL * vy_filt;
}


// ============================================================
// ENCODER READING — generalized to operate on a given EncoderState
// ============================================================
// Same flip-detection algorithm as before, parameterized so we can
// call it once per sensor with independent state.
float readEncoderTheta(int analogPin, EncoderState &s,
                       float enc_m, float enc_b) {
  s.rawPos        = analogRead(analogPin);
  s.rawDiff       = s.rawPos - s.lastRawPos;
  s.lastRawDiff   = s.rawPos - s.lastLastRawPos;
  s.rawOffset     = abs(s.rawDiff);
  s.lastRawOffset = abs(s.lastRawDiff);
  s.lastLastRawPos = s.lastRawPos;
  s.lastRawPos    = s.rawPos;

  if ((s.lastRawOffset > flipThresh) && (!s.flipped)) {
    if (s.lastRawDiff > 0) s.flipNumber--; else s.flipNumber++;
    if (s.rawOffset > flipThresh) {
      s.updatedPos = s.rawPos + s.flipNumber * s.rawOffset;
      s.tempOffset = s.rawOffset;
    } else {
      s.updatedPos = s.rawPos + s.flipNumber * s.lastRawOffset;
      s.tempOffset = s.lastRawOffset;
    }
    s.flipped = true;
  } else {
    s.updatedPos = s.rawPos + s.flipNumber * s.tempOffset;
    s.flipped = false;
  }

  float theta_deg = enc_m * s.updatedPos + enc_b;
  float theta_rad = theta_deg * (PI / 180.0f);
  return theta_rad;
}

// ============================================================
// FORWARD KINEMATICS (unchanged)
// ============================================================
bool computeFwdKin(float theta1, float theta5, float &x3, float &y3) {
  float x2 = LINK_A1 * cosf(theta1);
  float y2 = LINK_A1 * sinf(theta1);
  float x4 = -LINK_A5 + LINK_A4 * cosf(theta5);
  float y4 =  LINK_A4 * sinf(theta5);

  float dx = x4 - x2;
  float dy = y4 - y2;
  float dP2P4 = sqrtf(dx*dx + dy*dy);

  if (dP2P4 < 1e-6f) return false;

  float dP2Ph = (LINK_A2*LINK_A2 - LINK_A3*LINK_A3 + dP2P4*dP2P4) / (2.0f * dP2P4);
  float h2    = LINK_A2*LINK_A2 - dP2Ph*dP2Ph;

  if (h2 < 0.0f) return false;
  float dP3Ph = sqrtf(h2);

  float xh_pt = x2 + (dP2Ph / dP2P4) * dx;
  float yh_pt = y2 + (dP2Ph / dP2P4) * dy;

  x3 = xh_pt + (dP3Ph / dP2P4) * dy;
  y3 = yh_pt - (dP3Ph / dP2P4) * dx;
  return true;
}

// Get pen-tip position from both thetas. No more "partner" concept —
// both are read locally so they're always synchronized.
void getPenTipPosition(float &x, float &y, float theta1, float theta5) {
  float x3, y3;
  if (computeFwdKin(theta1, theta5, x3, y3)) {
    xh_persist = x3;
    yh_persist = y3;
  }
  x = xh_persist;
  y = yh_persist;

}


// ============================================================
// JACOBIAN + MOTOR OUTPUT
// ============================================================
// Given Cartesian force and both joint angles, compute the torque
// for one motor by perturbing its own joint angle and observing
// how the pen tip moves (numerical finite-difference Jacobian column).
//
// `perturb_motor`: 1 to perturb theta1 (left motor), 5 to perturb theta5.
//
// Returns the PWM output (0..PWM_OUTPUT_CAP).
// Also sets dir_out: true = DIR pin HIGH, false = LOW.
int computeMotorOutputPWM(float Fx, float Fy,
                          float theta1, float theta5,
                          int perturb_motor,
                          float &tau_out, float &duty_out,
                          bool &dir_out) {
  float x0, y0;
  if (!computeFwdKin(theta1, theta5, x0, y0)) {
    tau_out = 0.0f; duty_out = 0.0f; dir_out = false; return 0;
  }

  float x_p, y_p;
  if (perturb_motor == 1) {
    if (!computeFwdKin(theta1 + JAC_PERTURB, theta5, x_p, y_p)) {
      tau_out = 0.0f; duty_out = 0.0f; dir_out = false; return 0;
    }
  } else {
    if (!computeFwdKin(theta1, theta5 + JAC_PERTURB, x_p, y_p)) {
      tau_out = 0.0f; duty_out = 0.0f; dir_out = false; return 0;
    }
  }

  float dxh_dth = (x_p - x0) / JAC_PERTURB;
  float dyh_dth = (y_p - y0) / JAC_PERTURB;

  float tau = dxh_dth * Fx + dyh_dth * Fy;
  tau_out = tau;
  dir_out = (tau >= 0.0f);

  float duty = sqrtf(fabsf(tau) / TORQUE_TO_DUTY_K);
  if (duty > 1.0f) duty = 1.0f;
  if (duty < 0.0f) duty = 0.0f;
  duty_out = duty;

  int pwm = (int)(duty * 255.0f);
  if (pwm > PWM_OUTPUT_CAP) pwm = PWM_OUTPUT_CAP;
  return pwm;
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  // High PWM frequency on BOTH motor PWM pins (5 and 6 share Timer 0).
  // CAUTION: this changes Timer 0 prescaler /64 → /1, making
  // millis()/micros()/delay() return values 64x larger than wall-clock.
  // delayMicroseconds() is unaffected.
  setPwmFrequency(PWM_PIN_M5, 1);
  setPwmFrequency(PWM_PIN_M1, 1);  // pins 5 and 6 are on the same timer, so this
                                   // call is technically redundant — but harmless
                                   // and self-documenting.

  // Motor pins — initialize to off
  pinMode(PWM_PIN_M5, OUTPUT);
  pinMode(DIR_PIN_M5, OUTPUT);
  pinMode(PWM_PIN_M1, OUTPUT);
  pinMode(DIR_PIN_M1, OUTPUT);
  analogWrite(PWM_PIN_M5, 0);
  analogWrite(PWM_PIN_M1, 0);
  digitalWrite(DIR_PIN_M5, LOW);
  digitalWrite(DIR_PIN_M1, LOW);

  pinMode(sensorPos_M5, INPUT);
  pinMode(sensorPos_M1, INPUT);

  Serial.println("Pantograph single-board firmware ready.");
  Serial.println("U1 drives both motors, reads both MR sensors directly.");

  if (MOTOR_TEST_ENABLED) {
    Serial.println("# MOTOR TEST MODE ARMED — send 'g' to start.");
  }
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  float theta1 = readEncoderTheta(sensorPos_M1, enc_M1, ENC_M_M1, ENC_B_M1);
  float theta5 = readEncoderTheta(sensorPos_M5, enc_M5, ENC_M_M5, ENC_B_M5);

  float xh, yh;
  getPenTipPosition(xh, yh, theta1, theta5);

  // Velocity — compute but DON'T update xh_prev yet
  float vx_raw = (xh - xh_prev) / DT_LOOP;
  float vy_raw = (yh - yh_prev) / DT_LOOP;
  if (fabsf(vx_raw) > 0.5f) vx_raw = 0.0f;
  if (fabsf(vy_raw) > 0.5f) vy_raw = 0.0f;
  vx_filt = 0.95f * vx_filt + 0.05f * vx_raw;
  vy_filt = 0.95f * vy_filt + 0.05f * vy_raw;

  // computeForce uses xh_prev/yh_prev = last frame's position
  float Fx, Fy;
  computeForce(xh, yh, Fx, Fy);

  // NOW update prev position
  xh_prev = xh;
  yh_prev = yh;


  // Filter force to smooth out jitter
  Fx_filt = 0.7f * Fx_filt + 0.3f * Fx;
  Fy_filt = 0.7f * Fy_filt + 0.3f * Fy;

  // 4. Compute torque + PWM for EACH motor.
  float tau_M1 = 0.0f, duty_M1 = 0.0f;
  float tau_M5 = 0.0f, duty_M5 = 0.0f;
  int  pwm_M1 = 0,    pwm_M5 = 0;
  bool dir_M1 = false, dir_M5 = false;

  pwm_M1 = computeMotorOutputPWM(Fx_filt, Fy_filt, theta1, theta5, 1, tau_M1, duty_M1, dir_M1);
  pwm_M5 = computeMotorOutputPWM(Fx_filt, Fy_filt, theta1, theta5, 5, tau_M5, duty_M5, dir_M5);

// 5. Apply forces to BOTH motors. Serial now used for commands (NOT auto-kill)
if (FORCE_OUTPUT_ENABLED && !force_killed && !motor_killed && !motor_test_started) {

  // --- read ALL incoming serial commands ---
  while (Serial.available()) {
    char c = Serial.read();

    // (A) safety kill switch
    if (c == 'X' || c == 'x') {
      force_killed = true;

      analogWrite(PWM_PIN_M5, 0);
      analogWrite(PWM_PIN_M1, 0);
      digitalWrite(DIR_PIN_M5, LOW);
      digitalWrite(DIR_PIN_M1, LOW);

      Serial.println("# FORCE OUTPUT KILLED");
    }

    // (B) shape switching
    setShape(c);
  }

  // --- motor output still runs normally ---
  digitalWrite(DIR_PIN_M5, dir_M5 ? LOW : HIGH);
  digitalWrite(DIR_PIN_M1, dir_M1 ? LOW : HIGH);
  analogWrite(PWM_PIN_M5, pwm_M5);
  analogWrite(PWM_PIN_M1, pwm_M1);

} else {
  analogWrite(PWM_PIN_M5, 0);
  analogWrite(PWM_PIN_M1, 0);
}
  // // 6. CSV output for Processing GUI — same format as before.
  static int csvCounter = 0;
  if (++csvCounter >= 5) {
    Serial.print(xh, 4);
    Serial.print(',');
    Serial.print(yh, 4);
    Serial.print(',');
    Serial.print(Fx, 3);
    Serial.print(',');
    Serial.println(Fy, 3);
    csvCounter = 0;
  }

  // 7. Human-readable debug — both motors now.
  static int printCounter = 0;
  if (++printCounter >= 500) {
    float theta1_deg = ENC_M_M1 * enc_M1.updatedPos + ENC_B_M1;
    float theta5_deg = ENC_M_M5 * enc_M5.updatedPos + ENC_B_M5;

    Serial.print("# t1=");     Serial.print(theta1_deg, 1);
    Serial.print("  t5=");     Serial.print(theta5_deg, 1);
    Serial.print("  vx="); Serial.print(vx_filt, 4);
    Serial.print("  vy="); Serial.print(vy_filt, 4);
    Serial.print("  xh=");     Serial.print(xh, 4);
    Serial.print("  yh=");     Serial.print(yh, 4);
    Serial.print("  Fx=");     Serial.print(Fx, 3);
    Serial.print("  Fy=");     Serial.print(Fy, 3);
    Serial.print("  tauM1=");  Serial.print(tau_M1, 4);
    Serial.print("  pwmM1=");  Serial.print(pwm_M1);
    Serial.print("  tauM5=");  Serial.print(tau_M5, 4);
    Serial.print("  pwmM5=");  Serial.print(pwm_M5);
    Serial.println();
    printCounter = 0;
  }

  // (Motor twitch test machinery omitted — not used in single-board setup.
  //  If you want it back, copy from the old motor5 firmware. Kept disabled
  //  via MOTOR_TEST_ENABLED = false anyway.)
}

// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// Copied from A3/A4 Hapkit template.
// --------------------------------------------------------------
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

void setShape(char c) {
  switch (c) {

    case 'F':
    case 'f':
      activeShape = FISH_PTS;
      activeShapeN = FISH_N;
      K_WALL = FISH_K_WALL;
      B_WALL = FISH_B_WALL;
      Serial.println("# shape: fish");
      break;

    case 'B':
    case 'b':
      activeShape = BANANA_PTS;
      activeShapeN = BANANA_N;
      K_WALL = BANANA_K_WALL;
      B_WALL = BANANA_B_WALL;
      Serial.println("# shape: banana");
      break;

    case 'D':
    case 'd':
      activeShape = DUCK_PTS;
      activeShapeN = DUCK_N;
      K_WALL = DUCK_K_WALL;
      B_WALL = DUCK_B_WALL;
      Serial.println("# shape: duck");
      break;

    case 'E':
    case 'e':
      activeShape = BELL_PTS;
      activeShapeN = BELL_N;
      K_WALL = BELL_K_WALL;
      B_WALL = BELL_B_WALL;
      Serial.println("# shape: bell");
      break;

    case 'H':
    case 'h':
      activeShape = HORSESHOE_PTS;
      activeShapeN = HORSESHOE_N;
      K_WALL = HORSESHOE_K_WALL;
      B_WALL = HORSESHOE_B_WALL;
      Serial.println("# shape: horseshoe");
      break;

    case 'M':
    case 'm':
      activeShape = MUSHROOM_PTS;
      activeShapeN = MUSHROOM_N;
      K_WALL = MUSHROOM_K_WALL;
      B_WALL = MUSHROOM_B_WALL;
      Serial.println("# shape: mushroom");
      break;

    case 'S':
    case 's':
      activeShape = SQUARE_PTS;
      activeShapeN = SQUARE_N;
      K_WALL = SQUARE_K_WALL;
      B_WALL = SQUARE_B_WALL;
      Serial.println("# shape: square");
      break;
  }
}
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
const float K_WALL = 70.0f;
const float B_WALL = 0.01f;

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
// ----- pear, 50 pts, CCW -----
// const int SHAPE_N = 20;
// const float SHAPE_PTS[SHAPE_N][2] PROGMEM = {
//     {  0.00250f, -0.04966f},
//     {  0.02013f, -0.05000f},
//     {  0.02073f, -0.04931f},
//     {  0.01456f, -0.03732f},
//     {  0.00511f, -0.03418f},
//     { -0.00096f, -0.03711f},
//     { -0.00172f, -0.03673f},
//     { -0.00244f, -0.03343f},
//     { -0.00082f, -0.03279f},
//     {  0.01189f, -0.02203f},
//     {  0.03194f,  0.01886f},
//     {  0.02410f,  0.04206f},
//     { -0.01244f,  0.05000f},
//     { -0.03194f,  0.03514f},
//     { -0.02739f,  0.00013f},
//     { -0.01260f, -0.02948f},
//     { -0.00868f, -0.03094f},
//     { -0.00570f, -0.04273f},
//     { -0.00315f, -0.04409f},
//     {  0.00250f, -0.04966f}
// };

// ----- hammer, 20 pts, CCW -----
// const int SHAPE_N = 20;
// const float SHAPE_PTS[SHAPE_N][2] PROGMEM = {
//     {  0.02711f, -0.02543f},
//     {  0.03373f, -0.02405f},
//     {  0.04837f,  0.00085f},
//     {  0.04601f,  0.01031f},
//     {  0.05000f,  0.01459f},
//     {  0.04975f,  0.02316f},
//     {  0.03763f,  0.02543f},
//     {  0.03595f,  0.01676f},
//     {  0.03882f,  0.01165f},
//     {  0.03618f,  0.00656f},
//     { -0.01592f,  0.01238f},
//     { -0.01681f,  0.01369f},
//     { -0.04632f,  0.01819f},
//     { -0.04977f,  0.01622f},
//     { -0.05000f,  0.00457f},
//     { -0.01909f,  0.00083f},
//     { -0.01718f,  0.00216f},
//     {  0.03274f, -0.00498f},
//     {  0.03485f, -0.01254f},
//     {  0.02711f, -0.02543f}
// };

// //----- bunny, 20 pts, CCW -----
const int SHAPE_N = 20;
const float SHAPE_PTS[SHAPE_N][2] PROGMEM = {
    { -0.01763f, -0.05000f},
    { -0.00264f, -0.03686f},
    { -0.00055f, -0.01979f},
    {  0.00176f, -0.01174f},
    {  0.00271f, -0.02243f},
    {  0.00587f, -0.03725f},
    {  0.01637f, -0.04951f},
    {  0.02466f, -0.04697f},
    {  0.02921f, -0.02604f},
    {  0.02587f, -0.01030f},
    {  0.03181f,  0.00429f},
    {  0.03523f,  0.01726f},
    {  0.02185f,  0.04514f},
    {  0.00604f,  0.05000f},
    { -0.02288f,  0.04358f},
    { -0.03523f,  0.02065f},
    { -0.02506f, -0.00145f},
    { -0.02181f, -0.00934f},
    { -0.02595f, -0.03154f},
    { -0.01763f, -0.05000f}
};

// //----- square, 10 pts, CCW -----
// const int SHAPE_N = 10;
// const float SHAPE_PTS[SHAPE_N][2] PROGMEM = {
//     { -0.03500f,  0.13500f},
//     { -0.03500f,  0.10179f},
//     { -0.03500f,  0.06858f},
//     { -0.01167f,  0.06500f},
//     {  0.02154f,  0.06500f},
//     {  0.03500f,  0.07846f},
//     {  0.03500f,  0.11167f},
//     {  0.03142f,  0.13500f},
//     { -0.00179f,  0.13500f},
//     { -0.03500f,  0.13500f}
// };

// const int   SHAPE_N = 4;
// const float SHAPE_PTS[SHAPE_N][2] PROGMEM = {
//   { -0.03f, 0.07f },   // bottom-left
//   {  0.03f, 0.07f },   // bottom-right
//   {  0.03f, 0.13f },   // top-right
//   { -0.03f, 0.13f }    // top-left
// };

inline float shapeX(int i) { return pgm_read_float(&SHAPE_PTS[i][0]); }
inline float shapeY(int i) { return pgm_read_float(&SHAPE_PTS[i][1]) + 0.10f; }



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

  for (int i = 0; i < SHAPE_N; i++) {
    int j = (i + 1) % SHAPE_N;
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
  for (int i = 0, j = SHAPE_N - 1; i < SHAPE_N; j = i++) {
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

//   Fx = ((w1 * F1x + w2 * F2x) / wsum);
//   Fy = ((w1 * F1y + w2 * F2y) / wsum);

//uncomment for damping
  // Fx = ((w1 * F1x + w2 * F2x) / wsum) - B_WALL * vx_filt;
  // Fy = ((w1 * F1y + w2 * F2y) / wsum) - B_WALL * vy_filt;


// ============================================================
// PROXY-BASED FORCE COMPUTATION
// ============================================================
// The proxy is a virtual point that:
//   - Follows the pen freely when INSIDE the shape
//   - Gets stuck on the nearest boundary point when OUTSIDE
// Force = spring between proxy and actual pen position
// This prevents "jumping through" thin features like the handle

void computeForce(float xh, float yh, float &Fx, float &Fy) {
  if (!pointInPolygon(xh, yh)) {
    proxy_x = xh;
    proxy_y = yh;
    Fx = 0.0f;
    Fy = 0.0f;
    return;
  }

  float nx1, ny1, d2_1, nx2, ny2, d2_2;
  findTwoNearestOnPolygon(xh, yh, nx1, ny1, d2_1, nx2, ny2, d2_2);

  proxy_x = nx1;
  proxy_y = ny1;

  // Spring toward wall + damping opposing motion
  // Damping only when moving INTO the wall (v_normal > 0)
  float spring_x = K_WALL * (proxy_x - xh);
  float spring_y = K_WALL * (proxy_y - yh);

  Fx = spring_x - B_WALL * vx_filt;
  Fy = spring_y - B_WALL * vy_filt;
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
  // 1. Read BOTH encoders locally — synchronous, no comms latency.
  float theta1 = readEncoderTheta(sensorPos_M1, enc_M1, ENC_M_M1, ENC_B_M1);
  float theta5 = readEncoderTheta(sensorPos_M5, enc_M5, ENC_M_M5, ENC_B_M5);

  // 2. Compute pen-tip position from both thetas.
  float xh, yh;
  getPenTipPosition(xh, yh, theta1, theta5);

  // 3. Update velocity 
  float vx = (xh - xh_prev) / DT_LOOP;
  float vy = (yh - yh_prev) / DT_LOOP;
  vx_filt = 0.1f * vx_filt + 0.9f * vx;
  vy_filt = 0.1f * vy_filt + 0.9f * vy;
  xh_prev = xh;
  yh_prev = yh;

  // 3. Compute Cartesian force.
  float Fx, Fy;
  computeForce(xh, yh, Fx, Fy);

  // 4. Compute torque + PWM for EACH motor.
  float tau_M1 = 0.0f, duty_M1 = 0.0f;
  float tau_M5 = 0.0f, duty_M5 = 0.0f;
  int  pwm_M1 = 0,    pwm_M5 = 0;
  bool dir_M1 = false, dir_M5 = false;

  pwm_M1 = computeMotorOutputPWM(Fx, Fy, theta1, theta5, 1, tau_M1, duty_M1, dir_M1);
  pwm_M5 = computeMotorOutputPWM(Fx, Fy, theta1, theta5, 5, tau_M5, duty_M5, dir_M5);

  // 5. Apply forces to BOTH motors. Any USB Serial input kills both.
  if (FORCE_OUTPUT_ENABLED && !force_killed && !motor_killed && !motor_test_started) {
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      force_killed = true;
      analogWrite(PWM_PIN_M5, 0);
      analogWrite(PWM_PIN_M1, 0);
      digitalWrite(DIR_PIN_M5, LOW);
      digitalWrite(DIR_PIN_M1, LOW);
      Serial.println("# FORCE OUTPUT KILLED by serial input.");
    } else {
      digitalWrite(DIR_PIN_M5, dir_M5 ? LOW : HIGH);
      digitalWrite(DIR_PIN_M1, dir_M1 ? LOW : HIGH);
      analogWrite(PWM_PIN_M5, pwm_M5);
      analogWrite(PWM_PIN_M1, pwm_M1);
    }
  } else {
    analogWrite(PWM_PIN_M5, 0);
    analogWrite(PWM_PIN_M1, 0);
  }

  // 6. CSV output for Processing GUI — same format as before.
  static int csvCounter = 0;
  if (++csvCounter >= 500) {
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
  if (++printCounter >= 10) {
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
Hapkit) just provides 5V/GND to its own MR sensor and
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
const float K_WALL = 70.0f;

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
// const int SHAPE_N = 20;
// const float SHAPE_PTS[SHAPE_N][2] PROGMEM = {
//     { -0.00800f,  0.04611f},
//     { -0.05000f,  0.02110f},
//     { -0.04673f, -0.02139f},
//     { -0.04344f, -0.02426f},
//     { -0.03949f, -0.02062f},
//     { -0.00964f, -0.00535f},
//     {  0.00176f, -0.00880f},
//     {  0.00143f, -0.01145f},
//     { -0.00922f, -0.02893f},
//     { -0.00278f, -0.04611f},
//     {  0.02506f, -0.04539f},
//     {  0.03691f, -0.03146f},
//     {  0.04921f, -0.03059f},
//     {  0.05000f, -0.02927f},
//     {  0.04529f, -0.01986f},
//     {  0.03294f, -0.01619f},
//     {  0.03059f, -0.01162f},
//     {  0.04502f,  0.00585f},
//     {  0.04302f,  0.02963f},
//     { -0.00800f,  0.04611f}
// };

// ----- bell, 20 pts, CCW -----
// const int SHAPE_N = 20;
// const float SHAPE_PTS[SHAPE_N][2] PROGMEM = {
//     { -0.00000f, -0.05000f},
//     {  0.00902f, -0.04267f},
//     {  0.00730f, -0.03493f},
//     {  0.00733f, -0.03477f},
//     {  0.02088f, -0.02680f},
//     {  0.03676f,  0.01834f},
//     {  0.04516f,  0.03429f},
//     {  0.03288f,  0.04239f},
//     {  0.01120f,  0.04253f},
//     {  0.00755f,  0.05000f},
//     { -0.00755f,  0.05000f},
//     { -0.01120f,  0.04253f},
//     { -0.03288f,  0.04239f},
//     { -0.04516f,  0.03429f},
//     { -0.03676f,  0.01834f},
//     { -0.02088f, -0.02680f},
//     { -0.00733f, -0.03477f},
//     { -0.00730f, -0.03493f},
//     { -0.00902f, -0.04267f},
//     { -0.00000f, -0.05000f}
// };

// //----- banana, 20 pts, CCW -----
const int SHAPE_N = 20;
const float SHAPE_PTS[SHAPE_N][2] PROGMEM = {
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

// //----- fish, 10 pts, CCW -----
// const int SHAPE_N = 10;
// const float SHAPE_PTS[SHAPE_N][2] PROGMEM = {
//     { -0.05000f, -0.00048f},
//     { -0.03185f, -0.01907f},
//     {  0.00304f, -0.02172f},
//     {  0.02803f, -0.01074f},
//     {  0.02831f, -0.01060f},
//     {  0.02867f, -0.01102f},
//     {  0.04733f, -0.02281f},
//     {  0.04935f, -0.02281f},
//     {  0.05000f, -0.02213f},
//     {  0.04993f, -0.01985f},
//     {  0.04750f,  0.01225f},
//     {  0.04993f,  0.02208f},
//     {  0.04937f,  0.02276f},
//     {  0.04708f,  0.02281f},
//     {  0.02893f,  0.01116f},
//     {  0.02832f,  0.01044f},
//     {  0.02804f,  0.01055f},
//     {  0.00235f,  0.02159f},
//     { -0.03114f,  0.01881f},
//     { -0.05000f, -0.00048f}
// };


inline float shapeX(int i) { return pgm_read_float(&SHAPE_PTS[i][0]); }
inline float shapeY(int i) { return pgm_read_float(&SHAPE_PTS[i][1]) + 0.10f; }



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

  for (int i = 0; i < SHAPE_N; i++) {
    int j = (i + 1) % SHAPE_N;
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
  for (int i = 0, j = SHAPE_N - 1; i < SHAPE_N; j = i++) {
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

// const float B_WALL = 0.5f;
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

//   Fx = ((w1 * F1x + w2 * F2x) / wsum);
//   Fy = ((w1 * F1y + w2 * F2y) / wsum);

//uncomment for damping
  // Fx = ((w1 * F1x + w2 * F2x) / wsum) - B_WALL * vx_filt;
  // Fy = ((w1 * F1y + w2 * F2y) / wsum) - B_WALL * vy_filt;


// ============================================================
// PROXY-BASED FORCE COMPUTATION
// ============================================================
// The proxy is a virtual point that:
//   - Follows the pen freely when INSIDE the shape
//   - Gets stuck on the nearest boundary point when OUTSIDE
// Force = spring between proxy and actual pen position
// This prevents "jumping through" thin features like the handle

void computeForce(float xh, float yh, float &Fx, float &Fy) {

  if (!pointInPolygon(xh, yh)) {
    // OUTSIDE: free space, proxy follows pen, no force
    proxy_x = xh;
    proxy_y = yh;
    Fx = 0.0f;
    Fy = 0.0f;
    return;
  }

  // INSIDE: proxy gets stuck on nearest boundary point
  float nx1, ny1, d2_1, nx2, ny2, d2_2;
  findTwoNearestOnPolygon(xh, yh, nx1, ny1, d2_1, nx2, ny2, d2_2);

  // proxy snaps to nearest wall point
  proxy_x = nx1;
  proxy_y = ny1;

  // force pushes pen toward the wall (outward)
  Fx = K_WALL * (proxy_x - xh);
  Fy = K_WALL * (proxy_y - yh);
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
  // 1. Read BOTH encoders locally — synchronous, no comms latency.
  float theta1 = readEncoderTheta(sensorPos_M1, enc_M1, ENC_M_M1, ENC_B_M1);
  float theta5 = readEncoderTheta(sensorPos_M5, enc_M5, ENC_M_M5, ENC_B_M5);

  // 2. Compute pen-tip position from both thetas.
  float xh, yh;
  getPenTipPosition(xh, yh, theta1, theta5);

  // // 3. Update velocity (must be AFTER position is computed)
  // float vx = (xh - xh_prev) / DT_LOOP;
  // float vy = (yh - yh_prev) / DT_LOOP;
  // vx_filt = 0.1f * vx_filt + 0.9f * vx;
  // vy_filt = 0.1f * vy_filt + 0.9f * vy;
  // xh_prev = xh;
  // yh_prev = yh;

  // 3. Compute Cartesian force.
  float Fx, Fy;
  computeForce(xh, yh, Fx, Fy);

  // 4. Compute torque + PWM for EACH motor.
  float tau_M1 = 0.0f, duty_M1 = 0.0f;
  float tau_M5 = 0.0f, duty_M5 = 0.0f;
  int  pwm_M1 = 0,    pwm_M5 = 0;
  bool dir_M1 = false, dir_M5 = false;

  pwm_M1 = computeMotorOutputPWM(Fx, Fy, theta1, theta5, 1, tau_M1, duty_M1, dir_M1);
  pwm_M5 = computeMotorOutputPWM(Fx, Fy, theta1, theta5, 5, tau_M5, duty_M5, dir_M5);

  // 5. Apply forces to BOTH motors. Any USB Serial input kills both.
  if (FORCE_OUTPUT_ENABLED && !force_killed && !motor_killed && !motor_test_started) {
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      force_killed = true;
      analogWrite(PWM_PIN_M5, 0);
      analogWrite(PWM_PIN_M1, 0);
      digitalWrite(DIR_PIN_M5, LOW);
      digitalWrite(DIR_PIN_M1, LOW);
      Serial.println("# FORCE OUTPUT KILLED by serial input.");
    } else {
      digitalWrite(DIR_PIN_M5, dir_M5 ? LOW : HIGH);
      digitalWrite(DIR_PIN_M1, dir_M1 ? LOW : HIGH);
      analogWrite(PWM_PIN_M5, pwm_M5);
      analogWrite(PWM_PIN_M1, pwm_M1);
    }
  } else {
    analogWrite(PWM_PIN_M5, 0);
    analogWrite(PWM_PIN_M1, 0);
  }

  // 6. CSV output for Processing GUI — same format as before.
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

// ============================================================
// ME327 Team 10 - Pantograph Position Only
// Stripped to: encoder reading, inter-board serial, FK, print x/y
// ============================================================

#include <math.h>
#include <SoftwareSerial.h>

// ============================================================
// BOARD ROLE — comment/uncomment ONE of these
// ============================================================
#define IS_MOTOR_1    // Board 1 (left motor)
// (comment out above for Board 5, right motor)

// ============================================================
// LINK LENGTHS [meters]
// ============================================================
const float LINK_A1 = 0.10f;
const float LINK_A2 = 0.10f;
const float LINK_A3 = 0.10f;
const float LINK_A4 = 0.10f;
const float LINK_A5 = 0.00f;  // motor pivots are coincident

// ============================================================
// PIN ASSIGNMENTS
// ============================================================
const int sensorPosPin  = A2;   // MR encoder
const int PWM_PIN_LOCAL = 5;    // motor PWM (kept off the whole time)
const int DIR_PIN_LOCAL = 8;    // motor direction
const int LINK_TX_PIN   = 9;    // SoftwareSerial TX to partner
const int LINK_RX_PIN   = 10;   // SoftwareSerial RX from partner

SoftwareSerial linkSerial(LINK_RX_PIN, LINK_TX_PIN);

// ============================================================
// ENCODER CALIBRATION
// ============================================================
// Board 1:  ENC_M = -0.0222,  ENC_B = 0.0
// Board 5:  ENC_M = -0.0227,  ENC_B = 95.5
// Measured from:
//   Board 1 — theta=0°  at updatedPos=1,    theta=90°  at updatedPos=-4058
//   Board 5 — theta=90° at updatedPos=258,  theta=180° at updatedPos=-3708
#ifdef IS_MOTOR_1
  const float ENC_M = -0.0222f;
  const float ENC_B =  0.0f;
#else
  const float ENC_M = -0.0227f;
  const float ENC_B =  95.5f;
#endif

const float SECTOR_GEAR_REDUCTION = 1.0f;

// ============================================================
// MOTOR TEST MODE (step 4 — temporary)
// ============================================================
// When true: motor twitches gently in alternating directions, then
// auto-stops after MOTOR_TEST_MAX_FLIPS. Send any character over USB
// Serial to KILL motor immediately at any time.
//
// Hapkit board pin mapping confirms:
//   D5 = PWM Output for Motor 1
//   D8 = Direction Output for Motor 1
// (Per ME327 Hapkit Pin Mapping doc, 11.14.2013)
const bool MOTOR_TEST_ENABLED      = false;
const int  MOTOR_TEST_PWM          = 100;      // 0-255, ~24% duty (with setPwmFrequency = smooth)
// NOTE: millis() runs 64x fast because setPwmFrequency(5,1) was called.
// 64000 here actually means ~1 second of real wall-clock time.
const unsigned long MOTOR_TEST_PERIOD_MS = 64000;   // ~1s real time
const unsigned long MOTOR_TEST_ON_MS     = 6400;    // ~100ms real time motor energized
const int  MOTOR_TEST_MAX_FLIPS    = 4;       // only 4 flips at this PWM level
bool motor_killed = false;
bool motor_test_started = false;   // gate — set to true only after 'g' received
int  motor_flip_count = 0;
// Step 5b: kill switch for FORCE_OUTPUT path. Distinct from motor_killed
// (which is for the twitch test). Set true if any USB Serial input arrives
// during force feedback.
bool force_killed = false;

// ============================================================
// FORCE MODEL — polygon wall test (step 3)
// ============================================================
// Shape is now a closed polygon, stored as an array of (x, y) vertices
// in physical workspace coordinates. The polygon is implicitly closed
// (last vertex connects back to first). To swap shapes later, just
// change SHAPE_PTS and SHAPE_N.
//
// Currently: same 6cm x 6cm square as step 2, centered at (0, 0.10).
const float K_WALL = 30.0f;  // spring stiffness [N/m]

// ============================================================
// MOTOR DRIVE CONSTANTS (step 5)
// ============================================================
// Torque -> duty cycle: duty = sqrt(|tau| / TORQUE_TO_DUTY_K)
// where duty ∈ [0,1] and output = duty * 255 (PWM 0..255).
// Constant 0.0183 lifted from A3/A4 Hapkit templates.
const float TORQUE_TO_DUTY_K = 0.0183f;

// Safety cap on PWM output — never exceed this even if math says higher.
// 153 = 60% of 255, matches Hapkit motor thermal limit.
const int   PWM_OUTPUT_CAP   = 153;

// Numerical Jacobian: perturbation size for finite-difference (radians)
const float JAC_PERTURB      = 1e-4f;

// MASTER SAFETY SWITCH for motor force output.
// Step 5a: keep FALSE — motor stays OFF, only print torque/duty for inspection.
// Step 5b: flip to TRUE after we trust the numbers.
const bool  FORCE_OUTPUT_ENABLED = true;

const int   SHAPE_N = 4;
const float SHAPE_PTS[SHAPE_N][2] = {
  { -0.03f, 0.07f },   // bottom-left
  {  0.03f, 0.07f },   // bottom-right
  {  0.03f, 0.13f },   // top-right
  { -0.03f, 0.13f }    // top-left
};

// ============================================================
// ENCODER STATE
// ============================================================
int rawPos = 0, lastRawPos = 0, lastLastRawPos = 0;
int rawDiff = 0, lastRawDiff = 0;
int rawOffset = 0, lastRawOffset = 0;
int flipNumber = 0, tempOffset = 0;
bool flipped = false;
const int flipThresh = 700;
int updatedPos = 0;

// ============================================================
// INTER-BOARD STATE
// ============================================================
// Partner's theta, received over SoftwareSerial.
// Initialized to a sane mid-workspace value so FK has a
// reasonable starting point before the first packet arrives.
#ifdef IS_MOTOR_1
  float theta_partner_rad = PI / 2.0f;   // board 5 starts near 90 deg
#else
  float theta_partner_rad = 0.0f;         // board 1 starts near 0 deg
#endif

bool partner_received = false;   // don't run FK until we have real data

// Last valid pen-tip position (used when FK fails)
float xh_persist = 0.0f, yh_persist = 0.0f;

// ============================================================
// POLYGON GEOMETRY HELPERS
// ============================================================

// Returns squared distance from point (px, py) to the line segment
// from (ax, ay) to (bx, by). Also writes the nearest point on the
// segment into (nx, ny).
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
    // Degenerate (zero-length) segment: nearest point is endpoint A.
    nx = ax; ny = ay;
    float dx = px - ax, dy = py - ay;
    return dx*dx + dy*dy;
  }

  // Project w onto v, clamp to [0,1] to stay on the segment.
  float t = (wx*vx + wy*vy) / seg_len2;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;

  nx = ax + t * vx;
  ny = ay + t * vy;
  float dx = px - nx, dy = py - ny;
  return dx*dx + dy*dy;
}

// Find the closest point on the polygon to (px, py).
// Writes the nearest point into (nx, ny).
// Returns the squared distance.
float findNearestPointOnPolygon(float px, float py, float &nx, float &ny) {
  float best_d2 = 1e30f;
  float best_nx = px, best_ny = py;

  for (int i = 0; i < SHAPE_N; i++) {
    int j = (i + 1) % SHAPE_N;   // wraps last->first
    float seg_nx, seg_ny;
    float d2 = distSqToSegment(px, py,
                               SHAPE_PTS[i][0], SHAPE_PTS[i][1],
                               SHAPE_PTS[j][0], SHAPE_PTS[j][1],
                               seg_nx, seg_ny);
    if (d2 < best_d2) {
      best_d2 = d2;
      best_nx = seg_nx;
      best_ny = seg_ny;
    }
  }

  nx = best_nx;
  ny = best_ny;
  return best_d2;
}

// Ray-casting inside/outside test. Returns true if (px, py) is
// inside the polygon. Robust against horizontal-edge edge cases.
bool pointInPolygon(float px, float py) {
  bool inside = false;
  for (int i = 0, j = SHAPE_N - 1; i < SHAPE_N; j = i++) {
    float xi = SHAPE_PTS[i][0], yi = SHAPE_PTS[i][1];
    float xj = SHAPE_PTS[j][0], yj = SHAPE_PTS[j][1];

    // Does the horizontal ray from (px, py) going +x cross edge i-j?
    bool crosses = ((yi > py) != (yj > py)) &&
                   (px < (xj - xi) * (py - yi) / (yj - yi) + xi);
    if (crosses) inside = !inside;
  }
  return inside;
}

// ============================================================
// FORCE COMPUTATION — square wall (step 2 of N)
// ============================================================
// Pen tip is at (xh, yh) in physical workspace coords.
// Square wall sits at [SQUARE_CX - SQUARE_HALF, SQUARE_CX + SQUARE_HALF]
//                  x [SQUARE_CY - SQUARE_HALF, SQUARE_CY + SQUARE_HALF].
//
// Outside the square: linear spring force pulling pen back toward the
// nearest point on the square boundary.
// Inside the square: no force (we'll add the "wall" behavior later
// when we have a closed polygon with inside/outside detection).
//
// This is the placeholder. Later we'll replace it with a real
// polygon-based force model (nearest-segment + ray-casting).
void computeForce(float xh, float yh, float &Fx, float &Fy) {
  // Inside the polygon? No force (this is the "interior" — pen moves freely).
  if (pointInPolygon(xh, yh)) {
    Fx = 0.0f;
    Fy = 0.0f;
    return;
  }

  // Outside: find the nearest point on the polygon boundary,
  // then apply spring force pulling pen back toward it.
  float nx, ny;
  findNearestPointOnPolygon(xh, yh, nx, ny);
  Fx = K_WALL * (nx - xh);
  Fy = K_WALL * (ny - yh);
}

// ============================================================
// JACOBIAN + MOTOR OUTPUT (step 5)
// ============================================================
// Given Cartesian force (Fx, Fy) and current joint angles (theta1, theta5),
// compute the torque this board's motor needs to produce, then convert to
// PWM duty cycle.
//
// Uses NUMERICAL Jacobian (finite difference) — perturb our own theta,
// re-run FK, observe (dxh, dyh). The column of J^T for our motor is then:
//   [dxh/dtheta_self, dyh/dtheta_self]
// And tau_self = (dxh/dtheta_self) * Fx + (dyh/dtheta_self) * Fy.
//
// Returns the PWM output (0..255, capped at PWM_OUTPUT_CAP).
// Always writes Tx_out and duty_out for telemetry, even if motor disabled.
// ============================================================
// JACOBIAN + MOTOR OUTPUT (step 5)
// ============================================================
// Given Cartesian force (Fx, Fy) and current joint angles (theta1, theta5),
// compute the torque this board's motor needs to produce, then convert to
// PWM duty cycle. Also determines motor direction from torque sign.
//
// Uses NUMERICAL Jacobian (finite difference) — perturb our own theta,
// re-run FK, observe (dxh, dyh). The column of J^T for our motor is then:
//   [dxh/dtheta_self, dyh/dtheta_self]
// And tau_self = (dxh/dtheta_self) * Fx + (dyh/dtheta_self) * Fy.
//
// Returns the PWM output (0..PWM_OUTPUT_CAP).
// Also sets dir_out: true = DIR pin HIGH, false = DIR pin LOW.
// (Which sign means which physical direction will be calibrated on first test.)
int computeMotorOutputPWM(float Fx, float Fy,
                          float theta1, float theta5,
                          float &tau_out, float &duty_out,
                          bool &dir_out) {
  // Baseline FK at current angles
  float x0, y0;
  if (!computeFwdKin(theta1, theta5, x0, y0)) {
    tau_out = 0.0f;
    duty_out = 0.0f;
    dir_out = false;
    return 0;
  }

  float x_p, y_p;
#ifdef IS_MOTOR_1
  if (!computeFwdKin(theta1 + JAC_PERTURB, theta5, x_p, y_p)) {
    tau_out = 0.0f; duty_out = 0.0f; dir_out = false; return 0;
  }
#else
  if (!computeFwdKin(theta1, theta5 + JAC_PERTURB, x_p, y_p)) {
    tau_out = 0.0f; duty_out = 0.0f; dir_out = false; return 0;
  }
#endif

  float dxh_dth = (x_p - x0) / JAC_PERTURB;
  float dyh_dth = (y_p - y0) / JAC_PERTURB;

  float tau = dxh_dth * Fx + dyh_dth * Fy;
  tau_out = tau;

  // Direction from sign of tau. Convention TBD on first test — if motor
  // pushes the WRONG way, we'll invert this bool.
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
  linkSerial.begin(19200);
  linkSerial.setTimeout(10);

  // High PWM frequency for smooth motor torque (matches A3/A4 Hapkit templates).
  // CAUTION: this changes Timer 0 prescaler /64 -> /1, so millis(), micros(),
  // and delay() return values 64x LARGER than real wall-clock time.
  // delayMicroseconds() is NOT affected.
  setPwmFrequency(PWM_PIN_LOCAL, 1);

  // Motor pins — initialize to off
  pinMode(PWM_PIN_LOCAL, OUTPUT);
  pinMode(DIR_PIN_LOCAL, OUTPUT);
  analogWrite(PWM_PIN_LOCAL, 0);
  digitalWrite(DIR_PIN_LOCAL, LOW);

  pinMode(sensorPosPin, INPUT);

  #ifndef IS_MOTOR_1
    Serial.println("Pantograph position-only firmware ready.");
    Serial.println("Role: MOTOR 5 (theta5 = own encoder)");
  #endif

if (MOTOR_TEST_ENABLED) {
    Serial.println("# MOTOR TEST MODE ARMED — motor is OFF.");
    Serial.println("# Send 'g' to START twitch test.");
    Serial.println("# Send ANY OTHER character to KILL motor if running.");
    Serial.print("# Will auto-stop after ");
    Serial.print(MOTOR_TEST_MAX_FLIPS);
    Serial.println(" direction flips (~10 seconds).");
  }

}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  // 1. Read own encoder once per loop (important: only once, 
  //    calling it multiple times per loop confuses the flip detector)
  float theta_self = readThisBoardTheta();

  // 2. Send own theta to partner, receive partner's theta
  handleSerialLink(theta_self);

  // 3. Compute pen-tip position (only if we have real data from partner)
  float xh, yh;
  if (partner_received) {
    getPenTipPosition(xh, yh, theta_self);
  } else {
    xh = 0.0f;
    yh = 0.0f;
  }

  // 3b. Compute force (display only — motors are NOT driven this step)
  float Fx, Fy;
  computeForce(xh, yh, Fx, Fy);

  // 3c. Compute torque + PWM from force (Jacobian transpose).
  float tau_out = 0.0f, duty_out = 0.0f;
  int  pwm_out = 0;
  bool dir_out = false;
  if (partner_received) {
  #ifdef IS_MOTOR_1
    float t1 = theta_self;
    float t5 = theta_partner_rad;
  #else
    float t1 = theta_partner_rad;
    float t5 = theta_self;
  #endif
    pwm_out = computeMotorOutputPWM(Fx, Fy, t1, t5, tau_out, duty_out, dir_out);
  }

  // 3d. Apply force to motor — ONLY if all safety gates pass.
  //     ANY USB Serial input (while not in twitch test) kills force output.
  if (FORCE_OUTPUT_ENABLED && !force_killed && !motor_killed && !motor_test_started) {
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      force_killed = true;
      analogWrite(PWM_PIN_LOCAL, 0);
      digitalWrite(DIR_PIN_LOCAL, LOW);
      Serial.println("# FORCE OUTPUT KILLED by serial input.");
    } else {
      digitalWrite(DIR_PIN_LOCAL, dir_out ? HIGH : LOW);
      analogWrite(PWM_PIN_LOCAL, pwm_out);
    }
  } else {
    // Force output disabled or killed — make sure motor is off
    analogWrite(PWM_PIN_LOCAL, 0);
  }

  #ifndef IS_MOTOR_1
    // 4a. CSV output for Processing GUI — every loop, format: "xh,yh,Fx,Fy\n"
    //     Only Board 5 prints; Board 1 stays silent on USB.
    Serial.print(xh, 4);
    Serial.print(',');
    Serial.print(yh, 4);
    Serial.print(',');
    Serial.print(Fx, 3);
    Serial.print(',');
    Serial.println(Fy, 3);

    // 4b. Human-readable debug every 50 loops (GUI will ignore these as bad packets)
    static int printCounter = 0;
    if (++printCounter >= 50) {
      float theta_self_deg  = ENC_M * updatedPos + ENC_B;
      float theta_partner_deg = theta_partner_rad * (180.0f / PI);

      Serial.print("# updatedPos=");    Serial.print(updatedPos);
      Serial.print("  self_deg=");      Serial.print(theta_self_deg, 1);
      Serial.print("  partner_deg=");   Serial.print(theta_partner_deg, 1);
      Serial.print("  partner_rcvd=");  Serial.print(partner_received ? "YES" : "NO ");
      Serial.print("  xh=");            Serial.print(xh, 4);
      Serial.print("  yh=");            Serial.print(yh, 4);
      Serial.print("  Fx=");            Serial.print(Fx, 3);
      Serial.print("  Fy=");            Serial.print(Fy, 3);
      Serial.print("  tau=");           Serial.print(tau_out, 5);
      Serial.print("  duty=");          Serial.print(duty_out, 3);
      Serial.print("  pwm=");           Serial.print(pwm_out);
      Serial.println();
      printCounter = 0;
    }
  #endif

  // 5. MOTOR TEST (step 4) — gated by serial 'g' trigger.
  //    'g'  -> START the twitch test (only if not already running/killed)
  //    any other char -> KILL motor immediately
  if (Serial.available()) {
    char c = Serial.read();
    while (Serial.available()) Serial.read();   // drain rest of buffer

    if (c == 'g' && !motor_test_started && !motor_killed) {
      motor_test_started = true;
      Serial.println("# MOTOR TEST STARTING — twitching begins now.");
    } else if (motor_test_started && !motor_killed) {
      // Any character other than the initial 'g' = kill switch
      motor_killed = true;
      analogWrite(PWM_PIN_LOCAL, 0);
      digitalWrite(DIR_PIN_LOCAL, LOW);
      Serial.println("# MOTOR KILLED by serial input.");
    }
  }

  if (MOTOR_TEST_ENABLED && motor_test_started && !motor_killed) {
    if (motor_flip_count >= MOTOR_TEST_MAX_FLIPS) {
      analogWrite(PWM_PIN_LOCAL, 0);
      digitalWrite(DIR_PIN_LOCAL, LOW);
      if (motor_flip_count == MOTOR_TEST_MAX_FLIPS) {
        Serial.println("# MOTOR TEST COMPLETE — auto-stopped after MAX_FLIPS.");
        motor_flip_count++;
      }
    } else {
      static unsigned long lastFlip = 0;
      static bool dir = false;
      static bool pulseActive = false;
      unsigned long now = millis();

      // Time to start a new flip?
      if (now - lastFlip > MOTOR_TEST_PERIOD_MS) {
        dir = !dir;
        digitalWrite(DIR_PIN_LOCAL, dir ? HIGH : LOW);
        analogWrite(PWM_PIN_LOCAL, MOTOR_TEST_PWM);
        lastFlip = now;
        pulseActive = true;
        motor_flip_count++;
        Serial.print("# Flip ");
        Serial.print(motor_flip_count);
        Serial.print("/");
        Serial.print(MOTOR_TEST_MAX_FLIPS);
        Serial.print(" — dir=");
        Serial.print(dir ? "HIGH" : "LOW");
        Serial.print(" PWM=");
        Serial.println(MOTOR_TEST_PWM);
      }

      // Pulse off after ON duration
      if (pulseActive && (now - lastFlip > MOTOR_TEST_ON_MS)) {
        analogWrite(PWM_PIN_LOCAL, 0);
        pulseActive = false;
      }
    }
  }
}

// ============================================================
// ENCODER READING
// ============================================================
float readThisBoardTheta() {
  rawPos       = analogRead(sensorPosPin);
  rawDiff      = rawPos - lastRawPos;
  lastRawDiff  = rawPos - lastLastRawPos;
  rawOffset    = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);
  lastLastRawPos = lastRawPos;
  lastRawPos   = rawPos;

  if ((lastRawOffset > flipThresh) && (!flipped)) {
    if (lastRawDiff > 0) flipNumber--; else flipNumber++;
    if (rawOffset > flipThresh) {
      updatedPos = rawPos + flipNumber * rawOffset;
      tempOffset = rawOffset;
    } else {
      updatedPos = rawPos + flipNumber * lastRawOffset;
      tempOffset = lastRawOffset;
    }
    flipped = true;
  } else {
    updatedPos = rawPos + flipNumber * tempOffset;
    flipped = false;
  }

  float theta_deg = ENC_M * updatedPos + ENC_B;
  float theta_rad = theta_deg * (PI / 180.0f);
  return theta_rad;
}

// ============================================================
// FORWARD KINEMATICS
// ============================================================
bool computeFwdKin(float theta1, float theta5, float &x3, float &y3) {
  float x2 = LINK_A1 * cosf(theta1);
  float y2 = LINK_A1 * sinf(theta1);
  float x4 = -LINK_A5 + LINK_A4 * cosf(theta5);
  float y4 =  LINK_A4 * sinf(theta5);

  float dx = x4 - x2;
  float dy = y4 - y2;
  float dP2P4 = sqrtf(dx*dx + dy*dy);

  if (dP2P4 < 1e-6f) return false;  // singular pose

  float dP2Ph = (LINK_A2*LINK_A2 - LINK_A3*LINK_A3 + dP2P4*dP2P4) / (2.0f * dP2P4);
  float h2    = LINK_A2*LINK_A2 - dP2Ph*dP2Ph;

  if (h2 < 0.0f) return false;  // pose unreachable
  float dP3Ph = sqrtf(h2);

  float xh_pt = x2 + (dP2Ph / dP2P4) * dx;
  float yh_pt = y2 + (dP2Ph / dP2P4) * dy;

  x3 = xh_pt + (dP3Ph / dP2P4) * dy;
  y3 = yh_pt - (dP3Ph / dP2P4) * dx;
  return true;
}

void getPenTipPosition(float &x, float &y, float theta_self) {
#ifdef IS_MOTOR_1
  float theta1 = theta_self;
  float theta5 = theta_partner_rad;
#else
  float theta1 = theta_partner_rad;
  float theta5 = theta_self;
#endif

  float x3, y3;
  if (computeFwdKin(theta1, theta5, x3, y3)) {
    xh_persist = x3;
    yh_persist = y3;
  }
  // If FK failed, reuse last good value (don't snap to 0,0)
  x = xh_persist;
  y = yh_persist;
}

// ============================================================
// INTER-BOARD SERIAL LINK
// ============================================================
// Sends own theta, receives partner's theta.
//
// THE BUG FIX: previously parseFloat() returned 0 on timeout and
// silently overwrote theta_partner_rad. Now we only update it if
// the parsed value is a reasonable angle (between -2*PI and 2*PI,
// roughly -360° to +360°), and we keep the "partner_received" flag
// so FK doesn't run on stale/default data.
void handleSerialLink(float theta_self) {
  // --- Send own theta (rate-limited: every 20 loops) ---
  static int txCounter = 0;
  if (++txCounter >= 5) {
    linkSerial.print("A");
    linkSerial.println(theta_self, 4);
    txCounter = 0;
  }

  // --- Receive partner's theta ---
  while (linkSerial.available()) {
    char c = (char)linkSerial.read();
    if (c == 'A') {
      float val = linkSerial.parseFloat();

      // Only accept values in a plausible angle range (-2pi to +2pi).
      // This guards against parseFloat() returning 0 on timeout.
      if (val > -3.2f && val < 3.2f && val != 0.0f) {
        theta_partner_rad = val;
        partner_received  = true;
      }

      // Consume trailing newline
      while (linkSerial.available() &&
             (linkSerial.peek() == '\n' || linkSerial.peek() == '\r')) {
        linkSerial.read();
      }
    }
  }
}

// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// Copied from A3/A4 Hapkit template.
// CAUTION: calling with pin=5 or 6 (Timer 0) makes millis()/micros()/delay()
// return values 64x larger than real wall-clock time.
// --------------------------------------------------------------
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
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

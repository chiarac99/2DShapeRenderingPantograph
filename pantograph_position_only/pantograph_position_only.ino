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
// SHAPE DATA
// ============================================================
enum ShapeId { SHAPE_BUNNY = 0, SHAPE_HAMMER = 1, SHAPE_PEAR = 2, SHAPE_SQUARE = 3 };
ShapeId currentShape = SHAPE_TEST;     // change here for testing
// note: SHAPE_SQUARE is a tester: four points, a square
const int N_SQUARE   = 4;
const float shape_square[N_SQUARE][2] PROGMEM = {
  {  -0.03f,  -0.03f},
  { 0.03f,  -0.03f},
  { 0.03f,  0.03f},
  {  -0.03f,  0.03f},
  {  -0.03f,  -0.03f}
}
// active shape
const float (*shape)[2] = shape_square;
int N_POINTS = N_SQUARE;
// Read x or y of the i-th vertex of the active shape, from PROGMEM.
inline float shapeX(int i) {
  return pgm_read_float(&shape[i][0]);
}
inline float shapeY(int i) {
  return pgm_read_float(&shape[i][1]);
}

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
// CAPSTAN DRIVE TRANSMISSION  (per Hapkit board)
// ============================================================
const float R_PULLEY  = 0.0045f;    // m, motor capstan/drum radius   (A3: 0.0045)
const float R_SECTOR  = 0.075f;     // m, sector pulley radius         (A3: 0.075)
const float CAPSTAN_RATIO = R_PULLEY / R_SECTOR;
const float TORQUE_TO_DUTY_K = 0.0183f;

// ============================================================
// SETUP
// ============================================================
// PWM frequency divisor helper (copied from A3 template -- DO NOT EDIT).
// Increases PWM frequency on the motor pins to keep motor noise inaudible
// and force-response smooth.
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
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}


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
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  linkSerial.begin(19200);
  linkSerial.setTimeout(10);

  // Configure motor PWM at high frequency (matches A3 template).
  setPwmFrequency(PWM_PIN_LOCAL, 1);

  // Motor pins — keep motor OFF the whole time (position-only mode)
  pinMode(PWM_PIN_LOCAL, OUTPUT);
  pinMode(DIR_PIN_LOCAL, OUTPUT);
  analogWrite(PWM_PIN_LOCAL, 0);
  digitalWrite(DIR_PIN_LOCAL, LOW);
  
  pinMode(sensorPosPin, INPUT);

  setShape(currentShape);  // initializes shape pointer, arc-length table, prints CCW check

  Serial.println("Pantograph position-only firmware ready.");
#ifdef IS_MOTOR_1
  Serial.println("Role: MOTOR 1 (theta1 = own encoder)");
#else
  Serial.println("Role: MOTOR 5 (theta5 = own encoder)");
#endif
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

  // 4. Print telemetry every 50 loops (~50ms at 1kHz, readable in Serial Monitor) -> sends to Processing if motor 1
  #ifdef IS_MOTOR_1
    static int printCounter = 0;
    if (++printCounter >= 50) {
      float theta_self_deg  = ENC_M * updatedPos + ENC_B;
      float theta_partner_deg = theta_partner_rad * (180.0f / PI);

      // Serial.print("updatedPos=");      Serial.print(updatedPos);
      // Serial.print("  self_deg=");      Serial.print(theta_self_deg, 1);
      // Serial.print("  partner_deg=");   Serial.print(theta_partner_deg, 1);
      // Serial.print("  partner_rcvd=");  Serial.print(partner_received ? "YES" : "NO ");
      // Serial.print("  xh=");           Serial.print(xh, 4);
      // Serial.print("  yh=");           Serial.print(yh, 4);
      Serial.print(xh, 4);
      Serial.print(",");
      Serial.print(yh, 4);
      Serial.println();
      printCounter = 0;
    }
  #endif

  // 5. Computer force
  float Fx, Fy;

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
  if (++txCounter >= 20) {
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
      if (val > -7.0f && val < 7.0f && val != 0.0f) {
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

// ============================================================
// SET SHAPE  (runtime switcher)
// ============================================================
// Swaps the active shape pointer + length, then rebuilds the arc-length
// table and verifies winding. Called once in setup() and again whenever
// a shape-switch command arrives over serial.
void setShape(ShapeId s) {
  currentShape = s;
  switch (s) {
    case SHAPE_SQUARE:
      shape = shape_square;
      N_POINTS = N_SQUARE;
      // Serial.println("Active shape: BUNNY");
      break;
  }
  checkWinding();
  toggleLed();
}

// ============================================================
// WINDING CHECK  (warn over serial if CW)
// ============================================================
void checkWinding() {
  float area = signedPolygonArea();
  if (area < 0.0f) {
    Serial.println("!!! WARNING: shape array is CW. Force normals will point");
    Serial.println("!!! INWARD. Reverse the array order in Python (or here)");
    Serial.println("!!! before testing.");
  } else {
    Serial.print("Shape OK (CCW), area = ");
    Serial.print(area, 6);
    Serial.println(" m^2");
  }
}

// ============================================================
// NEAREST SEGMENT SEARCH
// ============================================================
// For each segment (shape[i], shape[i+1])  (wrapping at N-1 -> 0),
// computes the perpendicular distance from (x,y) to that segment,
// the segment's outward unit normal (assuming CCW polygon), and the
// fractional position t in [0,1] along the segment.
//
// Outward normal: for tangent (tx,ty), outward direction is (-ty, tx)
// when the polygon is CCW.
int findNearestSegment(float x, float y, float &dist, float &nx, float &ny,
                       float &t_frac) {
  int best_i = 0;
  float best_dist = 1.0e9f;
  float best_nx = 0.0f, best_ny = 0.0f;
  float best_t = 0.0f;

  for (int i = 0; i < N_POINTS; i++) {
    int j = (i + 1) % N_POINTS;

    float ax = shapeX(i), ay = shapeY(i);
    float bx = shapeX(j), by = shapeY(j);
    float tx = bx - ax;
    float ty = by - ay;
    float seg_len2 = tx*tx + ty*ty;

    float t = ((x - ax)*tx + (y - ay)*ty) / seg_len2;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    float px = ax + t*tx;
    float py = ay + t*ty;
    float ddx = x - px;
    float ddy = y - py;
    float d = sqrtf(ddx*ddx + ddy*ddy);

    if (d < best_dist) {
      best_dist = d;
      best_i = i;
      best_t = t;
      float seg_len = sqrtf(seg_len2);
      // Outward normal for a CCW polygon in math (y-up) coords:
      // rotate tangent -90 deg, i.e. (tx, ty) -> (ty, -tx).
      best_nx =  ty / seg_len;
      best_ny = -tx / seg_len;
    }
  }

  dist = best_dist;
  nx = best_nx;
  ny = best_ny;
  t_frac = best_t;
  return best_i;
}

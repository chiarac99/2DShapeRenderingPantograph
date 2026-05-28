// ============================================================
// ME327 Team 10 - Pantograph Single-Board Firmware
// 4-QUADRANT HAPTIC TRAINING MODE
// ============================================================

#include <math.h>

// ============================================================
// QUADRANT & SQUARE CONFIGURATION (Tinker Here)
// ============================================================
// The workspace center is x = 0.0, y = 0.10. 
// We define 4 squares. Each is 4cm x 4cm (0.04m).

struct QuadrantZone {
  float minX, maxX;
  float minY, maxY;
  float K_WALL;
  float B_WALL;
};

// Tune K_WALL (Stiffness) and B_WALL (Damping) empirically here:
const float low_k = 30;
const float high_k = 50;
const float low_b = 0;
const float high_b = 0.03;
const QuadrantZone QUADRANTS[4] = {
  // Quad 1: Top-Right (x > 0, y > 0.10) -- "Stiff and Sticky"
  { 0.03f, 0.07f,   0.13f, 0.17f,   high_k, high_b},
  
  // Quad 2: Top-Left (x < 0, y > 0.10) -- "Stiff and Slippery"
  {-0.07f, -0.03f,  0.13f, 0.17f,   high_k, low_b},
  
  // Quad 3: Bottom-Left (x < 0, y < 0.10) -- "Squishy and Slippery"
  {-0.07f, -0.03f,  0.03f, 0.07f,   low_k, low_b},
  
  // Quad 4: Bottom-Right (x > 0, y < 0.10) -- "Squishy & Sticky"
  { 0.03f, 0.07f,   0.03f, 0.07f,   low_k, high_b}
};

// ============================================================
// GLOBAL VARIABLES
// ============================================================
float proxy_x = 0.0f;
float proxy_y = 0.0f;

// Last valid pen-tip position (used when FK fails)
float xh_persist = 0.0f, yh_persist = 0.0f;

struct EncoderState {
  int  rawPos, lastRawPos, lastLastRawPos;
  int  rawDiff, lastRawDiff;
  int  rawOffset, lastRawOffset;
  int  flipNumber, tempOffset;
  bool flipped;
  int  updatedPos;
};

EncoderState enc_M1 = {0,0,0,0,0,0,0,0,0,false,0};
EncoderState enc_M5 = {0,0,0,0,0,0,0,0,0,false,0};
const int flipThresh = 700;

// ============================================================
// LINK LENGTHS [meters]
// ============================================================
const float LINK_A1 = 0.10f;
const float LINK_A2 = 0.10f;
const float LINK_A3 = 0.10f;
const float LINK_A4 = 0.10f;
const float LINK_A5 = 0.00f; // motor pivots are coincident

// ============================================================
// PIN ASSIGNMENTS
// ============================================================
const int sensorPos_M1 = A3; // LEFT MR sensor
const int sensorPos_M5 = A2; // RIGHT MR sensor
const int PWM_PIN_M5   = 5;  // RIGHT motor PWM
const int DIR_PIN_M5   = 8;  // RIGHT motor DIR
const int PWM_PIN_M1   = 6;  // LEFT motor PWM
const int DIR_PIN_M1   = 7;  // LEFT motor DIR

// ============================================================
// ENCODER CALIBRATION
// ============================================================
const float ENC_M_M1 = -0.0222f;
const float ENC_B_M1 =  0.0f;
const float ENC_M_M5 = -0.0227f;
const float ENC_B_M5 =  95.5f;

// ============================================================
// VELOCITY TRACKING 
// ============================================================
float xh_prev = 0.0f, yh_prev = 0.0f;
float vx_filt = 0.0f, vy_filt = 0.0f;
const float DT_LOOP = 0.001f;

// ============================================================
// MOTOR DRIVE CONSTANTS
// ============================================================
const float TORQUE_TO_DUTY_K = 0.0183f;
const int   PWM_OUTPUT_CAP   = 153; // 60% of 255
const float JAC_PERTURB      = 1e-4f;
const bool  FORCE_OUTPUT_ENABLED = true;

void computeForce(float xh, float yh, float &Fx, float &Fy) {
  Fx = 0.0f;
  Fy = 0.0f;

  // Find quadrant using RAW physical coordinates
  int qIndex = -1;
  if (xh >  0.0f && yh >  0.10f) qIndex = 0; 
  if (xh <= 0.0f && yh >  0.10f) qIndex = 1; 
  if (xh <= 0.0f && yh <= 0.10f) qIndex = 2; 
  if (xh >  0.0f && yh <= 0.10f) qIndex = 3; 

  if (qIndex == -1) return;

  QuadrantZone q = QUADRANTS[qIndex];

  // AABB Collision in RAW coordinates
  if (xh > q.minX && xh < q.maxX && yh > q.minY && yh < q.maxY) {
    float d_left   = xh - q.minX;
    float d_right  = q.maxX - xh;
    float d_bottom = yh - q.minY;
    float d_top    = q.maxY - yh;
    
    float min_d = d_left;
    proxy_x = q.minX; 
    proxy_y = yh;
    
    if (d_right < min_d)  { min_d = d_right;  proxy_x = q.maxX; proxy_y = yh; }
    if (d_bottom < min_d) { min_d = d_bottom; proxy_x = xh;     proxy_y = q.minY; }
    if (d_top < min_d)    { min_d = d_top;    proxy_x = xh;     proxy_y = q.maxY; }

    Fx = q.K_WALL * (proxy_x - xh) - q.B_WALL * vx_filt;
    Fy = q.K_WALL * (proxy_y - yh) - q.B_WALL * vy_filt;
  } else {
    proxy_x = xh;
    proxy_y = yh;
  }
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
// ENCODER READING
// ============================================================
float readEncoderTheta(int pin, EncoderState &st, float m, float b) {
  st.rawPos = analogRead(pin);
  if (st.rawPos == st.lastRawPos) {
    return m * st.updatedPos + b; 
  }

  st.rawDiff = st.rawPos - st.lastRawPos;
  if (st.rawDiff < -flipThresh) st.flipped = true;
  if (st.rawDiff > flipThresh)  st.flipped = true;

  if (st.lastRawDiff > 0 && st.rawDiff < -flipThresh) {
    st.flipNumber++;
    st.rawOffset = st.flipNumber * 1024;
  } else if (st.lastRawDiff < 0 && st.rawDiff > flipThresh) {
    st.flipNumber--;
    st.rawOffset = st.flipNumber * 1024;
  }

  st.updatedPos = st.rawPos + st.tempOffset + st.rawOffset;

  st.lastLastRawPos = st.lastRawPos;
  st.lastRawPos = st.rawPos;
  st.lastRawDiff = st.rawPos - st.lastLastRawPos;

  return (m * st.updatedPos + b) * PI / 180.0f; 
}

// ============================================================
// JACOBIAN + MOTOR OUTPUT
// ============================================================
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

  setPwmFrequency(PWM_PIN_M5, 1);
  setPwmFrequency(PWM_PIN_M1, 1);  

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

  Serial.println("Pantograph 4-Quadrant Training Mode ready.");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  // 1. Read Encoders
  float theta1 = readEncoderTheta(sensorPos_M1, enc_M1, ENC_M_M1, ENC_B_M1);
  float theta5 = readEncoderTheta(sensorPos_M5, enc_M5, ENC_M_M5, ENC_B_M5);

  // 2. Forward Kinematics
  float xh, yh;
  getPenTipPosition(xh, yh, theta1, theta5);

  // 3. Velocity Filter
  float vx = (xh - xh_prev) / DT_LOOP;
  float vy = (yh - yh_prev) / DT_LOOP;
  vx_filt = 0.1f * vx_filt + 0.9f * vx;
  vy_filt = 0.1f * vy_filt + 0.9f * vy;
  xh_prev = xh;
  yh_prev = yh;

  // 4. Compute Force
  float Fx, Fy;
  computeForce(xh, yh, Fx, Fy);

  // 5. Compute Torques/PWM
  float tau_M1 = 0.0f, duty_M1 = 0.0f;
  float tau_M5 = 0.0f, duty_M5 = 0.0f;
  int  pwm_M1 = 0,    pwm_M5 = 0;
  bool dir_M1 = false, dir_M5 = false;
  pwm_M1 = computeMotorOutputPWM(Fx, Fy, theta1, theta5, 1, tau_M1, duty_M1, dir_M1);
  pwm_M5 = computeMotorOutputPWM(Fx, Fy, theta1, theta5, 5, tau_M5, duty_M5, dir_M5);

  // 6. Apply Forces
  if (FORCE_OUTPUT_ENABLED) {
    digitalWrite(DIR_PIN_M5, dir_M5 ? LOW : HIGH);
    digitalWrite(DIR_PIN_M1, dir_M1 ? LOW : HIGH);
    analogWrite(PWM_PIN_M5, pwm_M5);
    analogWrite(PWM_PIN_M1, pwm_M1);
  } else {
      analogWrite(PWM_PIN_M5, 0);
      analogWrite(PWM_PIN_M1, 0);
  }
  // 7. Data Output for Processing Gui
  static int csvCounter = 0;
  if (++csvCounter >= 5) {
    Serial.print(xh, 4); Serial.print(',');
    Serial.print(yh, 4); Serial.print(',');
    Serial.print(Fx, 3); Serial.print(',');
    Serial.println(Fy, 3);
    csvCounter = 0;
  }

  // 8. Human-Readable Debug
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
  // delayMicroseconds(1000);
}

// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
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
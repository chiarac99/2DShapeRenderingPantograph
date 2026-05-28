import processing.serial.*;

// ---------- MODE SELECTION ----------
boolean STANDALONE_MODE = false;
Serial[] arduinoPorts = new Serial[0];

// ---------- SERIAL ----------
Serial arduinoPort = null;
float arduinoFx = 0, arduinoFy = 0;   
float arduinoForceX = 0, arduinoForceY = 0;

// ---------- PHYSICAL <-> SCREEN MAPPING ----------
float PIXELS_PER_METER = 4000.0f;
float ORIGIN_X, ORIGIN_Y;
float WORKSPACE_CENTER_X = 0.0;
float WORKSPACE_CENTER_Y = 0.1; 

// ---------- QUADRANT DATA ----------
class QuadZone {
  float minX, maxX, minY, maxY;
  String label;
  color fillColor;
  int strokeStyle; // 0=Solid, 1=Long Dash, 2=Short Dash, 3=Dotted

  QuadZone(float mx, float Mx, float my, float My, String l, color c, int s) {
    minX = mx; maxX = Mx; minY = my; maxY = My;
    label = l; fillColor = c; strokeStyle = s;
  }
}

QuadZone[] zones = new QuadZone[4];

void setup() {
  size(900, 900);
  ORIGIN_X = width / 2.0f;
  ORIGIN_Y = height / 2.0f;
  
  // Define zones matching the Arduino parameters from the firmware
  
  // Q1 (Top Right): K=100, B=2.0 -> high stiff, high damp
  zones[0] = new QuadZone(0.03f, 0.07f, 0.13f, 0.17f, "Stiff and Sticky", color(50, 100, 200, 200), 2);
  
  // Q2 (Top Left): K=150, B=3.0 -> High stiff, low damp
  zones[1] = new QuadZone(-0.07f, -0.03f, 0.13f, 0.17f, "Stiff and Slippery", color(50, 100, 200, 200), 0);
  
  // Q3 (Bottom Left): K=200, B=4.0 -> low stiff, low damp
  zones[2] = new QuadZone(-0.07f, -0.03f, 0.03f, 0.07f, "Squishy and Slippery", color(173, 216, 230, 200), 0);
  
  // Q4 (Bottom Right): K=50, B=1.0 -> Low Stiff, high damp
  zones[3] = new QuadZone(0.03f, 0.07f, 0.03f, 0.07f, "Squishy & Sticky", color(173, 216, 230, 200), 2);

  setupSerial();
}

void draw() {
  background(255);

  float xh = (arduinoFx - WORKSPACE_CENTER_X);
  float yh = (arduinoFy - WORKSPACE_CENTER_Y);

  // drawAxes();
  drawQuadrants();
  drawPenTip(xh, yh);
  drawForceVector(xh, yh);

  // HUD
  fill(80);
  textAlign(LEFT, TOP);
  textSize(14);
  text("Haptic texture sampler", 10, 10);
  text("pen: (" + nf(arduinoFx, 1, 4) + ", " + nf(arduinoFy, 1, 4) + ") m", 10, 30);
  text("force: (" + nf(arduinoForceX, 1, 3) + ", " + nf(arduinoForceY, 1, 3) + ") N", 10, 50);
}

// =============================================================
// VISUAL RENDERING
// =============================================================

float meterToPixelX(float xm) { return ORIGIN_X + xm * PIXELS_PER_METER; }
float meterToPixelY(float ym) { return ORIGIN_Y - ym * PIXELS_PER_METER; }

void drawAxes() {
  stroke(200);
  strokeWeight(1);
  line(meterToPixelX(0), 0, meterToPixelX(0), height);
  line(0, meterToPixelY(0), width, meterToPixelY(0));
}

void drawQuadrants() {
  strokeWeight(4); // Increased stroke width globally
  
  for (int i = 0; i < zones.length; i++) {
    QuadZone q = zones[i];
    
    float xm_min = q.minX - WORKSPACE_CENTER_X;
    float xm_max = q.maxX - WORKSPACE_CENTER_X;
    float ym_min = q.minY - WORKSPACE_CENTER_Y;
    float ym_max = q.maxY - WORKSPACE_CENTER_Y;

    float px1 = meterToPixelX(xm_min);
    float py1 = meterToPixelY(ym_min);
    float px2 = meterToPixelX(xm_max);
    float py2 = meterToPixelY(ym_max);

    float w = abs(px2 - px1);
    float h = abs(py2 - py1);
    float topX = min(px1, px2);
    float topY = min(py1, py2);

    // Draw Shape Fill based on Stiffness
    noStroke();
    fill(q.fillColor);
    rect(topX, topY, w, h);
    
    // Draw Styled Stroke based on Damping
    stroke(50);
    drawStyledRect(topX, topY, w, h, q.strokeStyle);

    // Draw Text Label
    fill(80);
    textAlign(CENTER, BOTTOM);
    text(q.label, topX + w/2, topY - 10);
  }
}

// Custom function to draw dashed borders robustly without relying on Java2D limits
void drawStyledRect(float x, float y, float w, float h, int style) {
  if (style == 0) {
    noFill();
    rect(x, y, w, h);
    return;
  }
  
  float[] pattern;
  if (style == 1) pattern = new float[]{25, 15};       // Long Dash
  else if (style == 2) pattern = new float[]{10, 10};  // Short Dash
  else pattern = new float[]{3, 8};                    // Dotted (Style 3)
  
  drawDashedLine(x, y, x+w, y, pattern);
  drawDashedLine(x+w, y, x+w, y+h, pattern);
  drawDashedLine(x+w, y+h, x, y+h, pattern);
  drawDashedLine(x, y+h, x, y, pattern);
}

void drawDashedLine(float x1, float y1, float x2, float y2, float[] pattern) {
  float d = dist(x1, y1, x2, y2);
  float nx = (x2 - x1) / d;
  float ny = (y2 - y1) / d;
  float p = 0;
  int i = 0;
  
  while (p < d) {
    float step = min(pattern[i % pattern.length], d - p);
    if (i % 2 == 0) {
      line(x1 + nx * p, y1 + ny * p, x1 + nx * (p + step), y1 + ny * (p + step));
    }
    p += step;
    i++;
  }
}

void drawPenTip(float xh, float yh) {
  noStroke();
  fill(255, 100, 150);
  ellipse(meterToPixelX(xh), meterToPixelY(yh), 12, 12);
}

void drawForceVector(float xh_screen, float yh_screen) {
  float fmag = sqrt(arduinoForceX*arduinoForceX + arduinoForceY*arduinoForceY);
  if (fmag < 1e-4f) return;

  float SCALE_PX_PER_N = 50.0f;
  float startX = meterToPixelX(xh_screen);
  float startY = meterToPixelY(yh_screen);
  float endX = startX - arduinoForceX * SCALE_PX_PER_N;
  float endY = startY + arduinoForceY * SCALE_PX_PER_N;

  stroke(255, 50, 50, 200);
  strokeWeight(3);
  line(startX, startY, endX, endY);

  float ang = atan2(endY - startY, endX - startX);
  float ah = 8;
  line(endX, endY, endX - ah*cos(ang - PI/6), endY - ah*sin(ang - PI/6));
  line(endX, endY, endX - ah*cos(ang + PI/6), endY - ah*sin(ang + PI/6));
}

// =============================================================
// SERIAL
// =============================================================
void setupSerial() {
  if (STANDALONE_MODE) return;
  String BOARD5_PORT = "/dev/tty.usbserial-A10POSFY"; 
  try {
    Serial board5 = new Serial(this, BOARD5_PORT, 115200);
    arduinoPorts = new Serial[]{ board5 };
    println("Connected to Board 5 on " + BOARD5_PORT);
  } catch (Exception e) {
    println("Failed to connect: " + e.getMessage());
    STANDALONE_MODE = true;
  }
}

void serialEvent(Serial p) {
  String raw = p.readStringUntil('\n');
  if (raw == null) return;
  raw = raw.trim();

  String[] parts = splitTokens(raw, ", \t");
  if (parts.length >= 4) {
    try {
      arduinoFx = Float.parseFloat(parts[0]);
      arduinoFy = Float.parseFloat(parts[1]);
      arduinoForceX = Float.parseFloat(parts[2]);
      arduinoForceY = Float.parseFloat(parts[3]);
    } catch (Exception e) {}
  }
}
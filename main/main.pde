import processing.svg.*;

PShape baseSVG;
float[][] points;

// --- Settings ---
color dotColor = color(226, 75, 74);
color lineColor = color(55, 138, 221);
float dotRadius = 5;
float lineWeight = 2;
float dotOpacity = 230;
float lineOpacity = 180;
float padding = 150;  // pixels of whitespace on each side

// --- display options ---
boolean showLines = true;
boolean showDots = true;
boolean showLabels = false;

float svgW, svgH;
float vbX, vbY;  // viewBox origin
float uniformScale, offsetX, offsetY;

String shape_file = "hammer";

void setup() {
  size(800, 600);
  
  baseSVG = loadShape(shape_file + "_outline.svg");
  
  String[] rawLines = loadStrings(shape_file + "_outline.svg");
  String raw = join(rawLines, " ");
  vbX  = getViewBoxDim(raw, 0);
  vbY  = getViewBoxDim(raw, 1); 
  svgW = getViewBoxDim(raw, 2);
  svgH = getViewBoxDim(raw, 3);

  // Available drawing area after padding
  float availW = width  - 2 * padding;
  float availH = height - 2 * padding;

  uniformScale = min(availW / svgW, availH / svgH);
  offsetX = (width  - svgW * uniformScale) / 2.0;
  offsetY = (height - svgH * uniformScale) / 2.0;
  
  points = loadCoordinates(shape_file + "_outline_sampled.csv",
                         vbX, vbY, uniformScale, offsetX, offsetY);
  println("Loaded " + points.length + " points");
}

void draw() {
  background(255);

  // Draw SVG at correct aspect ratio, centered
  shape(baseSVG, offsetX, offsetY, svgW * uniformScale, svgH * uniformScale);

  // Lines
  if (showLines && points.length > 1) {
    strokeWeight(lineWeight);
    stroke(red(lineColor), green(lineColor), blue(lineColor), lineOpacity);
    noFill();
    for (int i = 0; i < points.length - 1; i++) {
      line(points[i][0], points[i][1], points[i+1][0], points[i+1][1]);
    }
  }

  // Dots
  if (showDots) {
    fill(red(dotColor), green(dotColor), blue(dotColor), dotOpacity);
    noStroke();
    for (int i = 0; i < points.length; i++) {
      ellipse(points[i][0], points[i][1], dotRadius * 2, dotRadius * 2);
    }
  }

  // Labels
  if (showLabels) {
    fill(red(dotColor), green(dotColor), blue(dotColor), dotOpacity);
    textSize(11);
    noStroke();
    for (int i = 0; i < points.length; i++) {
      text(i + 1, points[i][0] + dotRadius + 2,
                  points[i][1] - dotRadius - 2);
    }
  }

  noLoop();
}

float getViewBoxDim(String svgText, int index) {
  int vbStart = svgText.indexOf("viewBox");
  if (vbStart == -1) {
    println("No viewBox found, falling back to baseSVG.width/height");
    return index == 2 ? baseSVG.width : baseSVG.height;
  }
  int quoteStart = svgText.indexOf("\"", vbStart) + 1;
  int quoteEnd   = svgText.indexOf("\"", quoteStart);
  String vbValue = svgText.substring(quoteStart, quoteEnd).trim();
  String[] parts = splitTokens(vbValue, " ,");
  return Float.parseFloat(parts[index]);
}

float[][] loadCoordinates(String filename, float vbX, float vbY,
                          float scale, float offX, float offY) {
  String[] lines = loadStrings(filename);
  if (lines == null) {
    println("Could not load " + filename);
    return new float[0][2];
  }
  
  int count = 0;
  for (String line : lines) {
    line = line.trim();
    if (line.isEmpty()) continue;
    String[] parts = splitTokens(line, ",;\t");
    if (parts.length >= 2) {
      try {
        Float.parseFloat(parts[0].trim());
        Float.parseFloat(parts[1].trim());
        count++;
      } catch (NumberFormatException e) {}
    }
  }
  
  float[][] pts = new float[count][2];
  int idx = 0;
  for (String line : lines) {
    line = line.trim();
    if (line.isEmpty()) continue;
    String[] parts = splitTokens(line, ",;\t");
    if (parts.length >= 2) {
      try {
        float rawX = Float.parseFloat(parts[0].trim());
        float rawY = Float.parseFloat(parts[1].trim());
        pts[idx][0] = (rawX - vbX) * scale + offX;
        pts[idx][1] = (rawY - vbY) * scale + offY;
        idx++;
      } catch (NumberFormatException e) {}
    }
  }
  
  return pts;
}

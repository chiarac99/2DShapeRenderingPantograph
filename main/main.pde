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
boolean showLines = true;
boolean showDots = true;
boolean showLabels = false;
float svgW, svgH;
float scaleX, scaleY;

void setup() {
  size(800, 600);
  
  baseSVG = loadShape("bunny_outline.svg");
  
  // Read the SVG's natural dimensions
  svgW = baseSVG.width;
  svgH = baseSVG.height;
  
  // Compute scale factors to fit the sketch window
  scaleX = width / svgW;
  scaleY = height / svgH;
  
  points = loadCoordinates("bunny_outline_sampled.csv");
  println("Loaded " + points.length + " points");
}

void draw() {
  background(255);
  
  // Draw SVG scaled to fill the window
  shape(baseSVG, 0, 0, width, height);
  
  // Scale each point before drawing
  if (showLines && points.length > 1) {
    strokeWeight(lineWeight);
    stroke(red(lineColor), green(lineColor), blue(lineColor), lineOpacity);
    noFill();
    for (int i = 0; i < points.length - 1; i++) {
      line(points[i][0] * scaleX, points[i][1] * scaleY,
           points[i+1][0] * scaleX, points[i+1][1] * scaleY);
    }
  }
  
  if (showDots) {
    fill(red(dotColor), green(dotColor), blue(dotColor), dotOpacity);
    noStroke();
    for (int i = 0; i < points.length; i++) {
      ellipse(points[i][0] * scaleX, points[i][1] * scaleY,
              dotRadius * 2, dotRadius * 2);
    }
  }
  
  if (showLabels) {
    fill(red(dotColor), green(dotColor), blue(dotColor), dotOpacity);
    textSize(11);
    noStroke();
    for (int i = 0; i < points.length; i++) {
      text(i + 1, points[i][0] * scaleX + dotRadius + 2,
                  points[i][1] * scaleY - dotRadius - 2);
    }
  }
  
  noLoop();
}

float[][] loadCoordinates(String filename) {
  String[] lines = loadStrings(filename);
  if (lines == null) {
    println("Could not load " + filename);
    return new float[0][2];
  }
  
  // Count valid rows (skip header if non-numeric)
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
      } catch (NumberFormatException e) {
        // skip header or bad row
      }
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
        pts[idx][0] = Float.parseFloat(parts[0].trim());
        pts[idx][1] = Float.parseFloat(parts[1].trim());
        idx++;
      } catch (NumberFormatException e) {
        // skip
      }
    }
  }
  
  return pts;
}

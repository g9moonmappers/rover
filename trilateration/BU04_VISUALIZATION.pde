import processing.serial.*;

Serial myPort;
String inString = "";

// Set your base station positions here
float[][] baseStations = {
  {0.0,   0.0},   // Base station 0 nedre venstre hjørne
  {0.0,   6.0},   // Base station 1 øvre venstre hjørne
  {2.425, 0.0},   // Base station 2 nedre høyre hjørne
  {2.425, 6.0},   // Base station 3 øvre høyre hjørne
};

float minX = -0.5, maxX = 10, minY = -0.5, maxY = 10;
float tagX = 0, tagY = 0;
float[] distances = new float[4];
boolean dataReceived = false;

color[] baseColors = {
  color(255, 0, 0),
  color(0, 255, 0),
  color(0, 0, 255),
  color(255, 255, 0)
};

void setup() {
  size(1400, 800);
  printArray(Serial.list());
  myPort = new Serial(this, "COM11", 115200); // change COM port
  myPort.bufferUntil('\n');
}

void draw() {
  background(0);
  drawGrid();
  if (dataReceived) {
    for (int i=0; i<4; i++)
      drawBaseStation(i, baseStations[i][0], baseStations[i][1], distances[i]);
    drawTag(tagX, tagY);
  }
  drawLegend();
}

void drawGrid() {
  stroke(50); strokeWeight(1);
  for (float x=minX; x<=maxX; x+=0.5) {
    float sx = map(x, minX, maxX, 50, width-50);
    line(sx, 50, sx, height-50);
  }
  for (float y=minY; y<=maxY; y+=0.5) {
    float sy = map(y, minY, maxY, height-50, 50);
    line(50, sy, width-50, sy);
  }
  stroke(100); strokeWeight(2);
  line(map(0, minX, maxX, 50, width-50), 50, map(0, minX, maxX, 50, width-50), height-50);
  line(50, map(0, minY, maxY, height-50, 50), width-50, map(0, minY, maxY, height-50, 50));
  fill(255); textAlign(CENTER);
  text("X (meters)", width/2, height-20);
  pushMatrix(); translate(20, height/2); rotate(-PI/2);
  text("Y (meters)", 0, 0); popMatrix();
}

void drawBaseStation(int index, float x, float y, float distance) {
  float sx = map(x, minX, maxX, 50, width-50);
  float sy = map(y, minY, maxY, height-50, 50);
  if (distance > 0) {
    stroke(baseColors[index]); strokeWeight(1);
    fill(baseColors[index], 30);
    float radius = map(distance, 0, maxX-minX, 0, width-100);
    ellipse(sx, sy, radius*2, radius*2);
  }
  stroke(baseColors[index]); strokeWeight(3);
  fill(baseColors[index]);
  ellipse(sx, sy, 12, 12);
  fill(255); textAlign(CENTER);
  text("BS" + index, sx, sy-15);
  if (distance > 0) text(nf(distance, 1, 2) + "m", sx, sy+25);
}

void drawTag(float x, float y) {
  float sx = map(x, minX, maxX, 50, width-50);
  float sy = map(y, minY, maxY, height-50, 50);
  stroke(255, 100, 100); strokeWeight(2);
  fill(255, 100, 100);
  ellipse(sx, sy, 14, 14);
  line(sx-10, sy, sx+10, sy);
  line(sx, sy-10, sx, sy+10);
  textAlign(CENTER);
  text("(" + nf(x,1,2) + ", " + nf(y,1,2) + ")", sx, sy-20);
}

void drawLegend() {
  fill(255); textAlign(LEFT);
  text("Legend:", 10, 20);
  int y = 40;
  for (int i=0; i<4; i++) {
    fill(baseColors[i]); ellipse(25, y, 8, 8);
    fill(255); text("Base Station " + i, 35, y+5);
    y += 20;
  }
  fill(255, 100, 100); ellipse(25, y, 8, 8);
  fill(255); text("Tag Position", 35, y+5);
}

void serialEvent(Serial p) {
  inString = trim(p.readStringUntil('\n'));
  if (inString == null || inString.startsWith("x,y")) return;
  String[] data = split(inString, ',');
  if (data.length >= 6) {
    try {
      tagX = float(data[0]);
      tagY = float(data[1]);
      for (int i=0; i<4; i++) distances[i] = float(data[2+i]);
      dataReceived = true;
    } catch (Exception e) {
      println("Parse error: " + inString);
    }
  }
}

#include <math.h>

const float BS[4][2] = {
  {0.0, 0.0},  // BS0
  {0.0, 0.0},  // BS1
  {0.0, 0.0},  // BS2
  {0.0, 0.0},  // BS3
};

bool parseUwbData(uint8_t* buffer, int length, float* distances) {
  if (length < 35 || buffer[0]!=0xAA || buffer[1]!=0x25 || buffer[2]!=0x01) return false;
  for (int i=0; i<4; i++) {
    int offset = 3 + i*4;
    uint32_t raw = buffer[offset] | (buffer[offset+1]<<8) | (buffer[offset+2]<<16) | (buffer[offset+3]<<24);
    distances[i] = raw > 0 ? raw/1000.0 : -1.0;
  }
  return true;
}

bool trilaterate(float* distances, float* x, float* y) {
  float x1=BS[0][0], y1=BS[0][1], r1=distances[0];
  float A[3][2], b[3];
  int equationCount=0;
  for (int i=1; i<4&&equationCount<3; i++) {
    if (distances[i] < 0) continue;
    A[equationCount][0]=2*(BS[i][0]-x1); A[equationCount][1]=2*(BS[i][1]-y1);
    b[equationCount]=distances[i]*distances[i]-r1*r1-BS[i][0]*BS[i][0]+x1*x1-BS[i][1]*BS[i][1]+y1*y1;
    equationCount++;
  }
  if (equationCount < 2) return false;
  float det = A[0][0]*A[1][1] - A[0][1]*A[1][0];
  if (fabs(det) < 1e-6) return false;
  *x = (b[0]*A[1][1] - b[1]*A[0][1]) / det;
  *y = (A[0][0]*b[1] - A[1][0]*b[0]) / det;
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial.println("x,y,dist0,dist1,dist2,dist3");
}

void loop() {
  static uint8_t buffer[256];
  static int bufferIndex = 0;
  static bool messageStarted = false;

  while (Serial2.available()) {
    uint8_t byte = Serial2.read();
    if (!messageStarted && byte==0xAA) { messageStarted=true; bufferIndex=0; buffer[bufferIndex++]=byte; }
    else if (messageStarted) {
      buffer[bufferIndex++] = byte;
      if (bufferIndex >= 35) {
        float distances[4];
        if (parseUwbData(buffer, bufferIndex, distances)) {
          float x, y;
          bool valid = trilaterate(distances, &x, &y);
          Serial.print(valid ? String(x,3) : "0"); Serial.print(",");
          Serial.print(valid ? String(y,3) : "0");
          for (int i=0; i<4; i++) { Serial.print(","); Serial.print(distances[i],3); }
          Serial.println();
        }
        messageStarted=false; bufferIndex=0;
      }
      if (bufferIndex >= 256) { messageStarted=false; bufferIndex=0; }
    }
  }
}
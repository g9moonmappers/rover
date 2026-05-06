#include <math.h>

const float BS[4][2] = {
  {0.0,   0.0},    // BS0 - lower left
  {0.0,   2.208},  // BS1 - upper left
  {2.404, 0.0},    // BS2 - lower right
  {2.404, 2.208},  // BS3 - upper right
};

const float CALIB_SLOPE[4]     = {1.0499, 1.0499, 1.0416, 1.0478};
const float CALIB_INTERCEPT[4] = {452.6,  452.6,  381.7,  262.0 };

bool parseUwbData(uint8_t* buffer, int length, float* distances) {
  if (length < 35 || buffer[0]!=0xAA || buffer[1]!=0x25 || buffer[2]!=0x01) return false;
  for (int i=0; i<4; i++) {
    int offset = 3 + i*4;
    uint32_t raw = buffer[offset] | (buffer[offset+1]<<8) | (buffer[offset+2]<<16) | (buffer[offset+3]<<24);
    if (raw > 0) {
      distances[i] = ((float)raw - CALIB_INTERCEPT[i]) / CALIB_SLOPE[i] / 1000.0;
    } else {
      distances[i] = -1.0;
    }
  }
  return true;
}

bool trilaterate(float* distances, float* x, float* y) {
  struct ValidData { float x, y, dist; };
  ValidData valid[4];
  int count = 0;
  for (int i=0; i<4; i++) {
    if (distances[i] > 0)
      valid[count++] = {BS[i][0], BS[i][1], distances[i]};
  }
  if (count < 3) return false;

  float x1=valid[0].x, y1=valid[0].y, r1=valid[0].dist;
  int eq = count - 1;
  float A[3][2], b[3];
  for (int i=0; i<eq; i++) {
    A[i][0] = 2*(valid[i+1].x - x1);
    A[i][1] = 2*(valid[i+1].y - y1);
    b[i] = valid[i+1].dist*valid[i+1].dist - r1*r1
           - valid[i+1].x*valid[i+1].x + x1*x1
           - valid[i+1].y*valid[i+1].y + y1*y1;
  }

  float ATA[2][2] = {0}, ATb[2] = {0};
  for (int i=0; i<eq; i++) {
    ATA[0][0] += A[i][0]*A[i][0];
    ATA[0][1] += A[i][0]*A[i][1];
    ATA[1][0] += A[i][1]*A[i][0];
    ATA[1][1] += A[i][1]*A[i][1];
    ATb[0]    += A[i][0]*b[i];
    ATb[1]    += A[i][1]*b[i];
  }

  float det = ATA[0][0]*ATA[1][1] - ATA[0][1]*ATA[1][0];
  if (fabs(det) < 1e-6) return false;

  *x = -(ATb[0]*ATA[1][1] - ATb[1]*ATA[0][1]) / det;
  *y = -(ATA[0][0]*ATb[1] - ATA[1][0]*ATb[0]) / det;
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

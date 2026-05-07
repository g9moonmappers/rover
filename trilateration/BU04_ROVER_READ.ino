#include <math.h>

// Må endres for hver gang trilatereringen blir plassert i et nytt rom eller flyttes på
const float baseStationPositions[4][2] = {
  {0.0,   0.0},   // Base station 0 nedre venstre hjørne
  {0.0,   2.208}, // Base station 1 øvre venstre hjørne
  {2.404, 0.0},   // Base station 2 nedre høyre hjørne
  {2.404, 2.208}, // Base station 3 øvre høyre hjørne
};

// Kalibrering av hver base station, finnes i google sheets
const float calibrationSlope[4]     = {1.0499, 1.0499, 1.0416, 1.0478};
const float calibrationIntercept[4] = {452.6,  452.6,  381.7,  262.0 };

// Leser av avstanden mellom tag og hver basestasjon fra BU-04
bool parseUwbData(byte* buffer, int length, float* distances) {
  if (length < 35 || buffer[0]!=0xAA || buffer[1]!=0x25 || buffer[2]!=0x01) return false;
  for (int i=0; i<4; i++) {
    int offset = 3 + i*4;
    unsigned long raw = buffer[offset] | (buffer[offset+1]<<8) | (buffer[offset+2]<<16) | (buffer[offset+3]<<24);
    distances[i] = raw > 0 ? ((float)raw - calibrationIntercept[i]) / calibrationSlope[i] / 1000.0 : -1.0;
  }
  return true;
}

// Regner ut posisjonen til taggen basert på avstanden til hver basestasjon, trenger minst 3 base stations
bool trilaterate(float* distances, float* x, float* y) {
  struct ValidStation { float x, y, dist; };
  ValidStation validStations[4];
  int count = 0;
  for (int i=0; i<4; i++) {
    if (distances[i] > 0)
      validStations[count++] = {baseStationPositions[i][0], baseStationPositions[i][1], distances[i]};
  }
  if (count < 3) return false;

  float refX=validStations[0].x, refY=validStations[0].y, refDist=validStations[0].dist;
  int equations = count - 1;
  float coefficients[3][2], rightSide[3];
  for (int i=0; i<equations; i++) {
    coefficients[i][0] = 2*(validStations[i+1].x - refX);
    coefficients[i][1] = 2*(validStations[i+1].y - refY);
    rightSide[i] = validStations[i+1].dist*validStations[i+1].dist - refDist*refDist
                   - validStations[i+1].x*validStations[i+1].x + refX*refX
                   - validStations[i+1].y*validStations[i+1].y + refY*refY;
  }

  float leastSquaresA[2][2] = {0}, leastSquaresB[2] = {0};
  for (int i=0; i<equations; i++) {
    leastSquaresA[0][0] += coefficients[i][0]*coefficients[i][0];
    leastSquaresA[0][1] += coefficients[i][0]*coefficients[i][1];
    leastSquaresA[1][0] += coefficients[i][1]*coefficients[i][0];
    leastSquaresA[1][1] += coefficients[i][1]*coefficients[i][1];
    leastSquaresB[0]    += coefficients[i][0]*rightSide[i];
    leastSquaresB[1]    += coefficients[i][1]*rightSide[i];
  }

  float determinant = leastSquaresA[0][0]*leastSquaresA[1][1] - leastSquaresA[0][1]*leastSquaresA[1][0];
  if (fabs(determinant) < 1e-6) return false;

  *x = -(leastSquaresB[0]*leastSquaresA[1][1] - leastSquaresB[1]*leastSquaresA[0][1]) / determinant;
  *y = -(leastSquaresA[0][0]*leastSquaresB[1] - leastSquaresA[1][0]*leastSquaresB[0]) / determinant;
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200); // BU-04 TX2 → Mega pin 17 (RX2)
  Serial.println("x,y,dist0,dist1,dist2,dist3");
}

// Les data fra BU-04, regn ut posisjon og send det ut på Serial
void loop() {
  static byte buffer[256];
  static int bufferIndex = 0;
  static bool messageStarted = false;
  char output[64];

  while (Serial2.available()) {
    byte incoming = Serial2.read();
    if (!messageStarted && incoming==0xAA) {
      messageStarted = true;
      bufferIndex = 0;
      buffer[bufferIndex++] = incoming;
    } else if (messageStarted) {
      buffer[bufferIndex++] = incoming;
      if (bufferIndex >= 35) {
        float distances[4];
        if (parseUwbData(buffer, bufferIndex, distances)) {
          float x, y;
          bool valid = trilaterate(distances, &x, &y);
          if (valid) snprintf(output, sizeof(output), "%.3f,%.3f", x, y);
          else       snprintf(output, sizeof(output), "0,0");
          Serial.print(output);
          for (int i=0; i<4; i++) {
            snprintf(output, sizeof(output), ",%.3f", distances[i]);
            Serial.print(output);
          }
          Serial.println();
        }
        messageStarted = false;
        bufferIndex = 0;
      }
      if (bufferIndex >= 256) { messageStarted = false; bufferIndex = 0; }
    }
  }
}

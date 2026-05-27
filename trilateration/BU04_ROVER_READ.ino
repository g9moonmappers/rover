#include <math.h>

// Må endres for hver gang trilatereringen blir plassert i et nytt rom eller flyttes på
const float ankerPosisjoner[4][2] = {
  {0.0,   0.0},   // Anker 0 nedre venstre hjørne
  {0.0,   6.0},   // Anker 1 øvre venstre hjørne
  {2.425, 0.0},   // Anker 2 nedre høyre hjørne
  {2.425, 6.0},   // Anker 3 øvre høyre hjørne
};

// Kalibrering av hvert anker, finnes i google sheets https://docs.google.com/spreadsheets/d/1d4-OWOsPB2cUo5gQq4QdnQ2pV3oUy5KFTMYp_f5sS5Y/edit?usp=sharing
const float kalibreringStigning[4]    = {1.0499, 1.0499, 1.0416, 1.0478};
const float kalibreringSkjæringspunkt[4] = {452.6,  452.6,  381.7,  262.0 };

// Leser av avstanden mellom tag og hvert anker fra BU-04
bool parseUwbData(byte* buffer, int length, float* distances) {
  if (length < 35 || buffer[0]!=0xAA || buffer[1]!=0x25 || buffer[2]!=0x01) return false;
  for (int i=0; i<4; i++) {
    int offset = 3 + i*4;
    unsigned long raw = buffer[offset] | (buffer[offset+1]<<8) | (buffer[offset+2]<<16) | (buffer[offset+3]<<24);
    distances[i] = raw > 0 ? ((float)raw - kalibreringSkjæringspunkt[i]) / kalibreringStigning[i] / 1000.0 : -1.0;
  }
  return true;
}

// Regner ut posisjonen til taggen basert på avstanden til hvert anker, trenger minst 3 ankere
bool trilaterate(float* distances, float* x, float* y) {
  struct GyldigAnker { float x, y, dist; };
  GyldigAnker gyldigeAnkere[4];
  int count = 0;
  for (int i=0; i<4; i++) {
    if (distances[i] > 0)
      gyldigeAnkere[count++] = {ankerPosisjoner[i][0], ankerPosisjoner[i][1], distances[i]};
  }
  if (count < 3) return false;

  float refX=gyldigeAnkere[0].x, refY=gyldigeAnkere[0].y, refDist=gyldigeAnkere[0].dist;
  int equations = count - 1;
  float coefficients[3][2], rightSide[3];
  for (int i=0; i<equations; i++) {
    coefficients[i][0] = 2*(gyldigeAnkere[i+1].x - refX);
    coefficients[i][1] = 2*(gyldigeAnkere[i+1].y - refY);
    rightSide[i] = gyldigeAnkere[i+1].dist*gyldigeAnkere[i+1].dist - refDist*refDist
                   - gyldigeAnkere[i+1].x*gyldigeAnkere[i+1].x + refX*refX
                   - gyldigeAnkere[i+1].y*gyldigeAnkere[i+1].y + refY*refY;
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
  char xStr[10], yStr[10], dStr[10];

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
          if (valid) {
            dtostrf(x, 1, 3, xStr);
            dtostrf(y, 1, 3, yStr);
          } else {
            strcpy(xStr, "0");
            strcpy(yStr, "0");
          }
          Serial.print(xStr); Serial.print(","); Serial.print(yStr);
          for (int i=0; i<4; i++) {
            dtostrf(distances[i], 1, 3, dStr);
            Serial.print(","); Serial.print(dStr);
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

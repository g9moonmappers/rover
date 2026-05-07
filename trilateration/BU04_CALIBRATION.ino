const int   START_MM              = 100;   // Kalibrering start mm
const int   END_MM                = 1500;  // Kalibrering slutt mm
const int   STEP_MM               = 100;   // Steg-størrelse mellom hver måling
const int   SAMPLES_PER_STEP      = 40;    // Hvor mange målinger som blir tatt på hver avstand
const int   SAMPLE_INTERVAL_MS    = 200;   // hvor ofte hver måling blir tatt
const int   TARGET_BASE_STATION   = 3;     // hvilken base station skal kalibreres
const float CALIBRATION_SLOPE     = 1.0;
const float CALIBRATION_INTERCEPT = 0.0;

const int numberOfSteps = (END_MM - START_MM) / STEP_MM + 1;
float allMeasurements[15][40];

static byte buffer[256];
static int bufferIndex = 0;
static bool messageStarted = false;
float distances[8];

// Leser av avstanden mellom tag og hver basestasjon fra BU-04
bool readUWB() {
  while (Serial2.available()) {
    byte incoming = Serial2.read();
    if (!messageStarted && incoming == 0xAA) {
      messageStarted = true;
      bufferIndex = 0;
      buffer[bufferIndex++] = incoming;
    } else if (messageStarted) {
      buffer[bufferIndex++] = incoming;
      if (bufferIndex >= 35) {
        if (buffer[0]==0xAA && buffer[1]==0x25 && buffer[2]==0x01) {
          for (int i=0; i<8; i++) {
            int offset = 3 + i*4;
            unsigned long raw = buffer[offset] | (buffer[offset+1]<<8) | (buffer[offset+2]<<16) | (buffer[offset+3]<<24);
            distances[i] = raw > 0 ? raw/1000.0 : -1.0;
          }
          messageStarted = false;
          bufferIndex = 0;
          return true;
        }
        messageStarted = false;
        bufferIndex = 0;
      }
      if (bufferIndex >= 256) { messageStarted = false; bufferIndex = 0; }
    }
  }
  return false;
}

int currentStep = 0;
int samplesCollected = 0;
bool waitingForUser = true;
bool finished = false;
unsigned long lastSampleTime = 0;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  delay(1000);
  Serial.print("Plasser tag på"); Serial.print(START_MM); Serial.println("mm, send et tegn i serial monitor for å starte");
}

void loop() {
  if (finished) return;

  if (waitingForUser) {
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      waitingForUser = false;
    }
    return;
  }

  if (readUWB() && distances[TARGET_BASE_STATION] > 0 && millis() - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    float measurement = (distances[TARGET_BASE_STATION] * 1000.0 - CALIBRATION_INTERCEPT) / CALIBRATION_SLOPE;
    allMeasurements[currentStep][samplesCollected] = measurement;
    lastSampleTime = millis();
    samplesCollected++;
    Serial.println((int)measurement);

    if (samplesCollected >= SAMPLES_PER_STEP) {
      samplesCollected = 0;
      currentStep++;
      if (currentStep >= numberOfSteps) {
        Serial.println("\n--- RESULTATER (mm) ---");
        for (int step=0; step<numberOfSteps; step++)
          for (int sample=0; sample<SAMPLES_PER_STEP; sample++)
            Serial.println((int)allMeasurements[step][sample]);
        Serial.println("Ferdig.");
        finished = true;
      } else {
        int nextDistance = START_MM + currentStep * STEP_MM;
        Serial.print("Plasser tag på "); Serial.print(nextDistance); Serial.println("mm, send et tegn i serial monitor for å fortsette");
        waitingForUser = true;
      }
    }
  }
}
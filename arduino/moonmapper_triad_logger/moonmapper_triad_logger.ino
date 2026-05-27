/*
  Triad-logger: to AS7265X via I2C-mux.
  Serial 115200: SB | BURST <sample_id> | PS
  CSV: sample_id,burst_index,timestamp_ms,S0_410..S1_940,N0_*,N1_*
  Kalibrering: LED_ON - LED_OFF, minus backgroundDiff etter SB (sand).
*/

#include <Wire.h>

#include <SparkFun_AS7265X.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>

AS7265X triad;
QWIICMUX mux;

static const uint8_t SENSOR0_PORT = 0;
static const uint8_t SENSOR1_PORT = 1;

static const uint8_t NUM_CHANNELS = 18;
static const uint8_t NUM_SENSORS = 2;
static const uint8_t BURST_SAMPLES = 20;
static const uint8_t TRIAD_DEVICES[3] = {AS72651_NIR, AS72652_VISIBLE, AS72653_UV};

// Bolgelengder nm (18 kanaler per sensor)
static const uint16_t WL[NUM_CHANNELS] = {
  410, 435, 460, 485, 510, 535, 560, 585, 610, 645, 680, 705, 730, 760, 810, 860, 900, 940
};

// Bakgrunn per sensor (sand), LED_ON - LED_OFF
float backgroundDiff[NUM_SENSORS][NUM_CHANNELS];
bool backgroundReady[NUM_SENSORS] = {false, false};


bool selectSensorPort(uint8_t port) {
  if (port > 7) return false;
  mux.setPort(port);
  delay(2);
  return true;
}

bool initSensorOnPort(uint8_t port) {
  if (!selectSensorPort(port)) return false;
  delay(5);
  if (!triad.begin()) {
    return false;
  }

  triad.setGain(AS7265X_GAIN_64X);
  triad.setIntegrationCycles(18);  // ca. 50 ms eksponering
  triad.disableIndicator();
  triad.disableBulb(AS72651_NIR);
  triad.disableBulb(AS72652_VISIBLE);
  triad.disableBulb(AS72653_UV);
  return true;
}

// LED_ON - LED_OFF, minus bakgrunn hvis SB er kjort
bool readDiffCorrected(uint8_t sensorIdx, uint8_t muxPort, float corrected[NUM_CHANNELS]) {
  if (!selectSensorPort(muxPort)) return false;

  float onVals[NUM_CHANNELS];
  float offVals[NUM_CHANNELS];

  // LED av
  triad.disableBulb(AS72651_NIR);
  triad.disableBulb(AS72652_VISIBLE);
  triad.disableBulb(AS72653_UV);
  triad.takeMeasurements();
  offVals[0]  = triad.getCalibratedA();
  offVals[1]  = triad.getCalibratedB();
  offVals[2]  = triad.getCalibratedC();
  offVals[3]  = triad.getCalibratedD();
  offVals[4]  = triad.getCalibratedE();
  offVals[5]  = triad.getCalibratedF();
  offVals[6]  = triad.getCalibratedG();
  offVals[7]  = triad.getCalibratedH();
  offVals[8]  = triad.getCalibratedR();
  offVals[9]  = triad.getCalibratedI();
  offVals[10] = triad.getCalibratedS();
  offVals[11] = triad.getCalibratedJ();
  offVals[12] = triad.getCalibratedT();
  offVals[13] = triad.getCalibratedU();
  offVals[14] = triad.getCalibratedV();
  offVals[15] = triad.getCalibratedW();
  offVals[16] = triad.getCalibratedK();
  offVals[17] = triad.getCalibratedL();

  // LED pa
  triad.enableBulb(AS72651_NIR);
  triad.enableBulb(AS72652_VISIBLE);
  triad.enableBulb(AS72653_UV);
  triad.takeMeasurements();
  onVals[0]  = triad.getCalibratedA();
  onVals[1]  = triad.getCalibratedB();
  onVals[2]  = triad.getCalibratedC();
  onVals[3]  = triad.getCalibratedD();
  onVals[4]  = triad.getCalibratedE();
  onVals[5]  = triad.getCalibratedF();
  onVals[6]  = triad.getCalibratedG();
  onVals[7]  = triad.getCalibratedH();
  onVals[8]  = triad.getCalibratedR();
  onVals[9]  = triad.getCalibratedI();
  onVals[10] = triad.getCalibratedS();
  onVals[11] = triad.getCalibratedJ();
  onVals[12] = triad.getCalibratedT();
  onVals[13] = triad.getCalibratedU();
  onVals[14] = triad.getCalibratedV();
  onVals[15] = triad.getCalibratedW();
  onVals[16] = triad.getCalibratedK();
  onVals[17] = triad.getCalibratedL();

  triad.disableBulb(AS72651_NIR);
  triad.disableBulb(AS72652_VISIBLE);
  triad.disableBulb(AS72653_UV);

  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    float diff = onVals[i] - offVals[i];
    if (backgroundReady[sensorIdx]) {
      diff -= backgroundDiff[sensorIdx][i];
    }
    corrected[i] = diff;
  }

  return true;
}

void printHeader() {
  Serial.print("sample_id,burst_index,timestamp_ms");
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    Serial.print(",S0_");
    Serial.print(WL[i]);
  }
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    Serial.print(",S1_");
    Serial.print(WL[i]);
  }
  // Normaliserte kolonner N0_*, N1_* (form, ikke absolutt nivaa)
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    Serial.print(",N0_");
    Serial.print(WL[i]);
  }
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    Serial.print(",N1_");
    Serial.print(WL[i]);
  }
  Serial.println();
}

void printCsvRow(const String &sampleId, uint8_t burstIndex, unsigned long ts, const float s0[NUM_CHANNELS], const float s1[NUM_CHANNELS]) {
  Serial.print(sampleId);
  Serial.print(",");
  Serial.print(burstIndex);
  Serial.print(",");
  Serial.print(ts);

  float sum0 = 0.0f;
  float sum1 = 0.0f;
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    // Kun positive bidrag i normalisering (unngar sum=0).
    if (s0[i] > 0.0f) sum0 += s0[i];
    if (s1[i] > 0.0f) sum1 += s1[i];
  }
  const float inv0 = (sum0 > 1e-9f) ? (1.0f / sum0) : 0.0f;
  const float inv1 = (sum1 > 1e-9f) ? (1.0f / sum1) : 0.0f;

  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    Serial.print(",");
    Serial.print(s0[i], 6);
  }
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    Serial.print(",");
    Serial.print(s1[i], 6);
  }
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    Serial.print(",");
    Serial.print((s0[i] > 0.0f ? s0[i] * inv0 : 0.0f), 6);
  }
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    Serial.print(",");
    Serial.print((s1[i] > 0.0f ? s1[i] * inv1 : 0.0f), 6);
  }
  Serial.println();
}

void handleSampleBackground() {
  float s0[NUM_CHANNELS];
  float s1[NUM_CHANNELS];
  bool prev0 = backgroundReady[0];
  bool prev1 = backgroundReady[1];
  backgroundReady[0] = false;
  backgroundReady[1] = false;

  bool ok0 = readDiffCorrected(0, SENSOR0_PORT, s0);
  bool ok1 = readDiffCorrected(1, SENSOR1_PORT, s1);

  if (!ok0 || !ok1) {
    Serial.println("ERROR,SB failed to read sensors (check wiring/mux ports)");
    backgroundReady[0] = prev0;
    backgroundReady[1] = prev1;
    return;
  }

  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    backgroundDiff[0][i] = s0[i];
    backgroundDiff[1][i] = s1[i];
  }
  backgroundReady[0] = true;
  backgroundReady[1] = true;

  Serial.println("OK,SB background sampled for sensor0 and sensor1");
}

void handlePrintSeparate() {
  float s0[NUM_CHANNELS];
  float s1[NUM_CHANNELS];
  bool ok0 = readDiffCorrected(0, SENSOR0_PORT, s0);
  bool ok1 = readDiffCorrected(1, SENSOR1_PORT, s1);
  if (!ok0 || !ok1) {
    Serial.println("ERROR,PS failed to read sensors");
    return;
  }

  Serial.print("SENSOR0");
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    Serial.print(",S0_");
    Serial.print(WL[i]);
    Serial.print("=");
    Serial.print(s0[i], 6);
  }
  Serial.println();

  Serial.print("SENSOR1");
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    Serial.print(",S1_");
    Serial.print(WL[i]);
    Serial.print("=");
    Serial.print(s1[i], 6);
  }
  Serial.println();
}

void handleBurst(const String &sampleId) {
  if (!backgroundReady[0] || !backgroundReady[1]) {
    Serial.println("ERROR,BURST requires background. Run SB first.");
    return;
  }

  printHeader();

  for (uint8_t i = 0; i < BURST_SAMPLES; i++) {
    float s0[NUM_CHANNELS];
    float s1[NUM_CHANNELS];
    bool ok0 = readDiffCorrected(0, SENSOR0_PORT, s0);
    bool ok1 = readDiffCorrected(1, SENSOR1_PORT, s1);
    if (!ok0 || !ok1) {
      Serial.println("ERROR,BURST failed to read sensors");
      return;
    }
    unsigned long ts = millis();
    printCsvRow(sampleId, i, ts, s0, s1);
  }
}

String readLine() {
  static String line = "";
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String out = line;
      line = "";
      out.trim();
      return out;
    }
    line += c;
    if (line.length() > 200) {
      line = "";
      return "";
    }
  }
  return "";
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Wire.begin();
  Wire.setClock(400000);

  if (!mux.begin()) {
    Serial.println("ERROR,Mux not detected. Check Qwiic Mux wiring.");
    return;
  }

  bool ok0 = initSensorOnPort(SENSOR0_PORT);
  bool ok1 = initSensorOnPort(SENSOR1_PORT);

  if (!ok0 || !ok1) {
    Serial.println("ERROR,AS7265X not detected on one or both mux ports (0 and 1).");
    Serial.print("INFO,ok0=");
    Serial.print(ok0 ? "1" : "0");
    Serial.print(",ok1=");
    Serial.println(ok1 ? "1" : "0");
  } else {
    Serial.println("OK,Triad logger ready. Commands: SB | BURST <sample_id> | PS");
  }
}

void loop() {
  String cmd = readLine();
  if (cmd.length() == 0) {
    delay(5);
    return;
  }

  if (cmd == "SB") {
    handleSampleBackground();
    return;
  }

  if (cmd == "PS") {
    handlePrintSeparate();
    return;
  }

  if (cmd.startsWith("BURST")) {
    int sp = cmd.indexOf(' ');
    if (sp < 0 || sp == (int)cmd.length() - 1) {
      Serial.println("ERROR,BURST requires sample_id. Usage: BURST S0001");
      return;
    }
    String sampleId = cmd.substring(sp + 1);
    sampleId.trim();
    handleBurst(sampleId);
    return;
  }

  Serial.print("ERROR,Unknown command: ");
  Serial.println(cmd);
}

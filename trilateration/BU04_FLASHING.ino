HardwareSerial BU04_configuration(2);

void configureBU04(int id, int role, int channel, int rate) {
  while (BU04_configuration.available()) BU04_configuration.read();
  delay(2000);

  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+SETCFG=%d,%d,%d,%d\r\n", id, role, channel, rate);
  Serial.println("Sending AT+SETCFG...");
  BU04_configuration.print(cmd);
  delay(1000);
  if (BU04_configuration.available()) Serial.println(BU04_configuration.readString());

  Serial.println("Sending AT+SAVE...");
  BU04_configuration.print("AT+SAVE\r\n");
  delay(3000);
  if (BU04_configuration.available()) Serial.println(BU04_configuration.readString());

  Serial.println("Sending AT+GETCFG...");
  BU04_configuration.print("AT+GETCFG\r\n");
  delay(1000);
  if (BU04_configuration.available()) Serial.println(BU04_configuration.readString());
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  BU04_configuration.begin(115200, SERIAL_8N1, D3, D2); // (baud rate, data format, RX1, TX1);
  delay(50);

  Serial.println("Starting board configuration...");
  configureBU04(2, 1, 1, 1); // id=2, role (0 = tag 1 = base station), channel=1 (6.25-6.75 GHz), data rate (0=850Kbps 1=6.8Mbps)
}

void loop() {}
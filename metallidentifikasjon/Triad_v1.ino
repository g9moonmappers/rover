/*
For å lære meg biblotekene og for å hente inspirasjon har jeg brukt ekesmeplfilene som er gitt med biblotekene.

Mux:
Example 1 - library: https://github.com/sparkfun/SparkFun_I2C_Mux_Arduino_Library 
By: Nathan Seidle @ SparkFun Electronics
Date: May 17th, 2020
License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

AS7265X: 
Hentet ut kommandoer for de forskjellige instillingene og hvilken betydning de har for systemet. fra eksempel 3. Hentet ut komandoer som 
-sensor.setIntegrationCycles(30);
-sensor.setGain(AS7265X_GAIN_64X);
Fra eksempel 1 hentet jeg grunnleggende funksjoner som hvordan sensoren kjører en måling og hvordan jeg henter ut målingen fra systemet.
jeg hentet også ut hvordan jeg kunne starte sensoren.

Example 1 and 3 - library: https://github.com/sparkfun/SparkFun_AS7265x_Arduino_Library
By: Nathan Seidle
SparkFun Electronics
Date: October 25th, 2018
License: MIT. See license file for more information but you can basically do whatever you want with this code.

*/

#include <SparkFun_I2C_Mux_Arduino_Library.h>
//#include "SparkFun_AS7265X.h"
//AS7265X sensor;
QWIICMUX myMux;

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (myMux.begin() == false) {
    Serial.println("Mux feilet tilkobling...");
    while (1)
      ;
  }
  for (int i = 0; i < 2; i++) {
    myMux.setPort(i);
    if (sensor.begin() == false) {
      Serial.print("AS7265x id: ");
      Serial.print(i + 1);
      Serial.println(" feilet tilkobling");
      while (1)
        ;
    }
  }
}

void AS7265x() {

  for (int i = 0; i < 2; i++) {
    Serial.print("initiate read on sensor: ");
    Serial.println(i + 1);
    myMux.setPort(i);
    byte currentPortNumber = myMux.getPort();
    sensor.disableIndicator();

    sensor.takeMeasurementsWithBulb();

    sensor.enableIndicator();

    delay(100);
  }
}


void AS7265x_array_datasett(float aluminium[], float jern[], float staal[], float titan[], float bakgrunn[], char flagg[]) {
  int rep = 50;

if(flagg[0] == 'S')
  if (flagg[1] == 'A') {

    for (int i = 0; i < rep; i++) {
      sensor.disableIndicator();
      sensor.takeMeasurementsWithBulb();
      sensor.enableIndicator();
      delay(100);
      aluminium[0] = aluminium[0] + sensor.getCalibratedA();
      aluminium[1] = aluminium[1] + sensor.getCalibratedB();
      aluminium[2] = aluminium[2] + sensor.getCalibratedC();
      aluminium[3] = aluminium[3] + sensor.getCalibratedD();
      aluminium[4] = aluminium[4] + sensor.getCalibratedE();
      aluminium[5] = aluminium[5] + sensor.getCalibratedF();
      aluminium[6] = aluminium[6] + sensor.getCalibratedG();
      aluminium[7] = aluminium[7] + sensor.getCalibratedH();
      aluminium[8] = aluminium[8] + sensor.getCalibratedR();
      aluminium[9] = aluminium[9] + sensor.getCalibratedI();
      aluminium[10] = aluminium[10] + sensor.getCalibratedS();
      aluminium[11] = aluminium[11] + sensor.getCalibratedJ();
      aluminium[12] = aluminium[12] + sensor.getCalibratedT();
      aluminium[13] = aluminium[13] + sensor.getCalibratedU();
      aluminium[14] = aluminium[14] + sensor.getCalibratedV();
      aluminium[15] = aluminium[15] + sensor.getCalibratedW();
      aluminium[16] = aluminium[16] + sensor.getCalibratedK();
      aluminium[17] = aluminium[17] + sensor.getCalibratedL();
      Serial.println("aluminium");
    }

    for (int i = 0; i <= 17; i++) {
      aluminium[i] = aluminium[i] / rep;
    }
    flagg[0] = 0;
    flagg[1] = 0;

  } 
  
  else if (flagg[1] == 'J') {

    for (int i = 0; i < rep; i++) {
      sensor.disableIndicator();
      sensor.takeMeasurementsWithBulb();
      sensor.enableIndicator();
      delay(100);
      jern[0] = jern[0] + sensor.getCalibratedA();
      jern[1] = jern[1] + sensor.getCalibratedB();
      jern[2] = jern[2] + sensor.getCalibratedC();
      jern[3] = jern[3] + sensor.getCalibratedD();
      jern[4] = jern[4] + sensor.getCalibratedE();
      jern[5] = jern[5] + sensor.getCalibratedF();
      jern[6] = jern[6] + sensor.getCalibratedG();
      jern[7] = jern[7] + sensor.getCalibratedH();
      jern[8] = jern[8] + sensor.getCalibratedR();
      jern[9] = jern[9] + sensor.getCalibratedI();
      jern[10] = jern[10] + sensor.getCalibratedS();
      jern[11] = jern[11] + sensor.getCalibratedJ();
      jern[12] = jern[12] + sensor.getCalibratedT();
      jern[13] = jern[13] + sensor.getCalibratedU();
      jern[14] = jern[14] + sensor.getCalibratedV();
      jern[15] = jern[15] + sensor.getCalibratedW();
      jern[16] = jern[16] + sensor.getCalibratedK();
      jern[17] = jern[17] + sensor.getCalibratedL();
      Serial.println("jern");
    }

    for (int i = 0; i <= 17; i++) {
      jern[i] = jern[i] / rep;
    }
    flagg[0] = 0;
    flagg[1] = 0;
  }


  else if (flagg[1] == 'B') {

    for (int i = 0; i < rep; i++) {
      sensor.disableIndicator();
      sensor.takeMeasurementsWithBulb();
      sensor.enableIndicator();
      delay(100);
      bakgrunn[0] = bakgrunn[0] + sensor.getCalibratedA();
      bakgrunn[1] = bakgrunn[1] + sensor.getCalibratedB();
      bakgrunn[2] = bakgrunn[2] + sensor.getCalibratedC();
      bakgrunn[3] = bakgrunn[3] + sensor.getCalibratedD();
      bakgrunn[4] = bakgrunn[4] + sensor.getCalibratedE();
      bakgrunn[5] = bakgrunn[5] + sensor.getCalibratedF();
      bakgrunn[6] = bakgrunn[6] + sensor.getCalibratedG();
      bakgrunn[7] = bakgrunn[7] + sensor.getCalibratedH();
      bakgrunn[8] = bakgrunn[8] + sensor.getCalibratedR();
      bakgrunn[9] = bakgrunn[9] + sensor.getCalibratedI();
      bakgrunn[10] = bakgrunn[10] + sensor.getCalibratedS();
      bakgrunn[11] = bakgrunn[11] + sensor.getCalibratedJ();
      bakgrunn[12] = bakgrunn[12] + sensor.getCalibratedT();
      bakgrunn[13] = bakgrunn[13] + sensor.getCalibratedU();
      bakgrunn[14] = bakgrunn[14] + sensor.getCalibratedV();
      bakgrunn[15] = bakgrunn[15] + sensor.getCalibratedW();
      bakgrunn[16] = bakgrunn[16] + sensor.getCalibratedK();
      bakgrunn[17] = bakgrunn[17] + sensor.getCalibratedL();
      Serial.println("Bakgrunn");
    }

    for (int i = 0; i <= 17; i++) {
      bakgrunn[i] = bakgrunn[i] / rep;
    }
    flagg[0] = 0;
    flagg[1] = 0;
  }

  else if (flagg[1] == 'S') {

    for (int i = 0; i < rep; i++) {
      sensor.disableIndicator();
      sensor.takeMeasurementsWithBulb();
      sensor.enableIndicator();
      delay(100);
      staal[0] = staal[0] + sensor.getCalibratedA();
      staal[1] = staal[1] + sensor.getCalibratedB();
      staal[2] = staal[2] + sensor.getCalibratedC();
      staal[3] = staal[3] + sensor.getCalibratedD();
      staal[4] = staal[4] + sensor.getCalibratedE();
      staal[5] = staal[5] + sensor.getCalibratedF();
      staal[6] = staal[6] + sensor.getCalibratedG();
      staal[7] = staal[7] + sensor.getCalibratedH();
      staal[8] = staal[8] + sensor.getCalibratedR();
      staal[9] = staal[9] + sensor.getCalibratedI();
      staal[10] = staal[10] + sensor.getCalibratedS();
      staal[11] = staal[11] + sensor.getCalibratedJ();
      staal[12] = staal[12] + sensor.getCalibratedT();
      staal[13] = staal[13] + sensor.getCalibratedU();
      staal[14] = staal[14] + sensor.getCalibratedV();
      staal[15] = staal[15] + sensor.getCalibratedW();
      staal[16] = staal[16] + sensor.getCalibratedK();
      staal[17] = staal[17] + sensor.getCalibratedL();
      Serial.println("staal");
    }

    for (int i = 0; i <= 17; i++) {
      staal[i] = staal[i] / rep;
    }
    flagg[0] = 0;
    flagg[1] = 0;
  }

  else if (flagg[1] == 'T') {

    for (int i = 0; i < rep; i++) {
      sensor.disableIndicator();
      sensor.takeMeasurementsWithBulb();
      sensor.enableIndicator();
      delay(100);
      titan[0] = titan[0] + sensor.getCalibratedA();
      titan[1] = titan[1] + sensor.getCalibratedB();
      titan[2] = titan[2] + sensor.getCalibratedC();
      titan[3] = titan[3] + sensor.getCalibratedD();
      titan[4] = titan[4] + sensor.getCalibratedE();
      titan[5] = titan[5] + sensor.getCalibratedF();
      titan[6] = titan[6] + sensor.getCalibratedG();
      titan[7] = titan[7] + sensor.getCalibratedH();
      titan[8] = titan[8] + sensor.getCalibratedR();
      titan[9] = titan[9] + sensor.getCalibratedI();
      titan[10] = titan[10] + sensor.getCalibratedS();
      titan[11] = titan[11] + sensor.getCalibratedJ();
      titan[12] = titan[12] + sensor.getCalibratedT();
      titan[13] = titan[13] + sensor.getCalibratedU();
      titan[14] = titan[14] + sensor.getCalibratedV();
      titan[15] = titan[15] + sensor.getCalibratedW();
      titan[16] = titan[16] + sensor.getCalibratedK();
      titan[17] = titan[17] + sensor.getCalibratedL();
      Serial.println("titan");
    }

    for (int i = 0; i <= 17; i++) {
      titan[i] = titan[i] / rep;
    }
    flagg[0] = 0;
    flagg[1] = 0;
  }
}

void AS7265x_array_sammenlign(float D_aluminium[], float D_jern[], float D_staal[], float D_titan[], float D_bakgrunn[], char flagg[]) {
  static float Test[18];
  static float gjett[18];
  float alu = 0;
  float jer = 0;
  float sta = 0;
  float tit = 0;

  int rep = 50;

  if (flagg[0] == 'T') {
    if (flagg[1] == 'T') {
      for (int i = 0; i < rep; i++) {
        sensor.disableIndicator();
        sensor.takeMeasurementsWithBulb();
        sensor.enableIndicator();
        delay(100);
        Test[0] = Test[0] + sensor.getCalibratedA();
        Test[1] = Test[1] + sensor.getCalibratedB();
        Test[2] = Test[2] + sensor.getCalibratedC();
        Test[3] = Test[3] + sensor.getCalibratedD();
        Test[4] = Test[4] + sensor.getCalibratedE();
        Test[5] = Test[5] + sensor.getCalibratedF();
        Test[6] = Test[6] + sensor.getCalibratedG();
        Test[7] = Test[7] + sensor.getCalibratedH();
        Test[8] = Test[8] + sensor.getCalibratedR();
        Test[9] = Test[9] + sensor.getCalibratedI();
        Test[10] = Test[10] + sensor.getCalibratedS();
        Test[11] = Test[11] + sensor.getCalibratedJ();
        Test[12] = Test[12] + sensor.getCalibratedT();
        Test[13] = Test[13] + sensor.getCalibratedU();
        Test[14] = Test[14] + sensor.getCalibratedV();
        Test[15] = Test[15] + sensor.getCalibratedW();
        Test[16] = Test[16] + sensor.getCalibratedK();
        Test[17] = Test[17] + sensor.getCalibratedL();
      }

      for (int i = 0; i <= 17; i++) {
        Test[i] = (Test[i] / rep);
      }

      Serial.println("Testdata:     G_aluminium:         G_Jern:        G_staal:        G_titan:");

      for (int i = 0; i <= 17; i++) {
        gjett[i] = abs(1 - (abs(D_aluminium[i] - Test[i])) / D_aluminium[i]) * 100;
        
        Serial.print(Test[i]);
        Serial.print("           ");
        Serial.print(alu = alu + abs(D_aluminium[i] - Test[i]));
        Serial.print("           ");
        Serial.print(jer = jer + abs(D_jern[i] - Test[i]));
        Serial.print("         ");
        Serial.print(sta = sta + abs(D_staal[i] - Test[i]));
        Serial.print("        ");
        Serial.println(tit = tit + abs(D_titan[i] - Test[i]));
      }

      Serial.print("Testen gir sterkest avslag på: ");
      if(alu < jer && alu < sta && alu < tit){
        Serial.println("Aliminium");
      }
      else if(jer < alu && jer < sta && jer < tit){
        Serial.println("Jern");
      }
      else if(sta < alu && sta < jer && sta < tit){
        Serial.println("Stål");
      }
      else if(tit < alu && tit < jer && tit < sta){
        Serial.println("Titan");
      }

      flagg[0] = 0;
      flagg[1] = 0;
    }
  }
}

void loop() {
  static float aluminium[18];
  static float bakgrunn[18];
  static float jern[18];
  static float staal[18];
  static float titan[18];
  static char flagg[2];

  for (int i = 0; Serial.available(); i++) {
    flagg[i] = Serial.read();
    delay(5);
  }

  //AS7265x_array_datasett(aluminium, jern, staal, titan, bakgrunn, flagg);
  AS7265x();
  //AS7265x_array_sammenlign(aluminium, jern, staal, titan, bakgrunn, flagg);
}
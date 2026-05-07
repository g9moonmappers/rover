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
#include "SparkFun_AS7265X.h"
AS7265X sensor;
QWIICMUX myMux;

#include <Wire.h>

#define NUM_METALS 4
#define NUM_REFS_PER_METAL 9
#define NUM_CHANNELS 18

static bool Tflagg = 0;
static float bakgrunn[18];
static float Test[18];
int met[NUM_METALS];
static float METALS[NUM_METALS][NUM_REFS_PER_METAL][NUM_CHANNELS] = {

  //ALUMINIUM
  { { 116.17, 19.4, 114.66, 9.05, 72.22, 39.44, 25.73, 41.04, 27.27, 19.92, 7.73, 1.53, 2.11, 8.87, 14.8, 4.76, 2.49, 0.0 },
    { 68.64, -0.38, 1.23, -2.63, 0.0, 1.86, 1.78, 3.09, -3.57, 3.67, 0.85, 0.0, 0.16, -0.81, -0.11, 0.0, 0.5, 0.0 },
    { 63.16, 15.64, 125.55, 13.66, 75.9, 42.9, 0.51, 2.97, 7.48, 2.1, 0.94, 0.0, 0.49, 1.45, 0.99, 0.68, 0.89, 0.0 },
    { 4.67, -0.38, 29.32, -0.33, 19.16, 10.97, 17.33, 29.28, 86.64, 14.15, 5.28, 5.51, 5.36, 34.67, 85.81, 2.72, 1.49, 0.0 },
    { 3.54, -8.1, -3.86, -5.27, -6.63, -0.32, 0.13, -2.12, -4.95, -3.67, 0.19, 0.0, 0.32, -0.81, -0.11, 0.0, 0.1, 0.0 },
    { 6.28, -3.01, -0.88, -4.94, -5.16, -0.32, 6.88, 8.31, -0.35, 5.24, 1.51, 0.46, 0.0, 1.61, 8.88, 5.44, 0.3, 0.0 },
    { 1.13, 0.0, -6.67, 0.33, -3.68, -1.99, -3.57, -0.06, 5.87, -0.31, 0.0, 0.61, 0.49, 2.74, 6.69, 0.68, 0.1, 0.0 },
    { -0.81, -1.32, -0.53, -5.43, -7.37, -1.6, -3.57, -3.46, -7.48, -1.57, 0.0, 0.15, -0.32, -1.61, -1.21, 0.0, 0.5, 0.0 },
    { 2.09, -8.48, -11.41, -0.82, 1.47, 0.06, -0.64, -1.15, -1.5, -0.2, 0.09, -0.46, 0.32, -0.16, -0.99, 0.54, 0.5, 0.0 } },

  //JERN
  { { 22.72, -2.26, 24.06, -0.82, 18.42, 11.73, 8.03, 19.22, 29.8, 14.68, 20.39, 5.37, 2.45, 2.11, 12.58, 31.02, 4.08, 1.49 },
    { 25.3, -5.65, -7.55, -4.94, -6.63, -0.32, 0.64, 2.97, -2.19, 4.72, 1.62, 1.22, 0.31, 0.49, 0.65, 1.86, 0.68, 0.5 },
    { 17.88, -3.96, 16.15, 2.14, 20.49, 13.4, 0.64, 4.18, 10.47, 3.14, 9.49, 1.79, 1.07, 1.3, 3.23, 1.64, 0.68, 0.5 },
    { 2.09, -1.88, 8.96, -0.16, 8.11, 6.09, 5.61, 13.15, 32.56, 6.81, 26.65, 4.15, 2.91, 2.76, 15.97, 47.45, 2.72, 1.09 },
    { -4.19, -8.85, -5.09, -6.75, -8.84, -0.58, -0.38, -1.76, -4.72, -2.62, -1.21, 0.19, 0.49, 0.49, 0.0, 0.99, 0.0, 0.3 },
    { 1.45, -4.9, -3.16, -6.42, -8.84, -2.24, 3.44, 8.31, 2.19, 7.34, 1.21, 2.17, 0.61, 0.81, 4.84, 18.96, 2.72, 0.89 },
    { 0.16, -0.94, -9.83, -0.66, -5.9, -4.17, -0.51, 0.06, 6.33, -0.42, 7.87, 0.38, 1.3, 1.3, 4.52, 13.26, 0.14, 0.5 },
    { -0.64, -0.94, -1.23, -4.77, -9.58, -1.6, -3.57, -2.61, -5.18, -0.52, -1.21, 0.09, 0.15, 0.49, 0.0, 1.86, 0.0, 0.5 },
    { -0.64, -0.94, -1.23, -0.66, -2.95, -1.6, -0.89, -0.91, -1.04, -0.31, -1.21, 0.09, 0.15, 0.49, 0.0, -0.11, 0.0, 0.5 } },

  //STAAL
  { { 12.57, -6.41, 2.11, -2.96, 8.25, 6.99, 4.2, 7.46, 10.93, 4.72, 7.67, 1.7, 0.76, 1.14, 4.84, 11.51, 1.36, 0.7 },
    { 6.45, -11.11, -9.31, -5.6, -5.01, 0.83, 0.51, 1.76, -4.03, 2.1, -1.21, 0.66, 0.0, 0.49, 0.0, 0.68, 0.68, 0.5 },
    { 10.23, -4.14, 13.87, 3.05, 24.76, 17.63, 1.97, 4.55, 11.05, 2.1, 7.87, 1.08, 0.84, 0.65, 2.42, 2.19, 0.68, 0.0 },
    { 2.26, -2.26, 0.35, -0.49, 5.16, 5.07, 4.84, 7.21, 16.68, 2.62, 12.52, 1.79, 1.38, 1.3, 8.06, 20.93, 1.36, 0.89 },
    { -6.45, -10.74, -6.67, -6.91, -8.84, -0.83, -0.38, -2.12, -5.64, -3.77, 0.0, 0.0, 0.76, 0.49, 0.0, 0.11, 0.0, 0.3 },
    { 0.64, -5.46, -3.86, -7.24, -8.11, -2.24, 3.95, 5.64, -0.35, 4.19, 1.21, 1.22, 0.15, 0.49, 2.42, 10.41, 1.36, 0.5 },
    { -2.58, -2.83, -9.31, -0.82, -4.42, -3.91, -0.64, -0.79, 4.03, 0.0, 3.63, 0.0, 0.76, 0.65, 3.23, 9.1, 0.0, 0.5 },
    { -0.97, -0.94, -1.23, -5.6, -9.58, -1.73, -3.44, -3.21, -6.79, -1.05, -1.21, 0.0, 0.15, 0.49, 0.33, -0.33, 0.11, 0.5 },
    { -0.97, -0.94, -1.23, -0.66, -2.95, -1.6, -0.89, -0.91, -1.04, -0.31, -0.61, 0.0, 0.15, 0.49, 0.0, -0.33, 0.11, 0.5 } },

  //TITAN
  { { 57.76, 20.16, 168.92, 18.69, 133.23, 65.99, 13.69, 21.64, 83.53, 11.53, 60.78, 4.19, 5.43, 4.22, 33.06, 82.42, 4.08, 0.99 },
    { 108.52, 1.7, 9.13, -2.55, 6.93, 5.58, 3.63, 8.31, -1.15, 10.59, -0.81, 1.65, -0.08, 0.0, 0.81, -1.1, 1.36, 0.0 },
    { 137.36, 17.52, 85.34, 10.78, 56.59, 34.95, 6.69, 15.58, 31.07, 7.97, 18.78, 3.53, 2.06, 1.46, 6.61, 3.73, 3.4, 0.8 },
    { 1.21, 0.0, 39.51, 1.73, 25.5, 14.43, 24.01, 47.71, 42.8, 26.73, 27.06, 7.77, 2.98, 2.44, 17.74, 39.45, 4.76, 1.99 },
    { -1.85, -11.49, -6.5, -7.49, -8.4, -1.22, -0.7, -2.0, -6.21, -3.25, -1.01, -0.14, -0.08, 0.0, -0.48, -0.66, 0.0, -0.6 },
    { 8.94, -3.2, -1.23, -6.01, -3.39, 1.99, 15.35, 20.91, 8.74, 13.73, 2.83, 3.63, 0.69, 0.81, 7.26, 29.37, 3.4, 0.99 },
    { -1.05, -2.45, -6.15, -0.58, -1.47, -1.47, -1.72, -1.15, -5.06, -0.52, -0.81, -0.05, 0.84, 0.81, 0.0, 1.32, 0.68, 0.0 },
    { -1.05, -0.94, -0.7, -6.01, -8.99, -1.47, -1.72, -1.79, -5.06, -0.52, -0.81, -0.24, 0.08, 0.81, 0.0, -0.88, 0.68, 0.0 },
    { 3.63, -9.61, -7.02, 0.08, 8.55, 4.17, -0.57, -0.79, -0.46, 0.61, 0.61, -0.24, -0.08, 0.0, 0.0, -0.88, 0.0, 0.0 } }
};


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
    sensor.setIntegrationCycles(30);
    sensor.setGain(AS7265X_GAIN_64X);
  }
  Wire.setClock(400000);
}

void AS7265x(float bakgrunn[]) {
  float sum[18];
  for (int i = 0; i < 18; i++) {
    sum[i] = 0;
  }
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < 2; i++) {
      myMux.setPort(i);
      sensor.disableIndicator();

      sensor.takeMeasurementsWithBulb();

      sum[0] += sensor.getCalibratedA();  //410nm
      sum[1] += sensor.getCalibratedB();  //435nm
      sum[2] += sensor.getCalibratedC();  //460nm
      sum[3] += sensor.getCalibratedD();  //485nm
      sum[4] += sensor.getCalibratedE();  //510nm
      sum[5] += sensor.getCalibratedF();  //535nm

      sum[6] += sensor.getCalibratedG();   //560nm
      sum[7] += sensor.getCalibratedH();   //585nm
      sum[8] += sensor.getCalibratedR();   //610nm
      sum[9] += sensor.getCalibratedI();   //645nm
      sum[10] += sensor.getCalibratedS();  //680nm
      sum[11] += sensor.getCalibratedJ();  //705nm

      sum[12] += sensor.getCalibratedT();  //730nm
      sum[13] += sensor.getCalibratedU();  //760nm
      sum[14] += sensor.getCalibratedV();  //810nm
      sum[15] += sensor.getCalibratedW();  //860nm
      sum[16] += sensor.getCalibratedK();  //900nm
      sum[17] += sensor.getCalibratedL();  //940nm
    }
  }
  for (int i = 0; i < 18; i++) {
    sum[i] /= 6;
    Serial.print(sum[i] - bakgrunn[i]);
    Serial.print(",");
  }
  Serial.println();
}

void AS7265x_array_datasett(float bakgrunn[], char flagg[]) {
  int rep = 5;
  static bool Bflagg = 0;

  if (flagg[0] == 'S') {
    for (int i = 0; i < 2; i++) {
      myMux.setPort(i);
      sensor.disableIndicator();
    }

    if (flagg[1] == 'B') {

      for (int i = 0; i <= 17; i++) {
        bakgrunn[i] = 0;
      }

      for (int i = 0; i < rep; i++) {
        for (int j = 0; j < 2; j++) {
          myMux.setPort(j);
          sensor.takeMeasurementsWithBulb();
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

          sensor.takeMeasurements();
          delay(100);
          bakgrunn[0] = bakgrunn[0] - sensor.getCalibratedA();
          bakgrunn[1] = bakgrunn[1] - sensor.getCalibratedB();
          bakgrunn[2] = bakgrunn[2] - sensor.getCalibratedC();
          bakgrunn[3] = bakgrunn[3] - sensor.getCalibratedD();
          bakgrunn[4] = bakgrunn[4] - sensor.getCalibratedE();
          bakgrunn[5] = bakgrunn[5] - sensor.getCalibratedF();
          bakgrunn[6] = bakgrunn[6] - sensor.getCalibratedG();
          bakgrunn[7] = bakgrunn[7] - sensor.getCalibratedH();
          bakgrunn[8] = bakgrunn[8] - sensor.getCalibratedR();
          bakgrunn[9] = bakgrunn[9] - sensor.getCalibratedI();
          bakgrunn[10] = bakgrunn[10] - sensor.getCalibratedS();
          bakgrunn[11] = bakgrunn[11] - sensor.getCalibratedJ();
          bakgrunn[12] = bakgrunn[12] - sensor.getCalibratedT();
          bakgrunn[13] = bakgrunn[13] - sensor.getCalibratedU();
          bakgrunn[14] = bakgrunn[14] - sensor.getCalibratedV();
          bakgrunn[15] = bakgrunn[15] - sensor.getCalibratedW();
          bakgrunn[16] = bakgrunn[16] - sensor.getCalibratedK();
          bakgrunn[17] = bakgrunn[17] - sensor.getCalibratedL();
        }
      }

      for (int i = 0; i <= 17; i++) {
        bakgrunn[i] /= (rep * 2);
      }

      Bflagg = 1;
    } else if (flagg[1] == 'A' && Bflagg == 1) {
      if (flagg[2] == '0' || flagg[2] == '1' || flagg[2] == '2' || flagg[2] == '3' || flagg[2] == '4' || flagg[2] == '5' || flagg[2] == '6' || flagg[2] == '7' || flagg[2] == '8') {
        for (int i = 0; i < rep; i++) {
          for (int j = 0; j < 2; j++) {
            myMux.setPort(j);
            sensor.takeMeasurementsWithBulb();
            delay(100);
            METALS[0][flagg[2]][0] += sensor.getCalibratedA();
            METALS[0][flagg[2]][1] += sensor.getCalibratedB();
            METALS[0][flagg[2]][2] += sensor.getCalibratedC();
            METALS[0][flagg[2]][3] += sensor.getCalibratedD();
            METALS[0][flagg[2]][4] += sensor.getCalibratedE();
            METALS[0][flagg[2]][5] += sensor.getCalibratedF();
            METALS[0][flagg[2]][6] += sensor.getCalibratedG();
            METALS[0][flagg[2]][7] += sensor.getCalibratedH();
            METALS[0][flagg[2]][8] += sensor.getCalibratedR();
            METALS[0][flagg[2]][9] += sensor.getCalibratedI();
            METALS[0][flagg[2]][10] += sensor.getCalibratedS();
            METALS[0][flagg[2]][11] += sensor.getCalibratedJ();
            METALS[0][flagg[2]][12] += sensor.getCalibratedT();
            METALS[0][flagg[2]][13] += sensor.getCalibratedU();
            METALS[0][flagg[2]][14] += sensor.getCalibratedV();
            METALS[0][flagg[2]][15] += sensor.getCalibratedW();
            METALS[0][flagg[2]][16] += sensor.getCalibratedK();
            METALS[0][flagg[2]][17] += sensor.getCalibratedL();

            sensor.takeMeasurements();
            delay(100);
            METALS[0][flagg[2]][0] -= sensor.getCalibratedA();
            METALS[0][flagg[2]][1] -= sensor.getCalibratedB();
            METALS[0][flagg[2]][2] -= sensor.getCalibratedC();
            METALS[0][flagg[2]][3] -= sensor.getCalibratedD();
            METALS[0][flagg[2]][4] -= sensor.getCalibratedE();
            METALS[0][flagg[2]][5] -= sensor.getCalibratedF();
            METALS[0][flagg[2]][6] -= sensor.getCalibratedG();
            METALS[0][flagg[2]][7] -= sensor.getCalibratedH();
            METALS[0][flagg[2]][8] -= sensor.getCalibratedR();
            METALS[0][flagg[2]][9] -= sensor.getCalibratedI();
            METALS[0][flagg[2]][10] -= sensor.getCalibratedS();
            METALS[0][flagg[2]][11] -= sensor.getCalibratedJ();
            METALS[0][flagg[2]][12] -= sensor.getCalibratedT();
            METALS[0][flagg[2]][13] -= sensor.getCalibratedU();
            METALS[0][flagg[2]][14] -= sensor.getCalibratedV();
            METALS[0][flagg[2]][15] -= sensor.getCalibratedW();
            METALS[0][flagg[2]][16] -= sensor.getCalibratedK();
            METALS[0][flagg[2]][17] -= sensor.getCalibratedL();
          }
          for (int i = 0; i <= 17; i++) {
            METALS[0][flagg[2]][i] = (METALS[0][flagg[2]][i] / (rep * 2)) - bakgrunn[i];
          }
        }
      } else {
        Serial.println("tom for plass... Maks for hvert metall er 9 samples! 0-8");
      }
    } else if (flagg[1] == 'J' && Bflagg == 1) {
      if (flagg[2] == '0' || flagg[2] == '1' || flagg[2] == '2' || flagg[2] == '3' || flagg[2] == '4' || flagg[2] == '5' || flagg[2] == '6' || flagg[2] == '7' || flagg[2] == '8') {
        for (int i = 0; i < rep; i++) {
          for (int j = 0; j < 2; j++) {
            myMux.setPort(j);
            sensor.takeMeasurementsWithBulb();
            delay(100);
            METALS[1][flagg[2]][0] += sensor.getCalibratedA();
            METALS[1][flagg[2]][1] += sensor.getCalibratedB();
            METALS[1][flagg[2]][2] += sensor.getCalibratedC();
            METALS[1][flagg[2]][3] += sensor.getCalibratedD();
            METALS[1][flagg[2]][4] += sensor.getCalibratedE();
            METALS[1][flagg[2]][5] += sensor.getCalibratedF();
            METALS[1][flagg[2]][6] += sensor.getCalibratedG();
            METALS[1][flagg[2]][7] += sensor.getCalibratedH();
            METALS[1][flagg[2]][8] += sensor.getCalibratedR();
            METALS[1][flagg[2]][9] += sensor.getCalibratedI();
            METALS[1][flagg[2]][10] += sensor.getCalibratedS();
            METALS[1][flagg[2]][11] += sensor.getCalibratedJ();
            METALS[1][flagg[2]][12] += sensor.getCalibratedT();
            METALS[1][flagg[2]][13] += sensor.getCalibratedU();
            METALS[1][flagg[2]][14] += sensor.getCalibratedV();
            METALS[1][flagg[2]][15] += sensor.getCalibratedW();
            METALS[1][flagg[2]][16] += sensor.getCalibratedK();
            METALS[1][flagg[2]][17] += sensor.getCalibratedL();

            sensor.takeMeasurements();
            delay(100);
            METALS[1][flagg[2]][0] -= sensor.getCalibratedA();
            METALS[1][flagg[2]][1] -= sensor.getCalibratedB();
            METALS[1][flagg[2]][2] -= sensor.getCalibratedC();
            METALS[1][flagg[2]][3] -= sensor.getCalibratedD();
            METALS[1][flagg[2]][4] -= sensor.getCalibratedE();
            METALS[1][flagg[2]][5] -= sensor.getCalibratedF();
            METALS[1][flagg[2]][6] -= sensor.getCalibratedG();
            METALS[1][flagg[2]][7] -= sensor.getCalibratedH();
            METALS[1][flagg[2]][8] -= sensor.getCalibratedR();
            METALS[1][flagg[2]][9] -= sensor.getCalibratedI();
            METALS[1][flagg[2]][10] -= sensor.getCalibratedS();
            METALS[1][flagg[2]][11] -= sensor.getCalibratedJ();
            METALS[1][flagg[2]][12] -= sensor.getCalibratedT();
            METALS[1][flagg[2]][13] -= sensor.getCalibratedU();
            METALS[1][flagg[2]][14] -= sensor.getCalibratedV();
            METALS[1][flagg[2]][15] -= sensor.getCalibratedW();
            METALS[1][flagg[2]][16] -= sensor.getCalibratedK();
            METALS[1][flagg[2]][17] -= sensor.getCalibratedL();
          }
          for (int i = 0; i <= 17; i++) {
            METALS[0][flagg[2]][i] = (METALS[0][flagg[2]][i] / (rep * 2)) - bakgrunn[i];
          }
        }
      } else {
        Serial.println("tom for plass... Maks for hvert metall er 9 samples! 0-8");
      }
    } else if (flagg[1] == 'S' && Bflagg == 1) {
      if (flagg[2] == '0' || flagg[2] == '1' || flagg[2] == '2' || flagg[2] == '3' || flagg[2] == '4' || flagg[2] == '5' || flagg[2] == '6' || flagg[2] == '7' || flagg[2] == '8') {
        for (int i = 0; i < rep; i++) {
          for (int j = 0; j < 2; j++) {
            myMux.setPort(j);
            sensor.takeMeasurementsWithBulb();
            delay(100);
            METALS[2][flagg[2]][0] += sensor.getCalibratedA();
            METALS[2][flagg[2]][1] += sensor.getCalibratedB();
            METALS[2][flagg[2]][2] += sensor.getCalibratedC();
            METALS[2][flagg[2]][3] += sensor.getCalibratedD();
            METALS[2][flagg[2]][4] += sensor.getCalibratedE();
            METALS[2][flagg[2]][5] += sensor.getCalibratedF();
            METALS[2][flagg[2]][6] += sensor.getCalibratedG();
            METALS[2][flagg[2]][7] += sensor.getCalibratedH();
            METALS[2][flagg[2]][8] += sensor.getCalibratedR();
            METALS[2][flagg[2]][9] += sensor.getCalibratedI();
            METALS[2][flagg[2]][10] += sensor.getCalibratedS();
            METALS[2][flagg[2]][11] += sensor.getCalibratedJ();
            METALS[2][flagg[2]][12] += sensor.getCalibratedT();
            METALS[2][flagg[2]][13] += sensor.getCalibratedU();
            METALS[2][flagg[2]][14] += sensor.getCalibratedV();
            METALS[2][flagg[2]][15] += sensor.getCalibratedW();
            METALS[2][flagg[2]][16] += sensor.getCalibratedK();
            METALS[2][flagg[2]][17] += sensor.getCalibratedL();

            sensor.takeMeasurements();
            delay(100);
            METALS[2][flagg[2]][0] -= sensor.getCalibratedA();
            METALS[2][flagg[2]][1] -= sensor.getCalibratedB();
            METALS[2][flagg[2]][2] -= sensor.getCalibratedC();
            METALS[2][flagg[2]][3] -= sensor.getCalibratedD();
            METALS[2][flagg[2]][4] -= sensor.getCalibratedE();
            METALS[2][flagg[2]][5] -= sensor.getCalibratedF();
            METALS[2][flagg[2]][6] -= sensor.getCalibratedG();
            METALS[2][flagg[2]][7] -= sensor.getCalibratedH();
            METALS[2][flagg[2]][8] -= sensor.getCalibratedR();
            METALS[2][flagg[2]][9] -= sensor.getCalibratedI();
            METALS[2][flagg[2]][10] -= sensor.getCalibratedS();
            METALS[2][flagg[2]][11] -= sensor.getCalibratedJ();
            METALS[2][flagg[2]][12] -= sensor.getCalibratedT();
            METALS[2][flagg[2]][13] -= sensor.getCalibratedU();
            METALS[2][flagg[2]][14] -= sensor.getCalibratedV();
            METALS[2][flagg[2]][15] -= sensor.getCalibratedW();
            METALS[2][flagg[2]][16] -= sensor.getCalibratedK();
            METALS[2][flagg[2]][17] -= sensor.getCalibratedL();
          }
          for (int i = 0; i <= 17; i++) {
            METALS[0][flagg[2]][i] = (METALS[0][flagg[2]][i] / (rep * 2)) - bakgrunn[i];
          }
        }
      } else {
        Serial.println("tom for plass... Maks for hvert metall er 9 samples! 0-8");
      }
    } else if (flagg[1] == 'T' && Bflagg == 1) {
      if (flagg[2] == '0' || flagg[2] == '1' || flagg[2] == '2' || flagg[2] == '3' || flagg[2] == '4' || flagg[2] == '5' || flagg[2] == '6' || flagg[2] == '7' || flagg[2] == '8') {
        for (int i = 0; i < rep; i++) {
          for (int j = 0; j < 2; j++) {
            myMux.setPort(j);
            sensor.takeMeasurementsWithBulb();
            delay(100);
            METALS[3][flagg[2]][0] += sensor.getCalibratedA();
            METALS[3][flagg[2]][1] += sensor.getCalibratedB();
            METALS[3][flagg[2]][2] += sensor.getCalibratedC();
            METALS[3][flagg[2]][3] += sensor.getCalibratedD();
            METALS[3][flagg[2]][4] += sensor.getCalibratedE();
            METALS[3][flagg[2]][5] += sensor.getCalibratedF();
            METALS[3][flagg[2]][6] += sensor.getCalibratedG();
            METALS[3][flagg[2]][7] += sensor.getCalibratedH();
            METALS[3][flagg[2]][8] += sensor.getCalibratedR();
            METALS[3][flagg[2]][9] += sensor.getCalibratedI();
            METALS[3][flagg[2]][10] += sensor.getCalibratedS();
            METALS[3][flagg[2]][11] += sensor.getCalibratedJ();
            METALS[3][flagg[2]][12] += sensor.getCalibratedT();
            METALS[3][flagg[2]][13] += sensor.getCalibratedU();
            METALS[3][flagg[2]][14] += sensor.getCalibratedV();
            METALS[3][flagg[2]][15] += sensor.getCalibratedW();
            METALS[3][flagg[2]][16] += sensor.getCalibratedK();
            METALS[3][flagg[2]][17] += sensor.getCalibratedL();

            sensor.takeMeasurements();
            delay(100);
            METALS[3][flagg[2]][0] -= sensor.getCalibratedA();
            METALS[3][flagg[2]][1] -= sensor.getCalibratedB();
            METALS[3][flagg[2]][2] -= sensor.getCalibratedC();
            METALS[3][flagg[2]][3] -= sensor.getCalibratedD();
            METALS[3][flagg[2]][4] -= sensor.getCalibratedE();
            METALS[3][flagg[2]][5] -= sensor.getCalibratedF();
            METALS[3][flagg[2]][6] -= sensor.getCalibratedG();
            METALS[3][flagg[2]][7] -= sensor.getCalibratedH();
            METALS[3][flagg[2]][8] -= sensor.getCalibratedR();
            METALS[3][flagg[2]][9] -= sensor.getCalibratedI();
            METALS[3][flagg[2]][10] -= sensor.getCalibratedS();
            METALS[3][flagg[2]][11] -= sensor.getCalibratedJ();
            METALS[3][flagg[2]][12] -= sensor.getCalibratedT();
            METALS[3][flagg[2]][13] -= sensor.getCalibratedU();
            METALS[3][flagg[2]][14] -= sensor.getCalibratedV();
            METALS[3][flagg[2]][15] -= sensor.getCalibratedW();
            METALS[3][flagg[2]][16] -= sensor.getCalibratedK();
            METALS[3][flagg[2]][17] -= sensor.getCalibratedL();
          }
          for (int i = 0; i <= 17; i++) {
            METALS[0][flagg[2]][i] = (METALS[0][flagg[2]][i] / (rep * 2)) - bakgrunn[i];
          }
        }
      } else {
        Serial.println("tom for plass... Maks for hvert metall er 9 samples! 0-8");
      }
    } else {
      Serial.println("Feil samplingskode!!! For å sample skriv samplingskoden din hvor x er hvilken minneplass du ønsker å bruke. Dette krever at bakgrunnen er samplet først!");
      Serial.println("SAx = Aluminium");
      Serial.println("SJx = Jern");
      Serial.println("SSx = Stål");
      Serial.println("STx = Titan");
      Serial.println("SB = Bakgrunn");
    }

    flagg[0] = 0;
    flagg[1] = 0;
    flagg[2] = 0;

    for (int i = 0; i < 2; i++) {
      myMux.setPort(i);
      sensor.enableIndicator();
    }
  }
}

void AS7265x_Test(float Test[], float D_bakgrunn[], char flagg[]) {
  int rep = 5;

  if ((flagg[0] == 'T') && (flagg[1] == 'T')) {
    Tflagg = 1;
    for (int i = 0; i <= 17; i++) {
      Test[i] = 0;
    }

    for (int i = 0; i < 2; i++) {
      myMux.setPort(i);
      sensor.disableIndicator();
    }

    for (int i = 0; i < rep; i++) {
      for (int j = 0; j < 2; j++) {
        myMux.setPort(j);

        sensor.takeMeasurementsWithBulb();
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

        sensor.takeMeasurements();
        delay(100);
        Test[0] = Test[0] - sensor.getCalibratedA();
        Test[1] = Test[1] - sensor.getCalibratedB();
        Test[2] = Test[2] - sensor.getCalibratedC();
        Test[3] = Test[3] - sensor.getCalibratedD();
        Test[4] = Test[4] - sensor.getCalibratedE();
        Test[5] = Test[5] - sensor.getCalibratedF();
        Test[6] = Test[6] - sensor.getCalibratedG();
        Test[7] = Test[7] - sensor.getCalibratedH();
        Test[8] = Test[8] - sensor.getCalibratedR();
        Test[9] = Test[9] - sensor.getCalibratedI();
        Test[10] = Test[10] - sensor.getCalibratedS();
        Test[11] = Test[11] - sensor.getCalibratedJ();
        Test[12] = Test[12] - sensor.getCalibratedT();
        Test[13] = Test[13] - sensor.getCalibratedU();
        Test[14] = Test[14] - sensor.getCalibratedV();
        Test[15] = Test[15] - sensor.getCalibratedW();
        Test[16] = Test[16] - sensor.getCalibratedK();
        Test[17] = Test[17] - sensor.getCalibratedL();
      }
    }
    for (int i = 0; i < 2; i++) {
      myMux.setPort(i);
      sensor.enableIndicator();
    }

    for (int i = 0; i <= 17; i++) {
      Test[i] = (Test[i] / (rep * 2)) - D_bakgrunn[i];
    }

    flagg[0] = 0;
    flagg[1] = 0;
    flagg[2] = 0;
  }
}

void AS7265X_SAM(float Test[], int met[]) {
  float best = 3;
  int Smet = 4;
  for (int i = 0; i < NUM_METALS; i++) {
    for (int j = 0; j < NUM_REFS_PER_METAL; j++) {
      float dot = 0, normD = 0, normT = 0, res = 0;
      for (int k = 0; k < NUM_CHANNELS; k++) {
        dot += METALS[i][j][k] * Test[k];
        normD += METALS[i][j][k] * METALS[i][j][k];
        normT += Test[k] * Test[k];
      }
      res = acos(dot / (sqrt(normD) * sqrt(normT)));
      if (best > res) {
        best = res;
        Smet = i;
      }
    }
  }
  if (Smet > NUM_METALS) {
    Serial.println("ingen metaller");
  } else {
    met[Smet]++;
  }
}

void AS7265X_CORRELATION(float Test[], float bakgrunn[], int met[]) {
  float avgT = 0, best = 0;
  int Smet = 4;
  for (int i = 0; i < NUM_CHANNELS; i++) {
    avgT += Test[i] - bakgrunn[i];
  }
  avgT /= NUM_CHANNELS;

  for (int i = 0; i < NUM_METALS; i++) {
    for (int j = 0; j < NUM_REFS_PER_METAL; j++) {
      float res = 0, avg = 0;
      for (int k = 0; k < NUM_CHANNELS; k++) {
        avg += METALS[i][j][k] - bakgrunn[k];
      }
      avg /= NUM_CHANNELS;

      float num = 0, denoM = 0, denoT = 0;

      for (int k = 0; k < NUM_CHANNELS; k++) {
        float r = METALS[i][j][k] - avg;
        float t = Test[k] - avgT;
        num += r * t;
        denoM += r * r;
        denoT += t * t;
      }
      res = num / (sqrt(denoM) * sqrt(denoT));
      if (best < res) {
        best = res;
        met = i;
      }
    }
  }

  if (Smet > NUM_METALS) {
    Serial.println("ingen metaller");
  } else {
    met[Smet]++;
  }
}

void AS7265X_RATIO(float Test[], int met[]) {
  float best = 0;
  int Smet = 4;
  for (int i = 0; i < NUM_METALS; i++) {
    for (int j = 0; j < NUM_REFS_PER_METAL; j++) {
      float Tratio = 0, Mratio = 0, res = 0;
      Mratio = (METALS[i][j][6] - METALS[i][j][8]) / (METALS[i][j][6] + METALS[i][j][8]);
      Tratio = (Test[6] - Test[8]) / (Test[6] + Test[8]);
      res = abs(Mratio - Tratio);
      if (best > res) {
        best = res;
        met = i;
      }
    }
  }
  if (Smet > NUM_METALS) {
    Serial.println("ingen metaller");
  } else {
    met[Smet]++;
  }
}

void AS7265X_DERIVATIVE(float Test[], int met[]) {
  float best = 0;
  met = 4;
  for (int i = 0; i < NUM_METALS; i++) {
    for (int j = 0; j < NUM_REFS_PER_METAL; j++) {
      float dot = 0, normD = 0, normT = 0, res = 0;
      float Tout[18], Mout[18];
      for (int k = 0; k < NUM_CHANNELS; k++) {
        Tout[k] = Test[k + 1] - Test[k];
        Mout[k] = METALS[i][j][k + 1] - METALS[i][j][k];
      }
      for (int k = 0; k < NUM_CHANNELS; k++) {
        dot += Tout[k] * Mout[k];
        normD += Mout[k] * Mout[k];
        normT += Tout[k] * Tout[k];
      }
      res = dot / (sqrt(normD) * sqrt(normT));
      if (best < res) {
        best = res;
        met = i;
      }
    }
  }
  if (met == 0) {
    Serial.println("aluminium er best");
  } else if (met == 1) {
    Serial.println("jern er best");
  } else if (met == 2) {
    Serial.println("jern er best");
  } else if (met == 3) {
    Serial.println("jern er best");
  } else {
    Serial.println("feil");
  }
}

void AS7265X_STEMME() {
}

void loop() {
  static char flagg[3];

  for (int i = 0; Serial.available(); i++) {
    flagg[i] = Serial.read();
    delay(5);
  }

  //AS7265x(bakgrunn);
  AS7265x_array_datasett(bakgrunn, flagg);
  AS7265x_Test(Test, bakgrunn, flagg);
  if (Tflagg == 1) {
    for (int i = 0; i < NUM_METALS; i++) {
      met[i] = 4;
    }
    Serial.println("test");
    AS7265X_CORRELATION(Test, bakgrunn, met);
    AS7265X_SAM(Test, met);
    AS7265X_RATIO(Test, met);
    AS7265X_DERIVATIVE(Test, met);
    AS7265X_STEMME();
    Tflagg = 0;
  }
}
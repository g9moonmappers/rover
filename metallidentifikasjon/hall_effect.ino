/*koden er basert på ekesmeplkode1 for bibloteket som kan finnes i
github her: https://github.com/sparkfun/SparkFun_TMAG5273_Arduino_Library

produsert av Sparkfun Electronics Inc
SPDX-License-Identifier: MIT
*/

#include "SparkFun_TMAG5273_Arduino_Library.h"
#include <Wire.h>
TMAG5273 sensor;
uint8_t i2cAddress = TMAG5273_I2C_ADDRESS_INITIAL;

void setup(){
  delay(1000);
  Wire.begin();
  Serial.begin(9600);
  if(sensor.begin(i2cAddress, Wire) == 1){
    Serial.println("Begin");
  }

  else{
    Serial.println("Device failed to setup - Freezing code.");
    while (1);
  }
}

void loop(){

  if(sensor.getMagneticChannel() != 0){
    static bool flag1 = 0;
    static float cal = 0;
    if(flag1 == 0){
      for(int i = 0; i<50; i++){
        delay(10);
        float magY = sensor.getYData();
        cal = cal + magY;
      }
      cal = abs(cal/50);
      flag1 = 1;
    }

    float magY = sensor.getYData();
    magY = abs(magY);

    Serial.print("cal= ");
    Serial.print(cal);
    Serial.print("  rådata= ");
    Serial.print(magY);
    Serial.print("mT  kalibrert= ");
    Serial.print(magY-cal);
    Serial.print("mT  Signal= ");
    Serial.print(round(magY-cal));
    Serial.println();
  }

  else{
    Serial.println("Mag Channels disabled, stopping..");
    while (1);
  }
}
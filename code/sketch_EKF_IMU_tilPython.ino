/* SJEKKLISTE:
  
  - Hvordan sende/lese data fra motorene?
  - Lese data fra sensorer/ hvordan lese ?
  - Hvordan sette ønsket posisjon 
  - Tune sensorstøy parametere (datablad)
  - Tune systemtøy (prøve seg frem / testing)

  ANDRE TING:

  - sampling time

*/



#include <Kalman.h>
  using namespace BLA;
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

#define OUTPUT_READABLE_ACCELGYRO
//#define OUTPUT_BINARY_ACCELGYRO

int16_t gx_raw, gy_raw, gz_raw;
int16_t ax_raw, ay_raw, az_raw;

float wl_received = 0.0;
float wr_received = 0.0;
float x_received  = 0.0;
float y_received  = 0.0;

bool new_position = false;



  #define Nstate 3 //posisjon (x y), vinkel teta
  #define Nobs 3 //posisjon (x y), vinkel teta

  #define n_x 0.01 //10 avvik på posisjon
  #define n_y 0.01 //10 avvik på posisjon
  #define n_teta 0.01 //1 cm avvik på teta (endre?)

  //systemstøy
  #define m_x 0.1 
  #define m_y 0.1 
  #define m_teta 0.8



  BLA::Matrix<Nobs> obs; //obersvasjonsvektor

  KALMAN<Nstate,Nobs> K; //Kalman fitleret
  unsigned long T; //nåværende tid
  float DT; //tid mellom oppdateringer i filteret.



  void setup() {


  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(); 
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(38400); 
  Serial1.begin(115200);

  Serial.setTimeout(10);
  Serial1.setTimeout(10);

  mpu.initialize();

  // IMU offset: hent fra kalibreringskode
  Serial.println("Updating internal sensor offsets...\n");
  mpu.setXAccelOffset(-4567);
  mpu.setYAccelOffset(-2279);
  mpu.setZAccelOffset(3467); 
  mpu.setXGyroOffset(-80);  
  mpu.setYGyroOffset(13);  
  mpu.setZGyroOffset(16); 




    K.F = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0,
    };

    //vi måler alle tilstander i systemet
    K.H = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0,
    };

    //K.H = {0.0, 0.0, 1.0};

    K.R = {
      n_x*n_x, 0.0, 0.0,
      0.0, n_y*n_y, 0.0,
      0.0, 0.0, n_teta*n_teta,
    };

    //K.R = {n_teta*n_teta};

    K.Q = {
      m_x*m_x, 0.0, 0.0,
      0.0, m_y*m_y, 0.0,
      0.0, 0.0, m_teta*m_teta,
    };

    T = micros();


  }

  void loop() {
    read_serial_data();

    //tid
    DT = (micros()-T)/1000000.0; 
    T = micros();

    float teta = K.x(2);


    //leser sensordata:
    //obs(0) = read_teta(DT);

    if (new_position) {
      obs(0) = read_x();
      obs(1) = read_y();
      obs(2) = read_teta(DT);
      new_position = false;
    }
    else {
      obs(0) = K.x(0);
      obs(1) = K.x(1);
      obs(2) = read_teta(DT);      
    }


    float wl_o = get_wl_o();
    float wr_o = get_wr_o();

    //gjør om til fra leste vinkelhastigheter til V og w
    float V = (0.05/2) * (wr_o + wl_o);
    float w = (0.05/0.29) * (wr_o - wl_o);

    //tar i bruk jacobian av systemetmodellen
    K.F = { 
      1.0, 0.0, -V*sin(teta)*DT,
      0.0, 1.0, V*cos(teta)*DT,
      0.0, 0.0, 1,
    };

    BLA::Matrix<Nstate> x_pred;
    x_pred(0) = K.x(0) + V * cos(teta) * DT;
    x_pred(1) = K.x(1) + V * sin(teta) * DT;
    x_pred(2) = K.x(2) + w * DT;
    
    K.x = x_pred; // setter inn den ikke linjære modellen

    K.update(obs);


    //sender til jetson nano:


    Serial.print("S:");      // sync marker
    Serial.print(K.x(0));
    Serial.print(",");
    Serial.print(K.x(1));
    Serial.print(",");
    Serial.println(K.x(2));
    //Serial.print(",");
    //Serial.println(obs(0));
  }


  void SKIFTE_TIL_VINKELHASITGHET(float V,float w, float &wr, float &wl){
    //funksjon for å gi vinkelhastighetene til hjulene

    float r = 0.05;//radiusen på hjula
    float B = 0.29;//bredden på roboten

    wr = (V/r) - (w*B/(2.0*r));
    wl = (V/r) + (w*B/(2.0*r));

  }

  float read_teta(float DT){ 
     mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);

     float gz_rad = gz_raw * 1.0/131.0 * M_PI/180;

     static float teta = 0.0;
     teta += gz_rad * DT;

     return teta;
  }




void read_serial_data() {
  //leser hastighetenee fra hjulene og bruker de som inngang i KF
  if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      
      int c1 = line.indexOf(',');
      int c2 = line.indexOf(',', c1 + 1);
      int c3 = line.indexOf(',', c2 + 1);
      
      if (c1 != -1 && c2 != -1 && c3 != -1) {
          wl_received = line.substring(0, c1).toFloat();
          wr_received = line.substring(c1 + 1, c2).toFloat();
          x_received  = line.substring(c2 + 1, c3).toFloat();
          y_received  = line.substring(c3 + 1).toFloat();
          new_position = true; 
      }
  }

}


  //leser jetson nano for hasstigheter og posisjon sensordata
//  if (Serial1.available()) {
//    String line = Serial1.readStringUntil('\n');
//    int comma = line.indexOf(',');
//    if (comma != -1) {
//      x_received = line.substring(0, comma).toFloat();
//      y_received = line.substring(comma + 1).toFloat();
//    }
//  }
//}

float get_wl_o() { return wl_received; }
float get_wr_o() { return wr_received; }
float read_x()   { return x_received; }
float read_y()   { return y_received; }
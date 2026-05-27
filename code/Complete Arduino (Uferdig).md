#Add all our arduino code in here

this should be the complete code used in our rover system

```
/* SJEKKLISTE:
  
  - Tune sensorstøy parametere (datablad + justeringer)
  - Tune systemtøy (prøve seg frem / testing)

  ANDRE TING:

  - sampling time
  - lavpass filter for IMU?

*/



#include <Kalman.h>
  using namespace BLA;
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

MPU6050 mpu;

#define OUTPUT_READABLE_ACCELGYRO
//#define OUTPUT_BINARY_ACCELGYRO

int16_t gx_raw, gy_raw, gz_raw;
int16_t ax_raw, ay_raw, az_raw;

float wl_received = 0.0;
float wr_received = 0.0;
float x_uwb;
float y_uwb;


bool new_position = false;
bool new_metal = false;

float Vb; //batteri volt
float Ib; //batteri strøm
float Til; //tilstand
float M; //metall ID signal (0 elelr 1)




  #define Nstate 3 //posisjon (x y), vinkel teta
  #define Nobs 3 //posisjon (x y), vinkel teta


  //sensorstøy
  #define n_x 0.1 //10 avvik på posisjon
  #define n_y 0.1 //10 avvik på posisjon
  #define n_teta 0.1 //0.01 rad avvik på teta (endre?)

  //systemstøy
  #define m_x 0.1
  #define m_y 0.1
  #define m_teta 0.1

  #define sample_time 20 //ms => 50 hz


  BLA::Matrix<Nobs> obs; //obersvasjonsvektor

  KALMAN<Nstate,Nobs> K; //Kalman fitleret

  //koordinater til baser
  const float BS[4][2] = {
    {0.0, 0.0}, //0
    {0.0, 2.208}, //1
    {2.404, 0}, //2
    {2.404, 2.208}, //3
  };

  const float CALIB_SLOPE[4]     = {1.0499, 1.0499, 1.0416, 1.0478};
  const float CALIB_INTERCEPT[4] = {452.6,  452.6,  381.7,  262.0 };


  unsigned long T; //nåværende tid
  float DT; //tid mellom oppdateringer i filteret.




  void setup() {


  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(); 
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(38400); 
  Serial2.begin(115200);
  Serial3.begin(38400);

  Serial.setTimeout(10);
  Serial2.setTimeout(10);

  mpu.initialize();

  // IMU offset: hent fra kalibreringskode
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


    K.R = {
      n_x*n_x, 0.0, 0.0,
      0.0, n_y*n_y, 0.0,
      0.0, 0.0, n_teta*n_teta,
    };


    K.Q = {
      m_x*m_x, 0.0, 0.0,
      0.0, m_y*m_y, 0.0,
      0.0, 0.0, m_teta*m_teta,
    };

    // venter til bu04 gir data
    while (!new_position) {
        read_uwb();
        delay(10);
    }

    K.x(0) = x_uwb;
    K.x(1) = y_uwb;
    K.x(2) = 0.0;
    new_position = false;

    T = micros();


  }

  void loop() {
    unsigned long start = millis();

    read_serial_data();
    read_uwb();

    //tid
    DT = (micros()-T)/1000000.0; 
    T = micros();

    float teta = K.x(2);

   if (new_position) {
    obs(0) = x_uwb;
    obs(1) = y_uwb;
    obs(2) = read_teta(DT);
    
    new_position = false;
   }
  else{
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
    Serial.print(K.x(0), 4);
    Serial.print(",");
    Serial.print(K.x(1), 4);
    Serial.print(",");
    Serial.println(K.x(2), 4);

/*
    //sender data over zulu til en ekstern PC
    if (new_metal = true){
      Serial3.print(K.x(0));
      Serial3.print(" ");
      Serial3.print(K.x(1));
      Serial3.print(" ");
      Serial3.print(M);
      Serial3.print(" ");
      Serial3.print(Til);
      Serial3.print(" ");
      Serial3.print(wl_o);
      Serial3.print(" ");
      Serial3.print(wr_o);
      Serial3.print(" ");
      Serial3.print(Vb);
      Serial3.print(" ");
      Serial3.println(Ib);
      new_metal = false;    // HUSK Å SETT TIL TRUE 
    }
    else{
      Serial3.print(K.x(0));
      Serial3.print(" ");
      Serial3.print(K.x(1));
      Serial3.print(" ");
      Serial3.print(0);
      Serial3.print(" ");
      Serial3.print(Til);
      Serial3.print(" ");
      Serial3.print(wl_o);
      Serial3.print(" ");
      Serial3.print(wr_o);
      Serial3.print(" ");
      Serial3.print(Vb);
      Serial3.print(" ");
      Serial3.println(Ib);
      new_metal = false;    // HUSK Å SETT TIL TRUE 
    }
    */

    //Serial.print(",");
    //Serial.println(obs(2));


    //setter en delay her slik at loopen kjører på en bestemt frekvens
    long finished = millis() - start;
    long lasting = sample_time - finished;
    if (lasting > 0) {
      delay(lasting);
    }
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
      
      int comma = line.indexOf(",");
      
      if (comma != -1) {
          wl_received = line.substring(0, comma).toFloat();
          wr_received = line.substring(comma + 1).toFloat();

      }
  }

}


void read_uwb(){
  static uint8_t buffer[256];
  static int bufferIndex = 0;
  static bool messageStarted = false;

  while (Serial2.available()) {
    uint8_t byte = Serial2.read();
    if (!messageStarted && byte == 0xAA) {
      messageStarted = true;
      bufferIndex = 0;
      buffer[bufferIndex++] = byte;
    } else if (messageStarted) {
      buffer[bufferIndex++] = byte;
      if (bufferIndex >= 35) {
        float distances[4];
        if (parseUwbData(buffer, bufferIndex, distances)) {
          float x, y;
          if (trilaterate(distances, &x, &y)) {
            x_uwb = x;
            y_uwb = y;
            new_position = true;   // flag for EKF update
          }
        }
        messageStarted = false;
        bufferIndex = 0;
      }
      if (bufferIndex >= 256) { messageStarted = false; bufferIndex = 0; }
    }
  }
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

float get_wl_o() { return wl_received; }
float get_wr_o() { return wr_received; }




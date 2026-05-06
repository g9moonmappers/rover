# Robot EKF Localization with bu04 trilateration

Extended Kalman Filter (EKF) based localization for a 6-wheel differential drive robot.
Sensor fusion of wheel odometry, IMU gyroscope, and UWB position (BU04) running across an Arduino Mega and a Jetson Nano (ROS2).

---



## Arduino — `ekf.ino`

Runs on the Arduino Mega. Reads UWB distances from the BU04, computes position via trilateration, reads gyro from the MPU6050, runs the EKF, and sends the estimated state back to the Jetson over USB Serial.

```cpp
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
float get_wr_o() { return wr_received; }```
```
---
```

## Python — `robot.py`

Runs on the Jetson Nano as a ROS2 node. Reads wheel encoder positions from Dynamixel motors, computes wheel velocities, sends them to the Arduino, reads the EKF state back, and publishes odometry and TF for Nav2.

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from dynamixel_sdk import *
import math
from tf2_ros import TransformBroadcaster
import serial

portHandler = PortHandler("/dev/ttyUSB0")
packetHandler = PacketHandler(2.0)

mode_adresse = 11
torque_on_address = 64
goal_velocity_adresse = 104
present_velocity_adresse = 128
present_position_adresse = 132

dxl_id1 = 1
dxl_id2 = 2
dxl_id3 = 3
dxl_id4 = 4
dxl_id5 = 5
dxl_id6 = 6

T_ON = 1
T_OFF = 0
velocity_mode = 1
MAX_VEL = 460

WHEEL_RADIUS_METERS = 0.05
WHEEL_SEPARATION_METERS = 0.29

groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)


class Robot(Node):
    def __init__(self):
        super().__init__('robot')

        if not portHandler.openPort():
            self.get_logger().error('Failed to open port!')
            return
        portHandler.setBaudRate(57600)
        self.ser = serial.Serial('/dev/ttyACM0', 38400, timeout=0.01)  # 10ms max, non-blocking

        for id in [dxl_id1, dxl_id2, dxl_id3, dxl_id4, dxl_id5, dxl_id6]:
            packetHandler.write1ByteTxRx(portHandler, id, mode_adresse, velocity_mode)
            packetHandler.write1ByteTxRx(portHandler, id, torque_on_address, T_ON)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left  = (self.read_pos(dxl_id1) + self.read_pos(dxl_id3) + self.read_pos(dxl_id5)) / 3
        self.last_right = (self.read_pos(dxl_id2) + self.read_pos(dxl_id4) + self.read_pos(dxl_id6)) / 3
        self.last_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel, 10)
        self.create_timer(0.02, self.update_odom)

        self.get_logger().info('Robot ready!')

    def read_ekf(self):
        try:
            line = self.ser.readline().decode().strip()
            if not line.startswith("S:"):
                return None
            parts = line[2:].split(",")
            return float(parts[0]), float(parts[1]), float(parts[2])
        except:
            return None

    def read_motor_velocities(self, dt):
        lp1 = self.read_pos(dxl_id1)
        rp2 = self.read_pos(dxl_id2)
        lp3 = self.read_pos(dxl_id3)
        rp4 = self.read_pos(dxl_id4)
        lp5 = self.read_pos(dxl_id5)
        rp6 = self.read_pos(dxl_id6)

        avg_left  = (lp1 + lp3 + lp5) / 3
        avg_right = (rp2 + rp4 + rp6) / 3

        ld = (avg_left  - self.last_left)  / 4096 * 2 * math.pi * WHEEL_RADIUS_METERS
        rd = -(avg_right - self.last_right) / 4096 * 2 * math.pi * WHEEL_RADIUS_METERS

        self.last_left  = avg_left
        self.last_right = avg_right

        if dt <= 0:
            return 0.0, 0.0

        wl = (ld / dt) / WHEEL_RADIUS_METERS
        wr = (rd / dt) / WHEEL_RADIUS_METERS

        return wl, wr

    def read_pos(self, id):
        pos, res, err = packetHandler.read4ByteTxRx(portHandler, id, present_position_adresse)
        if pos > 2147483648:
            pos -= 4294967296
        return pos

    def send_velocity(self, target_velocityV, target_velocityH):
        param_goal_velocity_H = [
            DXL_LOBYTE(DXL_LOWORD(target_velocityH)),
            DXL_HIBYTE(DXL_LOWORD(target_velocityH)),
            DXL_LOBYTE(DXL_HIWORD(target_velocityH)),
            DXL_HIBYTE(DXL_HIWORD(target_velocityH))
        ]
        param_goal_velocity_V = [
            DXL_LOBYTE(DXL_LOWORD(target_velocityV)),
            DXL_HIBYTE(DXL_LOWORD(target_velocityV)),
            DXL_LOBYTE(DXL_HIWORD(target_velocityV)),
            DXL_HIBYTE(DXL_HIWORD(target_velocityV))
        ]

        groupBulkWrite.addParam(dxl_id1, goal_velocity_adresse, 4, param_goal_velocity_V)
        groupBulkWrite.addParam(dxl_id3, goal_velocity_adresse, 4, param_goal_velocity_V)
        groupBulkWrite.addParam(dxl_id5, goal_velocity_adresse, 4, param_goal_velocity_V)
        groupBulkWrite.addParam(dxl_id2, goal_velocity_adresse, 4, param_goal_velocity_H)
        groupBulkWrite.addParam(dxl_id4, goal_velocity_adresse, 4, param_goal_velocity_H)
        groupBulkWrite.addParam(dxl_id6, goal_velocity_adresse, 4, param_goal_velocity_H)
        groupBulkWrite.txPacket()
        groupBulkWrite.clearParam()

    def cmd_vel(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        left_vel = (linear - angular * WHEEL_SEPARATION_METERS / 2) * MAX_VEL
        right_vel = (linear + angular * WHEEL_SEPARATION_METERS / 2) * MAX_VEL

        left_vel = max(-MAX_VEL, min(MAX_VEL, int(left_vel)))
        right_vel = max(-MAX_VEL, min(MAX_VEL, int(right_vel)))

        self.send_velocity(left_vel, -right_vel)
        self.get_logger().info(f'Left: {left_vel} Right: {right_vel}')

    def update_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9

        wl, wr = self.read_motor_velocities(dt)
        self.last_time = now

        # Send wheel velocities to Arduino EKF
        self.ser.write(f"{wl:.4f},{wr:.4f}\n".encode())

        # Read EKF state back (no reset_input_buffer — avoids stalling TF)
        ekf = self.read_ekf()
        if ekf is not None:
            self.x, self.y, self.theta = ekf

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2)
        t.transform.rotation.w = math.cos(self.theta / 2)
        self.tf.sendTransform(t)

        o = Odometry()
        o.header.stamp = now.to_msg()
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = math.sin(self.theta / 2)
        o.pose.pose.orientation.w = math.cos(self.theta / 2)
        o.twist.twist.linear.x = (wl + wr) / 2 * WHEEL_RADIUS_METERS
        o.twist.twist.angular.z = (wr - wl) * WHEEL_RADIUS_METERS / WHEEL_SEPARATION_METERS
        self.odom_pub.publish(o)

    def destroy_node(self):
        self.send_velocity(0, 0)
        self.ser.close()
        for id in [dxl_id1, dxl_id2, dxl_id3, dxl_id4, dxl_id5, dxl_id6]:
            packetHandler.write1ByteTxRx(portHandler, id, torque_on_address, T_OFF)
        portHandler.closePort()
        super().destroy_node()


def main():
    rclpy.init()
    node = Robot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Running the System

**Terminal 1 — Robot node:**
```bash
python3 robot.py
```

**Terminal 2 — Nav2 (when ready):**
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false
```

**Terminal 3 — RViz:**
```bash
ros2 run rviz2 rviz2
```

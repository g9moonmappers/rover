# EKF and Stereo Camera Fusion

This page documents the current implementation of the Extended Kalman Filter (EKF) fused with stereo camera odometry for robot localization.

---

## Python — ROS2 Node (`robot.py`)

Runs on the Jetson Nano. Reads wheel encoder positions, subscribes to camera odometry from RTAB-Map, sends wheel speeds and camera position to the Arduino, and publishes the EKF output as ROS2 odometry.

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
        self.ser = serial.Serial('/dev/ttyACM0', 38400, timeout=0.05)

        for id in [dxl_id1, dxl_id2, dxl_id3, dxl_id4, dxl_id5, dxl_id6]:
            packetHandler.write1ByteTxRx(portHandler, id, mode_adresse, velocity_mode)
            packetHandler.write1ByteTxRx(portHandler, id, torque_on_address, T_ON)

        self.create_subscription(Odometry, '/rtabmap/odom', self.odom_callback, 10)

        self.cam_x = 0
        self.cam_y = 0
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

    def odom_callback(self, msg):
        self.cam_x = msg.pose.pose.position.x
        self.cam_y = msg.pose.pose.position.y

        covariance = msg.pose.covariance[0]
        if covariance > 9000:
            print("WARNING: Tracking lost - skipping this measurement")
            return

    def read_ekf(self):
        line = ""
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

        ld = -(avg_left  - self.last_left)  / 4096 * 2 * math.pi * WHEEL_RADIUS_METERS
        rd =  (avg_right - self.last_right) / 4096 * 2 * math.pi * WHEEL_RADIUS_METERS

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

        left_vel  = (linear - angular * WHEEL_SEPARATION_METERS / 2) * MAX_VEL
        right_vel = (linear + angular * WHEEL_SEPARATION_METERS / 2) * MAX_VEL

        left_vel  = max(-MAX_VEL, min(MAX_VEL, int(left_vel)))
        right_vel = max(-MAX_VEL, min(MAX_VEL, int(right_vel)))

        self.send_velocity(-left_vel, right_vel)
        self.get_logger().info(f'Left: {left_vel} Right: {right_vel}')

    def update_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9

        wl, wr = self.read_motor_velocities(dt)
        self.last_time = now

        self.ser.write(f"{wl:.4f},{wr:.4f},{self.cam_x:.3f},{self.cam_y:.3f}\n".encode())
        self.ser.reset_input_buffer()
        ekf = self.read_ekf()

        if ekf is not None:
            self.x, self.y, self.theta = ekf
            print(f"Position -> x: {self.x:.3f}  y: {self.y:.3f}")

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
        o.twist.twist.linear.x  = (wl + wr) / 2 * WHEEL_RADIUS_METERS
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

## Arduino — EKF (`ekf.ino`)

Runs on the Arduino Mega. Reads wheel speeds and camera position from serial, reads gyro from MPU6050, runs the EKF, and sends the estimated state back to the Jetson.

```cpp
#include <Kalman.h>
using namespace BLA;
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

int16_t gx_raw, gy_raw, gz_raw;
int16_t ax_raw, ay_raw, az_raw;

float wl_received = 0.0;
float wr_received = 0.0;
float x_received  = 0.0;
float y_received  = 0.0;

bool new_position = false;

#define Nstate 3
#define Nobs   3

#define n_x    0.01   // measurement noise — x position
#define n_y    0.01   // measurement noise — y position
#define n_teta 0.01   // measurement noise — heading

#define m_x    0.1    // process noise — x
#define m_y    0.1    // process noise — y
#define m_teta 0.8    // process noise — heading

BLA::Matrix<Nobs> obs;
KALMAN<Nstate, Nobs> K;
unsigned long T;
float DT;


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

  K.H = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
  };

  K.R = {
    n_x*n_x, 0.0,     0.0,
    0.0,     n_y*n_y, 0.0,
    0.0,     0.0,     n_teta*n_teta,
  };

  K.Q = {
    m_x*m_x, 0.0,     0.0,
    0.0,     m_y*m_y, 0.0,
    0.0,     0.0,     m_teta*m_teta,
  };

  T = micros();
}


void loop() {
  read_serial_data();

  DT = (micros() - T) / 1000000.0;
  T = micros();

  float teta = K.x(2);

  // Update observations — use camera position if fresh, else feed back prediction
  if (new_position) {
    obs(0) = read_x();
    obs(1) = read_y();
    obs(2) = read_teta(DT);
    new_position = false;
  } else {
    obs(0) = K.x(0);     // zero innovation on x/y
    obs(1) = K.x(1);
    obs(2) = read_teta(DT);
  }

  float wl_o = get_wl_o();
  float wr_o = get_wr_o();

  float V = (0.05 / 2) * (wr_o + wl_o);
  float w  = (0.05 / 0.29) * (wr_o - wl_o);

  // Jacobian of motion model (linearized around current teta)
  K.F = {
    1.0, 0.0, -V*sin(teta)*DT,
    0.0, 1.0,  V*cos(teta)*DT,
    0.0, 0.0,  1.0,
  };

  // Nonlinear prediction
  BLA::Matrix<Nstate> x_pred;
  x_pred(0) = K.x(0) + V * cos(teta) * DT;
  x_pred(1) = K.x(1) + V * sin(teta) * DT;
  x_pred(2) = K.x(2) + w * DT;
  K.x = x_pred;

  K.update(obs);

  // Send EKF state to Jetson
  Serial.print("S:");
  Serial.print(K.x(0));
  Serial.print(",");
  Serial.print(K.x(1));
  Serial.print(",");
  Serial.println(K.x(2));
}


float read_teta(float DT) {
  mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);
  float gz_rad = gz_raw * (1.0 / 131.0) * (M_PI / 180.0);
  static float teta = 0.0;
  teta += gz_rad * DT;
  return teta;
}


void read_serial_data() {
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


void SKIFTE_TIL_VINKELHASITGHET(float V, float w, float &wr, float &wl) {
  float r = 0.05;
  float B = 0.29;
  wr = (V / r) - (w * B / (2.0 * r));
  wl = (V / r) + (w * B / (2.0 * r));
}

float get_wl_o() { return wl_received; }
float get_wr_o() { return wr_received; }
float read_x()   { return x_received; }
float read_y()   { return y_received; }
```

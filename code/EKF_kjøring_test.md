'''
  import rclpy
  from rclpy.node import Node
  from geometry_msgs.msg import Twist, TransformStamped
  from nav_msgs.msg import Odometry
  from dynamixel_sdk import *
  import math
  from tf2_ros import TransformBroadcaster
  import serial
  import time
  import csv
  
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
          self.ser = serial.Serial('/dev/ttyACM0', 38400, timeout=0.001)
  
          for id in [dxl_id1, dxl_id2, dxl_id3, dxl_id4, dxl_id5, dxl_id6]:
              packetHandler.write1ByteTxRx(
                  portHandler, id, mode_adresse, velocity_mode)
              packetHandler.write1ByteTxRx(
                  portHandler, id, torque_on_address, T_ON)
  
          self.x = 0.0
          self.y = 0.0
          self.theta = 0.0
          self.last_left  = (self.read_pos(dxl_id1) + self.read_pos(dxl_id3) + self.read_pos(dxl_id5)) / 3
          self.last_right = (self.read_pos(dxl_id2) + self.read_pos(dxl_id4) + self.read_pos(dxl_id6)) / 3
          self.last_time = self.get_clock().now()
  
          # logging
          self.log_ekf_x = []
          self.log_ekf_y = []
          self.log_theta = []
          self.log_time = []
          self.log_uwb_x = []
          self.log_uwb_y = []
          self.last_uwb = (None, None)
          self.log_start = time.time()
  
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
              if len(parts) != 6:
                  return None
              return float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])
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
          pos, res, err = packetHandler.read4ByteTxRx(
              portHandler, id, present_position_adresse)
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
  
          self.send_velocity(left_vel, -right_vel)
          self.get_logger().info(f'Left: {left_vel} Right: {right_vel}')
  
      def update_odom(self):
          now = self.get_clock().now()
          dt = (now - self.last_time).nanoseconds / 1e9
  
          wl, wr = self.read_motor_velocities(dt)
          self.last_time = now
  
          self.ser.write(f"{wl:.4f},{wr:.4f}\n".encode())
  
          self.ser.reset_input_buffer()
          ekf = self.read_ekf()
          if ekf is not None:
              self.x, self.y, self.theta, ux, uy = ekf
  
              # log EKF
              self.log_ekf_x.append(self.x)
              self.log_ekf_y.append(self.y)
              self.log_theta.append(self.theta)
              self.log_time.append(time.time() - self.log_start)
  
              # log UWB only when it changes
              if (ux, uy) != self.last_uwb:
                  self.log_uwb_x.append(ux)
                  self.log_uwb_y.append(uy)
                  self.last_uwb = (ux, uy)
  
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
  
      def save_log(self):
          with open('/home/jetson/ekf_log.csv', 'w', newline='') as f:
              writer = csv.writer(f)
              writer.writerow(['time', 'ekf_x', 'ekf_y', 'theta'])
              for i in range(len(self.log_ekf_x)):
                  writer.writerow([
                      self.log_time[i],
                      self.log_ekf_x[i],
                      self.log_ekf_y[i],
                      self.log_theta[i]
                  ])
          with open('/home/jetson/uwb_log.csv', 'w', newline='') as f:
              writer = csv.writer(f)
              writer.writerow(['uwb_x', 'uwb_y'])
              for i in range(len(self.log_uwb_x)):
                  writer.writerow([self.log_uwb_x[i], self.log_uwb_y[i]])
          self.get_logger().info(f'Log saved! EKF samples: {len(self.log_ekf_x)}, UWB samples: {len(self.log_uwb_x)}')
  
      def destroy_node(self):
          self.save_log()
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
'''
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

ekf = pd.read_csv('/home/jetson/ekf_log.csv')
uwb = pd.read_csv('/home/jetson/uwb_log.csv')

fig, axes = plt.subplots(1, 2, figsize=(14, 6))

axes[0].plot(uwb['uwb_x'], uwb['uwb_y'], 'r.', markersize=4, 
             label=f'UWB raw ({len(uwb)} pts)')
axes[0].plot(ekf['ekf_x'], ekf['ekf_y'], 'b-', linewidth=2,
             label=f'EKF ({len(ekf)} pts)')
axes[0].set_xlabel('x [m]')
axes[0].set_ylabel('y [m]')
axes[0].legend()
axes[0].set_aspect('equal')
axes[0].grid(True)
axes[0].set_title('XY Path')

axes[1].plot(ekf['time'], ekf['theta'], 'b-', linewidth=2)
axes[1].set_xlabel('time [s]')
axes[1].set_ylabel('theta [rad]')
axes[1].grid(True)
axes[1].set_title('Heading over time')

plt.tight_layout()

print(f"UWB std:  x={np.std(uwb['uwb_x']):.4f}  y={np.std(uwb['uwb_y']):.4f}")
print(f"EKF std:  x={np.std(ekf['ekf_x']):.4f}  y={np.std(ekf['ekf_y']):.4f}")

plt.show()
'''

'''

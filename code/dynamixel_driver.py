import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from dynamixel_sdk import *
import math
from tf2_ros import TransformBroadcaster

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

        for id in [dxl_id1, dxl_id2, dxl_id3, dxl_id4, dxl_id5, dxl_id6]:
            packetHandler.write1ByteTxRx(portHandler, id, mode_adresse, velocity_mode)
            packetHandler.write1ByteTxRx(portHandler, id, torque_on_address, T_ON)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left  = self.read_pos(dxl_id1)
        self.last_right = self.read_pos(dxl_id2)
        self.last_time  = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel, 10)
        self.create_timer(0.05, self.update_odom)

        self.get_logger().info('Robot ready!')

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
        linear  = msg.linear.x
        angular = msg.angular.z

        left_vel  = (linear - angular * WHEEL_SEPARATION_METERS / 2) * MAX_VEL
        right_vel = (linear + angular * WHEEL_SEPARATION_METERS / 2) * MAX_VEL

        left_vel  = max(-MAX_VEL, min(MAX_VEL, int(left_vel)))
        right_vel = max(-MAX_VEL, min(MAX_VEL, int(right_vel)))

        self.send_velocity(-left_vel, -right_vel)
        self.get_logger().info(f'Left: {left_vel} Right: {right_vel}')

    def update_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9

        lp = self.read_pos(dxl_id1)
        rp = self.read_pos(dxl_id2)

        ld = -(lp - self.last_left)  / 4096 * 2 * math.pi * WHEEL_RADIUS_METERS
        rd =  (rp - self.last_right) / 4096 * 2 * math.pi * WHEEL_RADIUS_METERS

        self.last_left  = lp
        self.last_right = rp
        self.last_time  = now

        dist = (ld + rd) / 2
        dth  = (rd - ld) / WHEEL_SEPARATION_METERS

        self.x     += dist * math.cos(self.theta)
        self.y     += dist * math.sin(self.theta)
        self.theta += dth

        t = TransformStamped()
        t.header.stamp    = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2)
        t.transform.rotation.w = math.cos(self.theta / 2)
        self.tf.sendTransform(t)

        o = Odometry()
        o.header.stamp    = now.to_msg()
        o.header.frame_id = 'odom'
        o.child_frame_id  = 'base_link'
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = math.sin(self.theta / 2)
        o.pose.pose.orientation.w = math.cos(self.theta / 2)
        if dt > 0:
            o.twist.twist.linear.x  = dist / dt
            o.twist.twist.angular.z = dth  / dt
        self.odom_pub.publish(o)

    def destroy_node(self):
        self.send_velocity(0, 0)
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

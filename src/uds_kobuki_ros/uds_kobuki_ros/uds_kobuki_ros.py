# Copyright 2024 Filip Stec
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import rclpy
from rclpy.time import Time
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

import math
import socket
import threading
from typing import List

from .kobuki import *
from .lidar import *
from .defines import *
from .mobile_robot import MobileRobot
from .utils import *
import numpy as np

class Kobuki(Node):
    stop_flag = False
    robot_data: TKobukiData = None
    lidar_data: TLaserMeasurement = None
    mobile_robot = MobileRobot()

    odom = [0.0, 0.0, 0.0]
    lock = threading.Lock()
    accumulated_yaw = 0.0
    last_pose_update_rates = [0.0, 0.0, 0.0]

    def __init__(self):
        super().__init__('kobuki')

        self.setup_parameters()
        self.setup_publishers()
        self.setup_subscribers()
        self.setup_timers()
        self.publish_static_tf()
        self.robot_setup_udp()
        self.lidar_setup_udp()
        self.get_logger().info(self.get_name() + " initialized")

    def __del__(self):
        self.robot_sock.close()
        self.lidar_sock.close()

    def setup_parameters(self):
        # self.declare_parameter('ip_address', '127.0.0.1')
        self.declare_parameter('ip_address', '192.168.1.13')
        self.declare_parameter('robot_upd_port_up', 5300)
        self.declare_parameter('robot_upd_port_down', 53000)
        self.declare_parameter('lidar_upd_port_up', 5299)
        self.declare_parameter('lidar_upd_port_down', 52999)

        self.ip_address = self.get_parameter('ip_address').get_parameter_value().string_value
        self.robot_upd_port_up = self.get_parameter('robot_upd_port_up').get_parameter_value().integer_value
        self.robot_upd_port_down = self.get_parameter('robot_upd_port_down').get_parameter_value().integer_value
        self.lidar_upd_port_up = self.get_parameter('lidar_upd_port_up').get_parameter_value().integer_value
        self.lidar_upd_port_down = self.get_parameter('lidar_upd_port_down').get_parameter_value().integer_value

    def setup_publishers(self):
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.laser_scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.wheel_speed_pub = self.create_publisher(JointState, '/wheel_speed', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

    def setup_subscribers(self):
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

    def setup_timers(self):
        self.main_timer = self.create_timer(0.05, self.main_timer_callback)
        self.cmd_vel_timer = self.create_timer(0.2, self.cmd_vel_timeout_callback)

    def publish_static_tf(self):
        self.map_to_odom_tf()
        self.base_link_to_base_laser_tf()

    def robot_setup_udp(self):
        self.robot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.robot_sock.bind(("0.0.0.0", self.robot_upd_port_down))

        # Message to be sent
        message = set_sound(440, 1000)
        self.send_robot_udp_message(message)

        self.robot_receiver_thread = threading.Thread(target=self.robot_udp_receiver_callback)
        self.robot_receiver_thread.start()

    def lidar_setup_udp(self):
        self.lidar_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.lidar_sock.bind(("0.0.0.0", self.lidar_upd_port_down))

        # Message to be sent
        message = 0x00
        self.send_lidar_udp_message(message)

        self.lidar_receiver_thread = threading.Thread(target=self.lidar_udp_receiver_callback)
        self.lidar_receiver_thread.start()

    def cmd_vel_callback(self, msg: Twist):
        # Reset timeout of cmd_vel command
        self.cmd_vel_timer.reset()
        
        if math.isclose(msg.angular.z, 0.0):
            message = set_translation_speed(msg.linear.x * 1000)
        elif math.isclose(msg.linear.x, 0.0):
            message = set_rotation_speed(msg.angular.z)
        else:
            radius = (msg.linear.x * 1000) / msg.angular.z
            message = set_arc_speed(msg.linear.x * 1000, radius)
        self.send_robot_udp_message(message)

    def main_timer_callback(self):
        with self.lock:
            now = self.get_clock().now()

            if self.robot_data:
                self.publish_odometry(now)
                self.publish_wheel_speed(now)
            
            if self.lidar_data:
                self.publish_laser_scan(now)
    
    def cmd_vel_timeout_callback(self):
        '''Stop the robot when cmd_vel command does not arrive for 0.2s.'''
        message = set_translation_speed(0)
        self.send_robot_udp_message(message)

    def robot_udp_receiver_callback(self):
        while not self.stop_flag:
            response, _ = self.robot_sock.recvfrom(1024)

            with self.lock:
                parsed_data = parse_kobuki_message(response)
                if isinstance(parsed_data, TKobukiData):
                    self.robot_data = parsed_data
        print("Robot UDP receiver stopped")
    
    def lidar_udp_receiver_callback(self):
        while not self.stop_flag:
            response, _ = self.lidar_sock.recvfrom(1000 * LaserData.MSG_SIZE)

            with self.lock:
                self.lidar_data = parse_lidar_message(response)
        print("Lidar UDP receiver stopped")

    
    def send_robot_udp_message(self, message: List[int]):
        message_bytes = bytes(message)
        self.robot_sock.sendto(message_bytes, (self.ip_address, self.robot_upd_port_up))
    
    def send_lidar_udp_message(self, message: List[int]):
        message_bytes = bytes(message)
        self.lidar_sock.sendto(message_bytes, (self.ip_address, self.lidar_upd_port_up))

    '''
    Based on https://github.com/kobuki-base/kobuki_core/blob/e2f0feac0f7a9964d021ac3241b7663f7728d5b9/src/driver/diff_drive.cpp#L51
    '''
    def publish_odometry(self, stamp: Time):
        # Calculate new robot state 
        pose_update, pose_update_rates = self.mobile_robot.update_diff_drive(
            self.robot_data.timestamp,
            self.robot_data.EncoderLeft,
            self.robot_data.EncoderRight)
    
        # Update odometry
        if not math.isclose(pose_update[0], 0.0):
            self.odom[0] += math.cos(self.odom[2]) * pose_update[0]
            self.odom[1] += math.sin(self.odom[2]) * pose_update[0]
        # self.odom[2] = wrap_angle(self.odom[2] + pose_update[2])
        self.odom[2] = self.odom[2] + pose_update[2]

        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        # Position
        msg.pose.pose.position.x = self.odom[0]
        msg.pose.pose.position.y = self.odom[1]
        msg.pose.pose.position.z = 0.0
        # Orientation
        quat = yaw_to_quaternion(self.odom[2])
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        # Linear velocity
        msg.twist.twist.linear.x = pose_update_rates[0]
        msg.twist.twist.linear.y = pose_update_rates[1]
        msg.twist.twist.linear.z = 0.0
        # Angular velocity
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = pose_update_rates[2]

        # Covariance
        self.accumulated_yaw = self.accumulated_yaw + pose_update_rates[2]
        yaw_acc = pose_update_rates[2] - self.last_pose_update_rates[2]
        # msg.twist.covariance[0] = 0.001*abs(self.accumulated_yaw)
        # msg.twist.covariance[7] = 0.001*abs(self.accumulated_yaw)
        # msg.twist.covariance[35] = 0.001*abs(self.accumulated_yaw)
        alpha = 0.0001
        beta = 0.00001
        # msg.pose.covariance[0] = alpha*abs(self.accumulated_yaw) + beta*yaw_acc
        # msg.pose.covariance[7] = alpha*abs(self.accumulated_yaw) + beta*yaw_acc
        # msg.pose.covariance[35] = alpha*abs(self.accumulated_yaw) + beta*yaw_acc
        msg.twist.covariance[0] = 1e-9    #co mas screennute
        msg.twist.covariance[7] = 1e-9
        msg.twist.covariance[35] = 1e-9
        # msg.twist.covariance[0] = 0.000001
        # msg.twist.covariance[7] = 0.000001
        # msg.twist.covariance[35] = 0.000001
        # msg.pose.covariance[0] += msg.twist.covariance[0]
        # msg.pose.covariance[7] += msg.twist.covariance[7]
        # msg.pose.covariance[35] += msg.twist.covariance[35]
        self.last_pose_update_rates = pose_update_rates

        self.odom_pub.publish(msg)
        # self.odom_to_base_link_tf(msg)

    def publish_laser_scan(self, stamp: Time):
        msg = LaserScan()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = 'base_laser'

        # Sort to angles ascending
        angles, msg.ranges = zip(*sorted(zip([x.scanAngle for x in self.lidar_data.Data], [x.scanDistance * 1e-3 for x in self.lidar_data.Data]), reverse=True))

        msg.angle_min = math.radians(360.0 - angles[0])
        msg.angle_max = math.radians(360.0 - angles[-1])
        msg.angle_increment = (msg.angle_max - msg.angle_min) / (self.lidar_data.numberOfScans - 1)

        msg.time_increment = 0.0
        msg.scan_time = 0.17

        msg.range_min = 0.15
        msg.range_max = 5.0
        
        self.laser_scan_pub.publish(msg)

    def publish_wheel_speed(self, stamp: Time):
        left_velocity = self.mobile_robot.last_velocity_left
        right_velocity = self.mobile_robot.last_velocity_right
        print(self.robot_data.GyroAngle)

        wheel_speed_msg = JointState()
        wheel_speed_msg.header.stamp = stamp.to_msg()
        wheel_speed_msg.name = ['left_angular', 'right_angular']
        wheel_speed_msg.velocity = [left_velocity, right_velocity]
        self.wheel_speed_pub.publish(wheel_speed_msg)
    
    def base_link_to_base_laser_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "base_laser"

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.15
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_static_broadcaster.sendTransform(t)
    
    def map_to_odom_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_static_broadcaster.sendTransform(t)
    
    def odom_to_base_link_tf(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.1
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)

    try:
        kobuki = Kobuki()
        rclpy.spin(kobuki)
    except KeyboardInterrupt:
        pass
    finally:
        kobuki.stop_flag = True

        # Destroy the node explicitly
        kobuki.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
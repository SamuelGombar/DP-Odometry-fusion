#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
from collections import deque

class MultiOdomSubscriber(Node):

    def __init__(self):
        super().__init__('multi_odom_subscriber')

        # Create Subscription for /odom
        # We use 10 as the queue size (default reliable reliability)
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # Create Subscription for /odom_icp
        self.sub_icp = self.create_subscription(
            Odometry,
            '/odom_icp',
            self.icp_callback,
            10)

        # Create Subscription for /odom_fused
        self.sub_fused = self.create_subscription(
            Odometry,
            '/odom_fused',
            self.fused_callback,
            10)

        buffer_len = 3000

        # Prevent unused variable warnings
        self.sub_odom
        self.sub_icp
        self.sub_fused

        self.odom_x = deque(maxlen=buffer_len)
        self.odom_y = deque(maxlen=buffer_len)
        
        self.icp_x = deque(maxlen=buffer_len)
        self.icp_y = deque(maxlen=buffer_len)
        
        self.fused_x = deque(maxlen=buffer_len)
        self.fused_y = deque(maxlen=buffer_len)

        self.get_logger().info("Multi-Odom Node has been started.")

    def odom_callback(self, msg):
        self.odom_x.append(msg.pose.pose.position.x)
        self.odom_y.append(msg.pose.pose.position.y)

    def icp_callback(self, msg):
        self.icp_x.append(msg.pose.pose.position.x)
        self.icp_y.append(msg.pose.pose.position.y)

    def fused_callback(self, msg):
        self.fused_x.append(msg.pose.pose.position.x)
        self.fused_y.append(msg.pose.pose.position.y)


def main(args=None):
    rclpy.init(args=args)

    node = MultiOdomSubscriber()
    # rclpy.spin(node)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    fig, ax = plt.subplots()
    ax.set_title("Real-time Odometry Comparison")
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.grid(True)

    ax.set_xlim(-2.5, 2.5)
    ax.set_ylim(-2.5, 2.5)

    ln_odom, = ax.plot([], [], 'b-', label='/odom', alpha=0.5)
    ln_icp, = ax.plot([], [], 'r--', label='/odom_icp', alpha=0.7)
    ln_fused, = ax.plot([], [], 'g-', label='/odom_fused', linewidth=2)
    
    ax.legend()

    def update_plot(frame):
        # Update data for each line
        ln_odom.set_data(node.odom_x, node.odom_y)
        ln_icp.set_data(node.icp_x, node.icp_y)
        ln_fused.set_data(node.fused_x, node.fused_y)

        # Rescale axes dynamically to fit the data
        ax.relim()
        ax.autoscale_view()
        
        return ln_odom, ln_icp, ln_fused

        # Create animation (updates every 100ms)
    ani = FuncAnimation(fig, update_plot, interval=100)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
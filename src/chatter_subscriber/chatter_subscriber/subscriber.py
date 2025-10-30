import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import roslibpy


class WebSocketToRos2(Node):
    def __init__(self):
        super().__init__('websocket_to_ros2')

        # ROS 2 publishers
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Connect to ROS bridge in Docker
        self.client = roslibpy.Ros(host='localhost', port=9090)
        self.client.run()
        self.get_logger().info('Connected to ROS bridge')

        # Subscribe to WebSocket topics
        self.scan_topic = roslibpy.Topic(self.client, '/scan', 'sensor_msgs/LaserScan')
        self.scan_topic.subscribe(self.ws_scan_callback)

        self.cmd_vel_topic = roslibpy.Topic(self.client, '/cmd_vel', 'geometry_msgs/Twist')
        self.cmd_vel_topic.subscribe(self.ws_cmd_vel_callback)

    def ws_scan_callback(self, msg):
        try:
            ros_msg = LaserScan()

            # Handle header stamp
            stamp = msg['header'].get('stamp', 0.0)
            if isinstance(stamp, dict):
                ros_msg.header.stamp.sec = stamp.get('sec', 0)
                ros_msg.header.stamp.nanosec = stamp.get('nanosec', 0)
            else:
                ros_msg.header.stamp.sec = int(stamp)
                ros_msg.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
            ros_msg.header.frame_id = msg['header'].get('frame_id', '')

            # Copy other LaserScan fields
            ros_msg.angle_min = msg.get('angle_min', 0.0)
            ros_msg.angle_max = msg.get('angle_max', 0.0)
            ros_msg.angle_increment = msg.get('angle_increment', 0.0)
            ros_msg.time_increment = msg.get('time_increment', 0.0)
            ros_msg.scan_time = msg.get('scan_time', 0.0)
            ros_msg.range_min = msg.get('range_min', 0.0)
            ros_msg.range_max = msg.get('range_max', 0.0)

            # Replace None with NaN in ranges/intensities
            ros_msg.ranges = [float(r) if r is not None else float('nan')
                              for r in msg.get('ranges', [])]
            ros_msg.intensities = [float(i) if i is not None else float('nan')
                                   for i in msg.get('intensities', [])]

            self.scan_pub.publish(ros_msg)
            self.get_logger().info('Republished LaserScan message')
        except Exception as e:
            self.get_logger().error(f'Error processing LaserScan: {e}')

    def ws_cmd_vel_callback(self, msg):
        try:
            ros_msg = Twist()
            ros_msg.linear.x = msg['linear'].get('x', 0.0)
            ros_msg.linear.y = msg['linear'].get('y', 0.0)
            ros_msg.linear.z = msg['linear'].get('z', 0.0)
            ros_msg.angular.x = msg['angular'].get('x', 0.0)
            ros_msg.angular.y = msg['angular'].get('y', 0.0)
            ros_msg.angular.z = msg['angular'].get('z', 0.0)

            self.cmd_vel_pub.publish(ros_msg)
            self.get_logger().info(f"Republished Twist: linear=({ros_msg.linear.x}, {ros_msg.linear.y}, {ros_msg.linear.z}) angular=({ros_msg.angular.x}, {ros_msg.angular.y}, {ros_msg.angular.z})")
        except Exception as e:
            self.get_logger().error(f'Error processing Twist: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = WebSocketToRos2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.client.terminate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# import roslibpy

# class WebSocketToRos2(Node):
#     def __init__(self):
#         super().__init__('websocket_to_ros2')

#         # ROS 2 publisher
#         self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)

#         # Connect to ROS bridge in Docker
#         self.client = roslibpy.Ros(host='localhost', port=9090)
#         self.client.run()
#         self.get_logger().info('Connected to ROS bridge')

#         # Subscribe to WebSocket topic
#         self.topic = roslibpy.Topic(self.client, '/scan', 'sensor_msgs/LaserScan')
#         self.topic.subscribe(self.ws_callback)

#     def ws_callback(self, msg):
#         try:
#             ros_msg = LaserScan()

#             # Handle header stamp
#             stamp = msg['header'].get('stamp', 0.0)
#             if isinstance(stamp, dict):
#                 ros_msg.header.stamp.sec = stamp.get('sec', 0)
#                 ros_msg.header.stamp.nanosec = stamp.get('nanosec', 0)
#             else:  # stamp is a float
#                 ros_msg.header.stamp.sec = int(stamp)
#                 ros_msg.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
#             ros_msg.header.frame_id = msg['header'].get('frame_id', '')

#             # Copy other LaserScan fields
#             ros_msg.angle_min = msg.get('angle_min', 0.0)
#             ros_msg.angle_max = msg.get('angle_max', 0.0)
#             ros_msg.angle_increment = msg.get('angle_increment', 0.0)
#             ros_msg.time_increment = msg.get('time_increment', 0.0)
#             ros_msg.scan_time = msg.get('scan_time', 0.0)
#             ros_msg.range_min = msg.get('range_min', 0.0)
#             ros_msg.range_max = msg.get('range_max', 0.0)

#             # Replace None with NaN in ranges
#             ros_msg.ranges = [float(r) if r is not None else float('nan') for r in msg.get('ranges', [])]

#             # Replace None with NaN in intensities if present
#             ros_msg.intensities = [float(i) if i is not None else float('nan') for i in msg.get('intensities', [])]

#             self.publisher_.publish(ros_msg)
#             self.get_logger().info('Republished LaserScan message')
#         except Exception as e:
#             self.get_logger().error(f'Error processing message: {e}')


# def main(args=None):
#     rclpy.init(args=args)
#     node = WebSocketToRos2()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.client.terminate()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

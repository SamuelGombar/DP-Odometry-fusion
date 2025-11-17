import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')

        self.wheel_odom = None
        self.lidar_odom = None
        self.velocity = 0.0
        self.scan_distances = []
        self.scan_angle_increment = 0.0
        self.wheel_xy_pairs = []
        self.scan_xy_pairs = []
        self.scan_xy_vector = np.array([])
        self.x_hat = np.array([])

        self.P = np.eye(3)
        self.A = np.eye(3)

        self.Q = np.array([[1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1]])

        self.C = []
        self.C_list = []

        self.R = []
        
        self.K = []

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.icp_sub = self.create_subscription(
            Odometry,
            '/odom_icp',
            self.icp_callback,
            10
        )

        self.velocity_sub = self.create_subscription(
            Float64,
            '/velocity',
            self.velocity_callback,
            10
        )

        self.subscription_ = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.fused_pub = self.create_publisher(Odometry, '/odom_fused', 10)


    def publish_fused(self):
        if self.x_hat is None:
            return  # nothing to publish yet

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"  # or your global frame

        msg.pose.pose.position.x = self.x_hat[0]
        msg.pose.pose.position.y = self.x_hat[1]
        msg.pose.pose.position.z = 0.0  # assuming 2D

        # Convert yaw to quaternion
        theta = self.x_hat[2]
        qz = np.sin(theta / 2)
        qw = np.cos(theta / 2)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        self.fused_pub.publish(msg)

    def prediction(self):
        """
        speed: float
        wheel_odom: np.array of shape (3,) [x, y, theta]
        """

        # Predict state
        self.x_hat = self.wheel_odom.copy()

        theta = self.wheel_odom[2]

        # State transition matrix
        self.A = np.array([
            [1, 0, -self.velocity * (-np.sin(theta))],
            [0, 1, self.velocity * np.cos(theta)],
            [0, 0, 1]
        ])

        # Predict covariance
        self.P = self.A @ self.P @ self.A.T + self.Q


    def correction(self):
        self.R = np.eye(2 * len(self.scan_distances))

        self.C_list.clear()
        self.to_cartesian(self.scan_distances)
        self.scan_into_wheel_pose()
        for p_xi, p_yi in self.scan_xy_pairs:
            Ci = np.array([
                [-1, 0,  p_xi * np.sin(self.lidar_odom[2]) + p_yi * np.cos(self.lidar_odom[2])],
                [ 0, -1, -p_xi * np.cos(self.lidar_odom[2]) + p_yi * np.sin(self.lidar_odom[2])]
            ])
            self.C_list.append(Ci)
        
        self.C = np.vstack(self.C_list) 
        S = self.C @ self.P @ self.C.T + self.R

        self.K = self.P @ self.C.T @ np.linalg.inv(S)

        self.x_hat = self.x_hat + self.K @ self.scan_xy_vector
        self.P = self.P - self.K @ self.C @ self.P
        # print("som tu")
        self.publish_fused()

    def scan_into_wheel_pose(self):
        x0, y0, theta0 = self.wheel_odom
        scan_points = self.scan_xy_vector.reshape(-1, 2)
        Rot = np.array([
            [np.cos(theta0), -np.sin(theta0)],
            [np.sin(theta0),  np.cos(theta0)]
        ])
        scan_points_transformed = (Rot @ scan_points.T).T + np.array([x0, y0])
        self.scan_xy_vector = scan_points_transformed.flatten()


    def to_cartesian(self, scans):
        self.scan_xy_pairs.clear()
        angle = 0.0
        for distance in scans:
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)
            self.scan_xy_pairs.append([x,y])
            angle += self.scan_angle_increment
        self.scan_xy_vector = np.array(self.scan_xy_pairs).flatten()


    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = msg.pose.pose.orientation.z

        self.wheel_odom = np.array([x, y, theta])
        # print(f"Wheel odom: x={x:.3f}, y={y:.3f}, theta={theta:.3f}")

    def icp_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = msg.pose.pose.orientation.z

        self.lidar_odom = np.array([x, y, theta])

        # print(f"ICP odom: x={x:.3f}, y={y:.3f}, theta={theta:.3f}")
        
        if self.wheel_odom is not None and self.lidar_odom is not None:
            self.prediction()
            self.correction()

    def scan_callback(self, msg: LaserScan):
            self.scan_distances = msg.ranges
            self.scan_angle_increment = msg.angle_increment

    def velocity_callback(self, msg: Float64):
            self.velocity = msg.datac
            # self.get_logger().info(f'Received: {self.value:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = OdomSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

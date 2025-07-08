# republish_imu_with_cov.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMURepublisher(Node):
    def __init__(self):
        super().__init__('imu_republisher')
        self.sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.pub = self.create_publisher(Imu, '/imu/data_fixed', 10)

    def imu_callback(self, msg):
        # Inject realistic covariances
        msg.orientation_covariance = [0.05, 0, 0,
                                      0, 0.05, 0,
                                      0, 0, 0.2]  # Yaw is typically noisier
        msg.angular_velocity_covariance = [0.02, 0, 0,
                                           0, 0.02, 0,
                                           0, 0, 0.03]
        msg.linear_acceleration_covariance = [0.1, 0, 0,
                                              0, 0.1, 0,
                                              0, 0, 0.1]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMURepublisher()
    rclpy.spin(node)
    rclpy.shutdown()

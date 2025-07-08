#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState
import math

class SkidSteeringOdomNode(Node):

    def __init__(self):
        super().__init__('skid_steering_odom_node')

        # Robot parameters
        self.wheel_base = 0.65  # distance between the wheels (meters)
        self.wheel_radius = 0.05  # radius of the wheels (meters)

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Wheel state
        self.left_degree = 0.0
        self.right_degree = 0.0
        self.prev_left_degree = None
        self.prev_right_degree = None

        self.prev_time = self.get_clock().now()

        # ROS Interfaces
        self.wheel_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer_ = self.create_timer(0.1, self.update_odometry)  # 10 Hz

    def joint_state_callback(self, msg):
        self.left_degree = msg.position[0]
        self.right_degree = msg.position[1]

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt == 0:
            return

        if self.prev_left_degree is None or self.prev_right_degree is None:
            self.prev_left_degree = self.left_degree
            self.prev_right_degree = self.right_degree
            self.prev_time = current_time
            return

        # Calculate wheel displacement
        left_change = self.left_degree - self.prev_left_degree
        right_change = self.right_degree - self.prev_right_degree

        # Handle encoder overflow or abnormal jumps
        MAX_DELTA = 6.28  # 1 full wheel rotation in radians
        if abs(left_change) > MAX_DELTA or abs(right_change) > MAX_DELTA:
            self.get_logger().warn(f"[SKIP] Abnormal encoder jump - left Δ: {left_change:.4f}, right Δ: {right_change:.4f}")
            # Update previous values to avoid being stuck
            self.prev_left_degree = self.left_degree
            self.prev_right_degree = self.right_degree
            self.prev_time = current_time
            return

        # Log normal changes
        #s elf.get_logger().info(f"[ENCODER] Δleft: {left_change:.4f} rad, Δright: {right_change:.4f} rad")

        # Update previous values
        self.prev_left_degree = self.left_degree
        self.prev_right_degree = self.right_degree
        self.prev_time = current_time

        left_distance = self.wheel_radius * left_change
        right_distance = self.wheel_radius * right_change
        d = (left_distance + right_distance) / 2.0
        theta_delta = (right_distance - left_distance) / self.wheel_base

        # Update position
        self.x += d * math.cos(self.theta + theta_delta / 2.0)
        self.y += d * math.sin(self.theta + theta_delta / 2.0)
        self.theta += theta_delta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # normalize

        lin_vel_x = d / dt
        ang_vel_z = theta_delta / dt

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        q = self.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = lin_vel_x
        odom_msg.twist.twist.angular.z = ang_vel_z

        self.odom_pub.publish(odom_msg)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return (qx, qy, qz, qw)

def main(args=None):
    rclpy.init(args=args)
    node = SkidSteeringOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

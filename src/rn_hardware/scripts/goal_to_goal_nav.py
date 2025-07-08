#!/usr/bin/env python3

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import rclpy
import time
import tf_transformations as tf

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    def make_goal_pose(x, y, yaw, frame='map'):
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        q = tf.quaternion_from_euler(0, 0, yaw)

        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    goals = [
        make_goal_pose(3.0, -1.2 ,0.0),
        make_goal_pose(4.2, 0.0 ,-1.57),
        make_goal_pose(7.5, 0.0, -1.57),
        make_goal_pose(7.2, -4.8, 1.57),
        make_goal_pose(5.8, -4.8,1.57),
        make_goal_pose(0.0, 0.0,0.0),
    ]

    cycles = 2  # Number of cycles
    current_cycle = 0

    while current_cycle < cycles:
        print(f"Starting cycle {current_cycle + 1} of {cycles}...")

        for i, goal in enumerate(goals):
            print(f"Navigating to goal {i+1} in cycle {current_cycle + 1}...")
            navigator.goToPose(goal)

            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                if feedback:
                    print(f"Distance remaining: {feedback.distance_remaining:.2f} m")

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print(f"Goal {i+1} in cycle {current_cycle + 1} reached!")
            else:
                print(f"Goal {i+1} in cycle {current_cycle + 1} failed.")
                #navigator.lifecycleShutdown()
                rclpy.shutdown()
                return

            time.sleep(3)

        current_cycle += 1  # Increment cycle counter

    #navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
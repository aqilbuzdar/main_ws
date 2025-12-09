#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time

def main():
    rclpy.init()
    navigator = BasicNavigator()

    print("⏳ Waiting for Nav2 (Engine) to start...")
    navigator.waitUntilNav2Active()
    print("✅ Nav2 Ready! Driver is taking control.")

    # 1. INITIAL POSE
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)
    time.sleep(2.0)

    # 2. GOAL 1
    print("🚀 Going to Point A...")
    
    goal1 = PoseStamped()
    goal1.header.frame_id = 'map'
    goal1.header.stamp = navigator.get_clock().now().to_msg()
    goal1.pose.position.x = 0.0  # 1 meter forward
    goal1.pose.position.y = 0.0
    goal1.pose.orientation.w = 0.0

    navigator.goToPose(goal1)

    while not navigator.isTaskComplete():
        pass

    print("✅ Reached Point A!")
    
    # 3. GOAL 2 (Back Home)
    print("🏠 Returning Home...")
    goal2 = PoseStamped()
    goal2.header.frame_id = 'map'
    goal2.header.stamp = navigator.get_clock().now().to_msg()
    goal2.pose.position.x = 0.0
    goal2.pose.position.y = 0.0
    goal2.pose.orientation.w = 0.0

    navigator.goToPose(goal2)

    while not navigator.isTaskComplete():
        pass

    print("🎉 Mission Complete!")
    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()
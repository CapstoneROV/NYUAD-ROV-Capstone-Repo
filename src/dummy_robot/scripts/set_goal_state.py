#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import copy

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_plan', anonymous=True)

    # Interface to the robot
    robot = moveit_commander.RobotCommander()

    # Interface to the world surrounding the robot
    scene = moveit_commander.PlanningSceneInterface()

    # Interface to the group of joints you wish to control
    group_name = "base"  # Replace this with your actual group name
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Create a publisher for displaying trajectory in Rviz
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    # Set up the target pose for the end effector
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 1.0
    pose_target.position.x = 4
    pose_target.position.y = 0.6
    pose_target.position.z = 0.4
    move_group.set_pose_target(pose_target)

    # Perform the motion planning
    plan = move_group.plan()
    #rospy.sleep(5)

    # Create a DisplayTrajectory message for Rviz
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)

    # Publish the DisplayTrajectory message
    display_trajectory_publisher.publish(display_trajectory)
    #rospy.sleep(5)

if __name__ == '__main__':
    main()

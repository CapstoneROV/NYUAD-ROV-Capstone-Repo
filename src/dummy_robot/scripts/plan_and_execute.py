#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('uav_moveit_plan_and_execute')

    # Ensure the robot's planning group name is correctly specified
    group_name = "base"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Parse command line arguments for the goal pose
    if len(sys.argv) < 8:
        print("Usage: plan_and_execute.py x y z qx qy qz qw")
        return
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = float(sys.argv[1])
    pose_goal.position.y = float(sys.argv[2])
    pose_goal.position.z = float(sys.argv[3])
    pose_goal.orientation.x = float(sys.argv[4])
    pose_goal.orientation.y = float(sys.argv[5])
    pose_goal.orientation.z = float(sys.argv[6])
    pose_goal.orientation.w = float(sys.argv[7])

    # Set the goal state
    move_group.set_pose_target(pose_goal)

    # Perform planning and execution
#    move_group.go(wait=True)

   
   # move_group.stop()  # Ensure there is no residual movement
   # move_group.clear_pose_targets()

if __name__ == '__main__':
    main()

#!/usr/bin/env python
#Unused script
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
import tf_conversions 

def setup_orientation_constraint():
    # Define an orientation constraint for yaw-only rotation
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = "world"  # Or your base frame
    orientation_constraint.link_name = "base_link" 
    
    quaternion = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    orientation_constraint.orientation.x = quaternion[0]
    orientation_constraint.orientation.y = quaternion[1]
    orientation_constraint.orientation.z = quaternion[2]
    orientation_constraint.orientation.w = quaternion[3]
    # Allow very small deviations in roll and pitch, effectively locking them
    orientation_constraint.absolute_x_axis_tolerance = 0.001
    orientation_constraint.absolute_y_axis_tolerance = 0.001
    # Allow full rotation around Z-axis
    orientation_constraint.absolute_z_axis_tolerance = 3.14
    orientation_constraint.weight = 1.0
    
    return orientation_constraint

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('constrain_yaw_only', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "base"  # Update with the name of your robot's primary planning group
    move_group = moveit_commander.MoveGroupCommander(group_name)

    constraints = Constraints()
    constraints.orientation_constraints.append(setup_orientation_constraint())
    move_group.set_path_constraints(constraints)

    # Now, setup your goal state or let the user specify it
    # This example uses a random target for simplicity
   # move_group.set_random_target()
   # plan = move_group.plan()

    # Planning only; execute if connected to a real robot or simulation

  #  move_group.clear_path_constraints()  # Clear constraints after planning

if __name__ == '__main__':
    main()


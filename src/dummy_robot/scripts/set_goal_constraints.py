#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('apply_orientation_constraints', anonymous=True)

    # Initialize the MoveIt! commander for the "base" group.
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "base"  # Change to your move group name
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Define the orientation constraint
    orientation_constraint = OrientationConstraint()
    orientation_constraint.link_name = "base_link"  # Change as needed
    orientation_constraint.header.frame_id = "base_link"  # Change as needed
    quaternion = quaternion_from_euler(0, 0, 0)  # Assuming no roll, no pitch, yaw=0
    orientation_constraint.orientation = Quaternion(*quaternion)
    orientation_constraint.absolute_x_axis_tolerance = 0.1  # Tolerances for roll
    orientation_constraint.absolute_y_axis_tolerance = 0.1  # Tolerances for pitch
    orientation_constraint.absolute_z_axis_tolerance = 3.14  # Allow any yaw
    orientation_constraint.weight = 1.0

    # Apply the orientation constraint to the move group
    constraints = move_group.get_path_constraints()
    constraints.orientation_constraints.append(orientation_constraint)
    move_group.set_path_constraints(constraints)

    rospy.loginfo("Orientation constraints applied. You can now set the goal state from RViz.")

    # Keep the node alive while the constraints are applied
    rospy.spin()

if __name__ == '__main__':
    main()


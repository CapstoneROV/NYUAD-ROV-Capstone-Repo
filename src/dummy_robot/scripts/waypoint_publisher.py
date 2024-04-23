#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import PoseStamped
import tf.transformations

def quaternion_to_euler(x, y, z, w):
    """
    Converts quaternion to Euler roll, pitch, yaw
    """
    euler = tf.transformations.euler_from_quaternion([x, y, z, w])
    return euler  # roll, pitch, yaw

def plan_callback(data):
    pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rate = rospy.Rate(10)
    print("-------------------- Trajectory -------------------")
    for idx, point in enumerate(data.trajectory[0].multi_dof_joint_trajectory.points):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "world"
        pose_stamped.pose.position.x = point.transforms[0].translation.x
        pose_stamped.pose.position.y = point.transforms[0].translation.y
        pose_stamped.pose.position.z = point.transforms[0].translation.z
        pose_stamped.pose.orientation.x = point.transforms[0].rotation.x
        pose_stamped.pose.orientation.y = point.transforms[0].rotation.y
        pose_stamped.pose.orientation.z = point.transforms[0].rotation.z
        pose_stamped.pose.orientation.w = point.transforms[0].rotation.w

        roll, pitch, yaw = quaternion_to_euler(
            pose_stamped.pose.orientation.x,
            pose_stamped.pose.orientation.y,
            pose_stamped.pose.orientation.z,
            pose_stamped.pose.orientation.w
        )

        # Check the roll and pitch angles against the constraint
        max_angle_rad = 25.0 * (3.14159265359 / 180.0) 
        exceeded = []

        if abs(roll) > max_angle_rad:
            exceeded.append('Roll exceeds ±15 degrees')

        if abs(pitch) > max_angle_rad:
            exceeded.append('Pitch exceeds ±15 degrees')

        if exceeded:
            print("Waypoint {}: FAILURE ({})".format(idx, ', '.join(exceeded)))
        else:
            print("Waypoint {}: SUCCESS".format(idx))

        pub.publish(pose_stamped)

        rate.sleep()

def listener():
    rospy.init_node('waypoint_publisher_node', anonymous=True)
    rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, plan_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

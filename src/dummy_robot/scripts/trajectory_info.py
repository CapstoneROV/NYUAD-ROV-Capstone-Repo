#!/usr/bin/env python

import rospy
from moveit_msgs.msg import DisplayTrajectory

def callback(display_trajectory):
    print("Received planned trajectory")
    trajectory = display_trajectory.trajectory
    for traj in trajectory:
        joint_trajectory = traj.joint_trajectory
        print("Joint Names:", joint_trajectory.joint_names)
        for point in joint_trajectory.points:
            print("Positions:", point.positions)
            print("Time from Start:", point.time_from_start.to_sec())

def listener():
    rospy.init_node('trajectory_listener', anonymous=True)
    rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

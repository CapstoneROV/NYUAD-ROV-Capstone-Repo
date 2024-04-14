#!/usr/bin/env python
import rospy
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import PoseStamped

def plan_callback(data):
    pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    # 10hz -- perhaps needs changing this rate -- or need to use ros action server 
    #- which is diffuclt to integrate 
    rate = rospy.Rate(10)  
    print("-------------------- Trajectory -------------------")
    for idx, point in enumerate(data.trajectory[0].multi_dof_joint_trajectory.points):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        
        # this needs adjustments ... depending on the frame used
        pose_stamped.header.frame_id = "world" 
        
        pose_stamped.pose.position.x = point.transforms[0].translation.x
        pose_stamped.pose.position.y = point.transforms[0].translation.y
        pose_stamped.pose.position.z = point.transforms[0].translation.z
        
        pose_stamped.pose.orientation.x = point.transforms[0].rotation.x
        pose_stamped.pose.orientation.y = point.transforms[0].rotation.y
        pose_stamped.pose.orientation.z = point.transforms[0].rotation.z
        pose_stamped.pose.orientation.w = point.transforms[0].rotation.w
        
        # Print the waypoint information

        print("Waypoint {}: Position: x: {:.3f}, y: {:.3f}, z: {:.3f}, Orientation: x: {:.3f}, y: {:.3f}, z: {:.3f}, w: {:.3f}".format(
            idx,
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            pose_stamped.pose.position.z,
            pose_stamped.pose.orientation.x,
            pose_stamped.pose.orientation.y,
            pose_stamped.pose.orientation.z,
            pose_stamped.pose.orientation.w,
        ))

        #rospy.loginfo("Publishing waypoint to /mavros/setpoint_position/local (to do...)")
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

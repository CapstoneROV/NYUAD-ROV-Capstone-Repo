#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry

def handle_odometry(msg, child_frame_id):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     msg.header.stamp,
                     child_frame_id,
                     msg.header.frame_id)

def odometry_listener():
    rospy.init_node('odometry_to_tf_broadcaster')
    child_frame_id = rospy.get_param('~child_frame_id', 'base_link')  # The frame attached to the moving part of the robot
    rospy.Subscriber('/m100/odometry', Odometry, handle_odometry, child_frame_id)
    rospy.spin()

if __name__ == '__main__':
    odometry_listener()

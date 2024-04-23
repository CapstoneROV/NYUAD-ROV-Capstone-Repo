#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import PlanningScene
from octomap_msgs.msg import Octomap

class OctomapToMoveIt:
    def __init__(self):
        rospy.init_node('octomap_to_moveit')
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.planning_scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene, queue_size=10)
        # rospy.Subscriber('/rtabmap/octomap_binary', Octomap, self.octomap_callback)
        rospy.Subscriber('/converted_octomap', Octomap, self.octomap_callback)

        # Set up timing control
        self.start_time = rospy.Time.now()
        self.duration = rospy.Duration(500)  # Duration of 5 seconds

    def octomap_callback(self, octomap_msg):
        rospy.loginfo("Received OctoMap, adding to the planning scene.")
        planning_scene_msg = PlanningScene()
        planning_scene_msg.world.octomap.header = octomap_msg.header
        planning_scene_msg.world.octomap.octomap = octomap_msg
        planning_scene_msg.is_diff = True
        self.planning_scene_pub.publish(planning_scene_msg)
        rospy.loginfo("Published OctoMap to the planning scene.")

if __name__ == '__main__':
    octomap_to_moveit = OctomapToMoveIt()
    rospy.spin()


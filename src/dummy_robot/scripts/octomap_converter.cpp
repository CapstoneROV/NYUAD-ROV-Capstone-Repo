#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

ros::Publisher octomap_pub; 
void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    octomap::ColorOcTree* colorTree = dynamic_cast<octomap::ColorOcTree*>(tree);

    if (colorTree) {
        octomap::OcTree ocTree(colorTree->getResolution());
        for (auto it = colorTree->begin_leafs(), end = colorTree->end_leafs(); it != end; ++it) {
            if (colorTree->isNodeOccupied(*it)) {
                ocTree.updateNode(it.getX(), it.getY(), it.getZ(), true);
            }
        }
        octomap_msgs::Octomap octomapMsg;
        octomap_msgs::binaryMapToMsg(ocTree, octomapMsg);
        octomapMsg.header.frame_id = "map";
        octomapMsg.header.stamp = ros::Time::now();

        octomap_pub.publish(octomapMsg);
    }

    delete tree; 
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_converter_standalone");
    ros::NodeHandle nh;
    octomap_pub = nh.advertise<octomap_msgs::Octomap>("converted_octomap", 1, true); 
    ros::Subscriber octomap_sub = nh.subscribe("rtabmap/octomap_binary", 1, octomapCallback);

    ros::spin();
    return 0;
}

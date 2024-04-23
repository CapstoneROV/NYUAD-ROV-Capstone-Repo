#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/PlanningScene.h>

void createAndApplyOctomap(ros::NodeHandle& nh) {
    // Wall parameters
    double wall_height = 2.0; // meters
    double wall_width = 0.1;  // meters
    double wall_length = 5.0; // meters
    double wall_x_start = 1.0; // meters
    double wall_y_start = -2.5; // meters
    double wall_z_start = 0.0;  // meters
    double resolution = 0.05;  // octomap resolution in meters

    // Initialize OcTree with specified resolution
    octomap::OcTree ocTree(resolution);

    // Populate OcTree with a wall-like structure
    for(double x = wall_x_start; x < wall_x_start + wall_width; x += resolution) {
        for(double y = wall_y_start; y < wall_y_start + wall_length; y += resolution) {
            for(double z = wall_z_start; z < wall_z_start + wall_height; z += resolution) {
                ocTree.updateNode(x, y, z, true); // Mark node as occupied
            }
        }
    }

    // Convert OcTree to an Octomap message
    octomap_msgs::Octomap octomapMsg;
    octomap_msgs::binaryMapToMsg(ocTree, octomapMsg);
    octomapMsg.header.frame_id = "map"; // Ensure this frame_id matches your setup
    octomapMsg.header.stamp = ros::Time::now();

    // Construct PlanningScene message
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.octomap.octomap = octomapMsg;
    planning_scene.world.octomap.header.frame_id = "map"; // Match frame_id
    planning_scene.is_diff = true; // Indicate this message should be merged with the existing scene

    // Service client to apply the PlanningScene
    ros::ServiceClient client = nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    client.waitForExistence(); // Ensure the service is available before proceeding
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planning_scene;

    if (client.call(srv)) {
        ROS_INFO("Planning scene successfully updated with octomap.");
    } else {
        ROS_ERROR("Failed to update planning scene with octomap.");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_to_planning_scene");
    ros::NodeHandle nh;

    // Function to create and apply OctoMap to the planning scene
    createAndApplyOctomap(nh);

    ros::spin();
    return 0;
}

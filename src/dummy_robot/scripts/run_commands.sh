#!/bin/bash
# Play the ROS ba
# Run your custom script
$(rospack find dummy_robot)/scripts/octomap_converter
rosbag play octomap_sample.bag.bz2 
// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the node definition for RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf


// #include "rrt/rrt.h"
#include "rrt/rrt_v2.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rrt");
    ros::NodeHandle nh;
    RRT rrt(nh);
    ros::spin();
    return 0;
}

#include <ros/ros.h>
#include "StackBlocks.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "block_world_node");
    ros::NodeHandle nh;
    StackBlocks stacker(nh);
    ros::spin();
    return 0;
}

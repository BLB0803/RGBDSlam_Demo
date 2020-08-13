#include <iostream>
#include "base.h"
int main(int argc, char**argv){
    ros::init(argc,argv,"slam");
    ros::NodeHandle n;
    ROS_INFO("running");
    SLAM slam(n);
    ros::spin();
    ros::shutdown();
    return 0;
}

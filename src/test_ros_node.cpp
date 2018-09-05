#include "laser_ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_test");
    LaserRos my_laser;
    ros::spin();
    return 0;
}

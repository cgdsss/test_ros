#include "laser_ros.h"

LaserRos::LaserRos()
{
    ros::NodeHandle private_nh("~");
    private_nh.param("a", a_, 2.0);
    private_nh.param("b", b_, 2.0);

    laser_subscriber_ = nh_.subscribe("/scan", 1, &LaserRos::laserCallback, this);
    markers_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/scan_filter", 1);
}

void LaserRos::laserCallback(const sensor_msgs::LaserScanConstPtr laser_msg)
{

}

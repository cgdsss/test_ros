#include "laser_ros.h"

LaserRos::LaserRos()
{
    ros::NodeHandle private_nh("~");
    private_nh.param("a", a_, 2.0);
    private_nh.param("b", b_, 2.0);
    laser_buffer_ = new RosLaserData[LaserBufferLen];
    curLaserBufferIndex = -1;

    laser_subscriber_ = nh_.subscribe("/scan", 1, &LaserRos::laserCallback, this);
    markers_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/scan_filter", 1);
}

LaserRos::~LaserRos()
{
    for (int i = 0; i < LaserBufferLen; i++)
    {
      if(laser_buffer_[i].data != NULL)
        delete[] laser_buffer_[i].data;
    }
    delete[] laser_buffer_;
    std::cout << "Free laser buffer" << std::endl;
}

void LaserRos::laserCallback(const sensor_msgs::LaserScanConstPtr laser_msg)
{
    laserBufferMutex.lock();
    laser_num_ = laser_msg->ranges.size();
    laser_angle_increment_ = laser_msg->angle_increment;
    laser_angle_min_ = laser_msg->angle_min;
    laser_angle_max_ = laser_msg->angle_max;
    curLaserBufferIndex = (curLaserBufferIndex + 1) % LaserBufferLen;
    ROS_INFO("%d", curLaserBufferIndex);
    RosLaserData & laserData = laser_buffer_[curLaserBufferIndex];
    if (laserData.data == NULL)
      laserData.data = new double[laser_num_];

    laserData.stamp = laser_msg->header.stamp;
    for (int i = 0; i < laser_num_; i++)
        laserData.data[i] = laser_msg->ranges[i];
    laserBufferMutex.unlock();
}

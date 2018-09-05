#include "laser_ros.h"
#include <math.h>

LaserRos::LaserRos()
{
    ros::NodeHandle private_nh("~");
    private_nh.param("a", a_, 2.0);
    private_nh.param("b", b_, 2.0);
    laser_buffer_ = new RosLaserData[LaserBufferLen];
    curLaserBufferIndex = -1;

    laser_subscriber_ = nh_.subscribe("/scan", 1, &LaserRos::laserCallback, this);
    markers_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/scan_filter", 1);

    obstacle_thread_ = new boost::thread(boost::bind(&LaserRos::obstacleloop, this));
    is_run_ = true;
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
    is_run_ = false;
    delete obstacle_thread_;
}

void LaserRos::laserCallback(const sensor_msgs::LaserScanConstPtr laser_msg)
{
    laserBufferMutex.lock();
    laser_num_ = laser_msg->ranges.size();
    laser_angle_increment_ = laser_msg->angle_increment;
    laser_angle_min_ = laser_msg->angle_min;
    laser_angle_max_ = laser_msg->angle_max;
    laser_link_ = laser_msg->header.frame_id;
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

void LaserRos::obstacleloop()
{
    ros::Rate r(20);
    while(is_run_)
    {
        if (curLaserBufferIndex != -1)
        {
            obstacles_.clear();
            laserBufferMutex.lock();
            ROS_INFO("run thread");
            RosLaserData & laserData = laser_buffer_[curLaserBufferIndex];
            for (int i = 0; i < laser_num_; i++)
            {
                double theta = laser_angle_min_ + laser_angle_increment_ * i;
                double r = laserData.data[i];
                double x = r * cos(theta);
                double y = r * sin(theta);
                if (a_ > fabs(x) && b_ > fabs(y))
                {
                    Point2f tmp(x, y);
                    obstacles_.push_back(tmp);
                }
            }
            laserBufferMutex.unlock();
            std::cout << obstacles_.size() << std::endl;
            publishObstacles(obstacles_);
            r.sleep();
        }
    }
}

void LaserRos::publishObstacles(std::vector<Point2f> &obstacles)
{
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.clear();
    int id = 0;
    visualization_msgs::Marker marker;
    marker.header.frame_id = laser_link_;
    marker.header.stamp = ros::Time::now();
    marker.color.a = 1;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.ns = "obstacle";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    for (std::vector<Point2f>::iterator it = obstacles.begin(); it != obstacles.end(); it++)
    {

        marker.pose.position.x = it->x;
        marker.pose.position.y = it->y;
        marker.id = id;
        marker_array.markers.push_back(marker);
        id ++;
    }
    markers_publisher_.publish(marker_array);
}

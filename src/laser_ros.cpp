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

    laser_service_ = nh_.advertiseService("/scan_server", &LaserRos::laserSrvCallback, this);

    obstacle_thread_ = new boost::thread(boost::bind(&LaserRos::obstacleloop, this));
    is_run_ = true;
    ROS_INFO("a: %f, b: %f", a_, b_);
    ROS_INFO("Everything is ready.");
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
    //ROS_INFO("%d", curLaserBufferIndex);
    RosLaserData & laserData = laser_buffer_[curLaserBufferIndex];
    if (laserData.data == NULL)
      laserData.data = new double[laser_num_];

    laserData.stamp = laser_msg->header.stamp;
    for (int i = 0; i < laser_num_; i++)
        laserData.data[i] = laser_msg->ranges[i];
    laserBufferMutex.unlock();
}

bool LaserRos::laserSrvCallback(test_ros::GetObstacles::Request &req, test_ros::GetObstacles::Response &res)
{
    int flag = req.flag;
    if (curLaserBufferIndex != -1)
    {
        laserBufferMutex.lock();
        if (flag == -1)
        {
            RosLaserData& laser_data = laser_buffer_[curLaserBufferIndex];
            Point2f res_point;
            if (getNearestObstacle(laser_data, res_point) == true)
            {
                geometry_msgs::PointStamped ps;
                ps.header.frame_id = laser_link_;
                ps.header.stamp = laser_data.stamp;
                ps.header.seq = curLaserBufferIndex;
                ps.point.x = res_point.x;
                ps.point.y = res_point.y;
                res.obstacles.push_back(ps);
            }
            ROS_INFO("Return the latest data");
            laserBufferMutex.unlock();
            return true;
        }
        else if (flag == 0)
        {
            for (int i = 0; i < LaserBufferLen; i++)
            {
                RosLaserData& laser_data = laser_buffer_[i];
                if (laser_data.data != NULL)
                {
                    Point2f res_point;
                    if (getNearestObstacle(laser_data, res_point) == true)
                    {
                        geometry_msgs::PointStamped ps;
                        ps.header.frame_id = laser_link_;
                        ps.header.stamp = laser_data.stamp;
                        ps.header.seq = i;
                        ps.point.x = res_point.x;
                        ps.point.y = res_point.y;
                        res.obstacles.push_back(ps);
                    }
                }
            }
            ROS_INFO("Return all data");
            laserBufferMutex.unlock();
            return true;
        }
        else if (flag > 0 && flag <= LaserBufferLen)
        {
            RosLaserData& laser_data = laser_buffer_[flag-1];
            Point2f res_point;
            if (getNearestObstacle(laser_data, res_point) == true)
            {
                geometry_msgs::PointStamped ps;
                ps.header.frame_id = laser_link_;
                ps.header.stamp = laser_data.stamp;
                ps.header.seq = flag-1;
                ps.point.x = res_point.x;
                ps.point.y = res_point.y;
                res.obstacles.push_back(ps);
            }
            ROS_INFO("Return the %dth data", flag);
            laserBufferMutex.unlock();
            return true;
        }
        else
        {
            ROS_WARN("Bad request");
            laserBufferMutex.unlock();
            return false;
        }
    }

}

bool LaserRos::getNearestObstacle(RosLaserData &laser, Point2f &point)
{
    int index = -1;
    double dist = 9999;
    Point2f tmp_point;
    for (int i = 0; i < laser_num_; i++)
    {
        double theta = laser_angle_min_ + laser_angle_increment_ * i;
        double r = laser.data[i];
        double x = r * cos(theta);
        double y = r * sin(theta);
        double tmp = x * x + y * y;
        if (tmp < dist)
        {
            dist = tmp;
            index = i;
            tmp_point.x = x;
            tmp_point.y = y;
        }
    }
    if (index != -1)
    {
        point = tmp_point;
        return true;
    }
    else
        return false;
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
            //ROS_INFO("run thread");
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
            //std::cout << obstacles_.size() << std::endl;
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

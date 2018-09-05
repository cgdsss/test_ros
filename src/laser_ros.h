#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/thread.hpp>

class RosLaserData
{
public:
  RosLaserData()
  {
    stamp = ros::Time(0);
    data = NULL;
  }
  ros::Time stamp;
  double* data;
};

class LaserRos
{
public:
    LaserRos();
    ~LaserRos();
private:
    ros::NodeHandle nh_;
    double a_;
    double b_;

    boost::recursive_mutex laserBufferMutex;
    RosLaserData *laser_buffer_;
    int curLaserBufferIndex;
    float laser_angle_min_;
    float laser_angle_max_;
    float laser_angle_increment_;
    int laser_num_;
    const static int LaserBufferLen = 20;

    ros::Subscriber laser_subscriber_;
    void laserCallback(const sensor_msgs::LaserScanConstPtr laser_msg);

    ros::Publisher markers_publisher_;
    ros::ServiceServer laser_service_;

    boost::thread* obstacle_thread_;
    void obstacleloop();
    bool is_run_;
};

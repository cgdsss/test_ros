#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/thread.hpp>
#include <test_ros/GetObstacles.h>
#include <geometry_msgs/PointStamped.h>

template<class T>
class Point2
{
public:
    Point2(){}
    Point2(T a, T b){x = a; y = b;}
    T x;
    T y;
};
typedef Point2<float> Point2f;

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
    std::string laser_link_;
    const static int LaserBufferLen = 20;

    ros::Subscriber laser_subscriber_;
    void laserCallback(const sensor_msgs::LaserScanConstPtr laser_msg);

    ros::Publisher markers_publisher_;
    void publishObstacles(std::vector<Point2f> &obstacles);

    ros::ServiceServer laser_service_;
    bool laserSrvCallback(test_ros::GetObstacles::Request  &req,
                     test_ros::GetObstacles::Response &res );
    bool getNearestObstacle(RosLaserData& laser, Point2f &point);

    boost::thread* obstacle_thread_;
    void obstacleloop();
    bool is_run_;
    std::vector<Point2f> obstacles_;
};

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>

class LaserRos
{
public:
    LaserRos();
private:
    ros::NodeHandle nh_;
    double a_;
    double b_;

    ros::Subscriber laser_subscriber_;
    void laserCallback(const sensor_msgs::LaserScanConstPtr laser_msg);

    ros::Publisher markers_publisher_;

    ros::ServiceServer laser_service_;
};

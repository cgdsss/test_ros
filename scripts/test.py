#!/usr/bin/python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from test_ros.srv import *

class TestRos(object):
    """docstring for TestRos"""
    def __init__(self):
        self.scan_filter_sub = rospy.Subscriber('/scan_filter', MarkerArray, self.scan_filter_callback)
        self.scan_client_sub = rospy.Subscriber('/scan_client', Bool, self.scan_client_callback)
        self.point_cloud_pub = rospy.Publisher("/scan_point", PointCloud2, queue_size=10)
        self.srv_client = rospy.ServiceProxy('/scan_server', GetObstacles)

    def scan_filter_callback(self, msg):
        points = []
        cloud = PointCloud2()
        for marker in msg.markers:
            point = [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z]
            points.append(point[:])
        cloud = pc2.create_cloud_xyz32(cloud.header, points)
        cloud.header.stamp = rospy.Time.now()
        cloud.header.frame_id = msg.markers[0].header.frame_id
        self.point_cloud_pub.publish(cloud)

    def scan_client_callback(self, msg):
        if msg.data == True:
            res = self.srv_client(0)
            for r in res.obstacles:
                print "id: ", r.header.seq
                print "time: ", str(r.header.stamp)
                print "frame_id: ", r.header.frame_id
                print "point: ", r.point.x, r.point.y, r.point.z


if __name__ == '__main__':
    rospy.init_node('test_ros_py', anonymous=True)
    my_test = TestRos()
    rospy.spin()

#!/usr/bin/env python3
# coding: utf-8
import rospy
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

class PathPublisher:
    def __init__(self):
        rospy.init_node('path_publisher', anonymous=True)
        color_r = rospy.get_param('~color_r')
        color_g = rospy.get_param('~color_g')
        color_b = rospy.get_param('~color_b')
        self.color =  ColorRGBA(r=color_r, g=color_g, b=color_b, a=1.0)
        self.name = rospy.get_param('~name')
        self.path = []
        self.marker_pub = rospy.Publisher('path_marker', Marker, queue_size=10)
        rospy.Subscriber('odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.path.append(pose)
        self.publish_path(msg.header.frame_id)

    def publish_path(self, frame_id="world"):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = self.name
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Line width
        marker.color = self.color
        marker.points = [p.position for p in self.path]
        self.marker_pub.publish(marker)

if __name__ == '__main__':
    pp = PathPublisher()
    rospy.spin()

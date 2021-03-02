#!/usr/bin/env python2.7

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Point

class WTAVisualizer():
    def __init__(self, target_position):

        self.marker_pub = rospy.Publisher("/visualization/demo", Marker, queue_size=10)
        self.marker_key_index = 0
        self.odom_marker = Marker()
        self.target_marker = Marker()
        self.initialize_odom_markers()
        self.initialize_target_markers(target_position)

    def initialize_odom_markers(self):
        odom_color = ColorRGBA(1, 0, 0, 1)
        self.odom_marker.header.frame_id = "/map"
        self.odom_marker.type = Marker.SPHERE_LIST
        self.odom_marker.action = Marker.ADD
        self.odom_marker.pose.position.x = 0
        self.odom_marker.pose.position.y = 0
        self.odom_marker.pose.position.z = 0
        self.odom_marker.pose.orientation.x = 0
        self.odom_marker.pose.orientation.y = 0
        self.odom_marker.pose.orientation.z = 0
        self.odom_marker.pose.orientation.w = 1
        self.odom_marker.scale = Vector3(0.5, 0.5, 0.5)
        self.odom_marker.lifetime = rospy.Duration()
        self.odom_marker.color = odom_color

    def initialize_target_markers(self, target_position):
        self.target_marker.color = ColorRGBA(1, 0, 1, 1)
        self.target_marker.header.frame_id = "/map"
        self.target_marker.type = Marker.POINTS
        self.target_marker.action = Marker.ADD
        self.target_marker.pose.position.x = 0
        self.target_marker.pose.position.y = 0
        self.target_marker.pose.position.z = 0
        self.target_marker.pose.orientation.x = 0
        self.target_marker.pose.orientation.y = 0
        self.target_marker.pose.orientation.z = 0
        self.target_marker.pose.orientation.w = 1
        self.target_marker.scale = Vector3(0.2, 0.2, 0.2)
        self.target_marker.id = self.marker_key_index
        self.target_marker.ns = "TargetMarker"
        self.target_marker.lifetime = rospy.Duration()

        for target in target_position:
            target_point = Point()
            target_point.x = target["position"][0]
            target_point.y = target["position"][1]
            self.target_marker.points.append(target_point)


    def visualize_target(self):

        self.marker_pub.publish(self.target_marker)

    def visualize_robot(self, position, ns):

        self.odom_marker.id = self.marker_key_index
        self.odom_marker.ns = ns
        self.odom_marker.points = []
        self.odom_marker.points.append(position)

        self.marker_pub.publish(self.odom_marker)

    def visualize_communication(self, agent1, agent2, brighten=False):
        print("test")





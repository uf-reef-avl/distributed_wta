#!/usr/bin/env python2.7

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Point
import tf2_ros

class WTAVisualizer():
    def __init__(self, target_position):

        self.marker_pub = rospy.Publisher("/visualization/demo", Marker, queue_size=10)
        self.marker_key_index = 0
        self.odom_marker = Marker()
        self.target_marker = Marker()
        self.arrow_marker = Marker()
        self.initialize_odom_markers()
        self.initialize_target_markers(target_position)
        self.initialize_arrow_markers()

        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

    def initialize_odom_markers(self):
        odom_color = ColorRGBA(0, 0, 0, 1)
        self.odom_marker.header.frame_id = "/map"
        # self.odom_marker.type = Marker.SPHERE_LIST
        self.odom_marker.type = Marker.MESH_RESOURCE
        self.odom_marker.mesh_resource = "package://turtlebot3_description/meshes/bases/burger_base.stl"
        self.odom_marker.action = Marker.ADD
        self.odom_marker.pose.position.x = 0
        self.odom_marker.pose.position.y = 0
        self.odom_marker.pose.position.z = 0
        self.odom_marker.pose.orientation.x = 0
        self.odom_marker.pose.orientation.y = 0
        self.odom_marker.pose.orientation.z = 0
        self.odom_marker.pose.orientation.w = 1
        self.odom_marker.scale = Vector3(0.005,0.005,0.005)
        # self.odom_marker.scale = Vector3(0.5, 0.5, 0.5)
        self.odom_marker.lifetime = rospy.Duration()
        self.odom_marker.color = odom_color

    def initialize_target_markers(self, target_position):
        self.target_marker.color = ColorRGBA(1, 0, 0, 1)
        self.target_marker.header.frame_id = "/map"
        # self.target_marker.type = Marker.POINTS
        self.target_marker.type = Marker.MESH_RESOURCE
        self.target_marker.mesh_resource = "package://wta_distributed_optimization/resources/Dragon _stl.stl"
        self.target_marker.action = Marker.ADD
        self.target_marker.pose.position.x = 0
        self.target_marker.pose.position.y = 0
        self.target_marker.pose.position.z = 0
        self.target_marker.pose.orientation.x = 0
        self.target_marker.pose.orientation.y = 0
        self.target_marker.pose.orientation.z = 0
        self.target_marker.pose.orientation.w = 1
        self.target_marker.scale = Vector3(0.01,0.01,0.01)
        self.target_marker.id = self.marker_key_index
        self.target_marker.ns = "TargetMarker"
        self.target_marker.lifetime = rospy.Duration()
        self.target_marker.text = 'Targets'

        self.target_positions = target_position

        # for target in target_position:
        #     target_point = Point()
        #     target_point.x = target["position"][0]
        #     target_point.y = target["position"][1]
        #     self.target_marker.points.append(target_point)

    def initialize_arrow_markers(self):
        self.arrow_rgb = [1, 0.5, 0.5]
        arrow_color = ColorRGBA(self.arrow_rgb[0], self.arrow_rgb[1], self.arrow_rgb[2], 0.25)
        self.arrow_marker.header.frame_id = "/map"
        self.arrow_marker.type = Marker.ARROW
        self.arrow_marker.action = Marker.ADD
        self.arrow_marker.pose.position.x = 0
        self.arrow_marker.pose.position.y = 0
        self.arrow_marker.pose.position.z = 0
        self.arrow_marker.pose.orientation.x = 0
        self.arrow_marker.pose.orientation.y = 0
        self.arrow_marker.pose.orientation.z = 0
        self.arrow_marker.pose.orientation.w = 1
        self.arrow_marker.scale = Vector3(0.01, 0.01, 0.01)
        self.arrow_marker.lifetime = rospy.Duration()
        self.arrow_marker.color = arrow_color

    def visualize_target(self):

        id = 0
        for target in self.target_positions:
            temp_target = Marker()
            temp_target = self.target_marker
            temp_target.id = id
            id += 1
            temp_target.pose.position.x = target["position"][0]
            temp_target.pose.position.y = target["position"][1]
            # self.target_marker.points.append(target_point)
            self.marker_pub.publish(temp_target)

    def visualize_robot(self, position, ns):

        self.odom_marker.id = self.marker_key_index
        self.odom_marker.ns = ns
        self.odom_marker.points = []
        # self.odom_marker.points.append(position)
        self.odom_marker.pose.position.x = position.x
        self.odom_marker.pose.position.y = position.y
        self.odom_marker.pose.position.z = 0

        self.marker_pub.publish(self.odom_marker)

    def visualize_communication(self, my_number, communicated_agent_number, dual=False, brighten=False):


        my_name_target_frame = 'agent_0' + str(my_number) + '/odom'
        my_name_parent_frame = 'agent_0' + str(my_number) + '/base_footprint'

        try:
            trans = self.tfBuffer.lookup_transform(my_name_target_frame, my_name_parent_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return

        agent1_point = Point()
        agent1_point.x = trans.transform.translation.x
        agent1_point.y = trans.transform.translation.y

        agent_target_frame = 'agent_0' + str(communicated_agent_number) + '/odom'
        agent_parent_frame = 'agent_0' + str(communicated_agent_number) + '/base_footprint'
        try:
            trans = self.tfBuffer.lookup_transform(agent_target_frame, agent_parent_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return

        agent2_point = Point()
        agent2_point.x = trans.transform.translation.x
        agent2_point.y = trans.transform.translation.y

        self.arrow_marker.ns = 'arrow_' + str(my_number) + str(communicated_agent_number)
        self.odom_marker.id = self.marker_key_index
        self.arrow_marker.points = []
        self.arrow_marker.points = [agent1_point, agent2_point]

        # rgb_value = self.arrow_rgb

        if dual:
            rgb_value = [1, 1, 0]
        else:
            rgb_value = self.arrow_rgb

        if brighten:
            color = ColorRGBA(rgb_value[0], rgb_value[1], rgb_value[2], 1)
        else:
            color = ColorRGBA(rgb_value[0], rgb_value[1], rgb_value[2], 0.05)

        self.arrow_marker.color = color
        self.marker_pub.publish(self.arrow_marker)




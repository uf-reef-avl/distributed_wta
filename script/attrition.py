#!/usr/bin/env python2.7

import rospy
import rosnode
from wta_distributed_optimization.srv import *
from geometry_msgs.msg import Twist

def quad_callback(req):

    node_list = rosnode.get_node_names(req.namespace)
    for i, s in enumerate(node_list):
        if 'opt' in s or 'cont' in s:
            delt = rosnode.kill_nodes([s])
            print(delt)

    topic_name = req.namespace + '/cmd_vel'
    vel_pub = rospy.Publisher(topic_name, Twist, queue_size=10)
    message = Twist()
    message.linear.x = 0
    message.angular.z = 0.5

    for i in range(0, 100):
        vel_pub.publish(message)
        rospy.sleep(0.05)


    return kill_quadResponse(True)

if __name__ == "__main__":
    rospy.init_node('attrition', anonymous=False)
    print('test')

    s = rospy.Service('kill_quad', kill_quad, quad_callback)
    rospy.spin()
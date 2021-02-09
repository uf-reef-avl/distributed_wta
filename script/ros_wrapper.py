#!/usr/bin/env python
"""ROS Wrapper for the Distributed WTA
This node is a ROS Wrapper for the distributed optimization. The code will run a ROS node and is intended to run on an
agent eg. tbot. This node will subscribe to primal and dual variable from other agents and perform optimizations
accordingly.
"""
import rospy
import numpy as np
import scipy.linalg as la
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
import inputs
import communicate
import time

class WTAOptimization():
    def __init__(self):

        self.num_weapons = rospy.get_param("~num_weapons", 3) # number of weapons/agent. Obtained from ROS Param
        self.num_targets = rospy.get_param("~num_targets", 2) # number of weapons/agent. Obtained from ROS Param
        self.k_max = rospy.get_param("~max_iter", 100) # number of weapons/agent. Obtained from ROS Param
        self.primal_pub_probability = rospy.get_param("~primal_pub_prob", 0.2) # number of weapons/agent. Obtained from ROS Param
        self.dual_pub_probability = rospy.get_param("~dual_pub_prob", 0.2) # number of weapons/agent. Obtained from ROS Param
        self.target_positions = rospy.get_param("~target_position") # number of weapons/agent. Obtained from ROS Param

        ## Assigning agent numbers
        # In order for the agent to assign weapon/agent id by themselves, we will rely on the ROS namespace which
        # will be declared in the launch file. The namespace can be used to identify the agent and also establish the
        # topics to publish and subscribe to.

        my_namespace = rospy.get_namespace() # number of weapons/agent. Obtained from ROS Param
        # my_namespace = "robot01" # adding for  debugging # number of weapons/agent. Obtained from ROS Param
        self.my_number = int(my_namespace[-2]) # number of weapons/agent. Obtained from ROS Param
        print "my number is " + str(self.my_number)
        # print "publishing threshold is " + str(self.primal_pub_prob)

        ## This line will generate a list of all the primal agents we have. Originally intended to have 2 primal and
        # 1 dual agents for each target. Weapons list if similar but the agent on which this node will run on will be
        # removed
        self.my_primal_variable_idx = range(self.my_number * self.num_targets, (self.my_number+1) * self.num_targets)
        self.my_dual_variable_idx = self.my_number
        self.weapon_list = range(0, self.num_weapons)
        self.weapon_list.pop(self.my_number)

        for i in [x for x in xrange(self.num_weapons) if x!=self.my_number]:
            primal_topic = "/primal_" + str(i)
            rospy.Subscriber(primal_topic, Float32MultiArray, self.primal_callback, i)
            dual_topic = "/dual_" + str(i)
            rospy.Subscriber(dual_topic, Float64, self.dual_callback, i)
            # Kat - agents only need to share their primal block values and dual scalar value. 
        pub_topic = "/primal_" + str(self.my_number)
        self.primal_pub = rospy.Publisher(pub_topic, Float32MultiArray, queue_size=10)
        pub_topic = "/dual_" + str(self.my_number)
        self.dual_pub = rospy.Publisher(pub_topic, Float64, queue_size=10)

        self.goal_pose_pub = rospy.Publisher("goal_pose", PoseStamped, queue_size=10)

        self.delta = rospy.get_param("delta", 0.01) # dual regularization parameter
        self.rho = rospy.get_param("rho", self.delta/(self.delta ** 2 + 2)) # dual step-size
        self.gamma = rospy.get_param("gamma", 1) # primal step-size

        self.n = self.num_weapons * self.num_targets    #size of primal variable
        self.m = self.num_weapons                       #size of dual variable
        pBlocks = self.num_targets*np.arange(self.num_weapons)

        #initialize variables
        self.mu = np.zeros(self.m)  #shared in primal and dual updates
        self.x = .5*np.ones(self.n) #shared in primal and dual updates
        self.stop_optimization = False
        self.update_dual_flag = False
        self.delay = 1
        
        #initialize algorithm
        from DACOA.algorithm import DACOA
        self.opt = DACOA(self.delta,self.gamma,self.rho, self.n, self.m) #TODO: add documentation
        self.opt.defBlocks(pBlocks,np.arange(self.m)) #TODO: add documentation
        self.opt.useScalars() #TODO: add documentation
        self.a=self.opt.xBlocks[self.my_number]  #lower boundary of primal block (included)
        self.b=self.opt.xBlocks[self.my_number+1] #upper boundary of primal block (not included)
        self.optimization()

    def primal_callback(self, msg, vehicle_num):
        if vehicle_num in self.weapon_list:
            self.weapon_list.pop(self.weapon_list.index(vehicle_num))

        if not self.weapon_list:
            self.delay = 0.05
            self.update_dual_flag = True
            self.weapon_list = range(0, self.num_weapons)
            self.weapon_list.pop(self.my_number)

        self.stop_optimization = True
        idx = range(vehicle_num * self.num_targets, (vehicle_num + 1) * self.num_targets)
        self.x[idx] = msg.data
        self.stop_optimization = False
        self.optimization()

    def dual_callback(self, msg, vehicle_num):
        self.mu[vehicle_num] = msg.data
        self.weapon_list = range(0, self.num_weapons)
        self.weapon_list.pop(self.my_number)

    def optimization(self):
        convdiff = 1
        k = 0
        while convdiff > 10 ** -8 and not self.stop_optimization and not rospy.is_shutdown() and k < self.k_max:
            if self.update_dual_flag:
                # print "update the duals"
                self.update_duals()
                self.update_dual_flag = False

            xUpdated = self.opt.singlePrimal(self.x, self.mu, self.my_number)
            convdiff = la.norm(self.x - xUpdated)
            self.x = np.copy(xUpdated)
            k += 1
            time.sleep(self.delay)

            if np.random.normal(loc=0, scale=1) < self.primal_pub_probability:
                pub_msg = Float32MultiArray()
                pub_msg.data = self.x[self.my_primal_variable_idx]  # publishes just the block
                self.primal_pub.publish(pub_msg)

            assignment = np.unravel_index(np.argmax(self.x[self.my_primal_variable_idx]), self.x.shape)  # get the index
            # print "Robot " + str(self.my_number) + " is going to destroy target " + str(assignment[0])
            # print "Robot " + str(self.my_number) + " primal " + str(self.x[self.my_primal_variable_idx] )
            # print "Robot " + str(self.my_number) + " convergence " + str(convdiff) + " number of steps " + str(k)
            setpoint_msg = self.pose_msg_from_dict(self.target_positions[assignment[0]])
            self.goal_pose_pub.publish(setpoint_msg)

    def update_duals(self):
        muUpdated = self.opt.singleDual(self.x, self.mu[self.my_number], self.my_number)
        self.mu[self.my_number] = np.copy(muUpdated)
        pub_msg = Float64()
        pub_msg.data = self.mu[self.my_number]
        if np.random.normal(loc=0, scale=1) < self.dual_pub_probability:
            self.dual_pub.publish(pub_msg)

    def pose_msg_from_dict(self, target_dictionary):
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = target_dictionary["position"][0]
        pose_msg.pose.position.y = target_dictionary["position"][1]
        pose_msg.pose.position.z = target_dictionary["position"][2]

        pose_msg.pose.orientation.x = 0
        pose_msg.pose.orientation.y = 0
        pose_msg.pose.orientation.z = 0
        pose_msg.pose.orientation.w = 1
        return pose_msg


if __name__ == '__main__':
    rospy.init_node('Distributed_WTA', anonymous=False)
    obj = WTAOptimization()
    rospy.spin()
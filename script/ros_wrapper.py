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

        self.num_weapons = rospy.get_param("num_weapons", 3) # number of weapons/agent. Obtained from ROS Param
        self.num_targets = rospy.get_param("num_targets", 2) # number of weapons/agent. Obtained from ROS Param
        self.k_max = rospy.get_param("max_iter", 100) # number of weapons/agent. Obtained from ROS Param
        self.publishing_threshold = rospy.get_param("pub_prob", 0.2) # number of weapons/agent. Obtained from ROS Param
        self.target_positions = rospy.get_param("target_position") # number of weapons/agent. Obtained from ROS Param

        ## Assigning agent numbers
        # In order for the agent to assign weapon/agent id by themselves, we will rely on the ROS namespace which
        # will be declared in the launch file. The namespace can be used to identify the agent and also establish the
        # topics to publish and subscribe to.

        # my_namespace = rospy.get_namespace() # number of weapons/agent. Obtained from ROS Param
        my_namespace = "robot01" # adding for  debugging # number of weapons/agent. Obtained from ROS Param
        self.my_number = int(my_namespace[-2]) # number of weapons/agent. Obtained from ROS Param
        print self.my_number

        ## This line will generate a list of all the primal agents we have. Originally intended to have 2 primal and
        # 1 dual agents for each target. Weapons list if similar but the agent on which this node will run on will be
        # removed
        self.my_primal_variable_idx = range(self.my_number * self.num_targets, (self.my_number+1) * self.num_targets)
        self.my_dual_variable_idx = self.my_number
        self.weapon_list = range(0, self.num_weapons)
        self.weapon_list.pop(self.my_number)
        print(self.weapon_list)

        for i in [x for x in xrange(self.num_weapons) if x!=self.my_number]:
            primal_topic = "/primal_" + str(i)
            rospy.Subscriber(primal_topic, Float32MultiArray, self.primal_callback, i)
            dual_topic = "/dual_" + str(i)
            rospy.Subscriber(dual_topic, Float32MultiArray, self.dual_callback, i)
        # TODO: Revisit this: what is the dimension of the primal and duals being shared. @Kat or @Prashant
            # Kat - agents only need to share their primal block values and dual scalar value. 
        pub_topic = "/primal_" + str(self.my_number)
        self.primal_pub = rospy.Publisher(pub_topic, Float32MultiArray, queue_size=10)
        pub_topic = "/dual_" + str(self.my_number)
        self.dual_pub = rospy.Publisher(pub_topic, Float32MultiArray, queue_size=10)

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
        elif not self.weapon_list:
            self.update_dual_flag = True    #TODO: is this only set to true after an update is received from all agents? Yes
            self.weapon_list = range(0, self.num_weapons)
            self.weapon_list.pop(self.my_number)

        self.stop_optimization = True
        print self.weapon_list
        self.x[vehicle_num] = msg.data[0]  # TODO: this needs to update a block of x, rather than a scalar.

    def dual_callback(self, msg, vehicle_num):
        self.mu[vehicle_num] = msg.data
        self.update_dual_flag = True
        self.weapon_list = range(0, self.num_weapons)
        self.weapon_list.pop(self.my_number)

    def optimization(self):

        convdiff = [1]
        k = 0 # When we get a primal agent and the optimization stops, it restarts the counter for the number of iteration
        while convdiff > 10 ** -8 and not self.stop_optimization and not rospy.is_shutdown() and k < self.k_max:
            if self.update_dual_flag:
                self.update_duals()
                self.update_dual_flag = False
            
            xUpdated = self.opt.singlePrimal(self.x, self.mu, self.my_number)
            # print xUpdated
            #convdiff.append(la.norm([self.x, xUpdated]))
            convdiff = la.norm([self.x, xUpdated])
            self.x = xUpdated
            k += 1
            print k
            print xUpdated
            time.sleep(1)

        print "done optimizing"
        print self.x
        # if np.random.normal(loc=0, scale=1) > self.publishing_threshold:
        #     pub_msg = Float32MultiArray()
        #     pub_msg.data = self.x[self.my_primal_variable_idx] # publishes just the block
        #     self.primal_pub.publish(pub_msg)
        #
        self.stop_optimization = False
        assignment = 0
        setpoint_msg = self.pose_msg_from_dict(self.target_positions[assignment])
        self.goal_pose_pub.publish(setpoint_msg)

        time.sleep(1)
        self.optimization() # calling this function recursively to make sure we keep getting assignments

        # Decide on the assignments; argmax
        # Once the assignment is done, publish the goal pose to the turtlebot pid node


    def update_duals(self):
        
        muUpdated = self.opt.singleDual(self.x,self.mu[self.my_number],self.my_number)
        self.mu[self.my_number] = muUpdated
        pub_msg = Float32MultiArray()
        pub_msg.data = muUpdated
        if np.random.normal(loc=0, scale=1) > self.publishing_threshold/3:
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
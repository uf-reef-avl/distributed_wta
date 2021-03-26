#!/usr/bin/env python2.7
"""ROS Wrapper for the Distributed WTA
This node is a ROS Wrapper for the distributed optimization. The code will run a ROS node and is intended to run on an
agent eg. tbot. This node will subscribe to primal and dual variable from other agents and perform optimizations
accordingly.
"""
import rospy
import numpy as np
import scipy.linalg as la
from std_msgs.msg import Float32MultiArray, Float64, Int64
from geometry_msgs.msg import PoseStamped, Point
import inputs
import communicate
import time
from visualization import WTAVisualizer
from inputs import WTAInputs
from nav_msgs.msg import Odometry


class WTAOptimization():
    def __init__(self):

        self.num_weapons = rospy.get_param("~num_weapons", 5)  # number of weapons/agent. Obtained from ROS Param
        self.num_targets = rospy.get_param("~num_targets", 4)  # number of weapons/agent. Obtained from ROS Param
        self.k_max = rospy.get_param("~max_iter", 100)  # number of weapons/agent. Obtained from ROS Param
        self.primal_pub_probability = rospy.get_param("~primal_pub_prob",
                                                      0.2)  # number of weapons/agent. Obtained from ROS Param
        self.dual_pub_probability = rospy.get_param("~dual_pub_prob",
                                                    0.2)  # number of weapons/agent. Obtained from ROS Param
        self.target_positions = rospy.get_param("/target_position")  # number of weapons/agent. Obtained from ROS Param
        self.attrition_threshold = rospy.get_param("~attrition_threshold", 7)
        self.attrition_check_threshold = rospy.get_param("~attrition_check_threshold", 2)
        self.is_simulated = rospy.get_param("~is_simulated", False)  # Tells if a turtlebot is simulated or real
        self.publishing_plotting = rospy.get_param("~pub_plotted", True)  # Publishes primal and dual agents for plotting
        self.delay_upper_bound = rospy.get_param("~delay_upper_bound", 2)  # Publishes primal and dual agents for plotting
        self.delay_lower_bound = rospy.get_param("~delay_lower_bound", 0.5)  # Publishes primal and dual agents for plotting

        Pk = np.array(rospy.get_param("/Pk"))  # number of weapons/agent. Obtained from ROS Param
        assert Pk.shape[0] == self.num_weapons
        assert Pk.shape[1] == self.num_targets

        # V = np.random.uniform(low=5, high=10, size=(1, self.num_targets))
        V = np.array(rospy.get_param("/V"))
        assert V.shape[0] == self.num_targets
        # For attritiion, I am going to increase the number targets.
        self.num_targets += 1
        Pk_att_col = np.zeros((self.num_weapons, 1))
        Pk = np.append(Pk_att_col, Pk, 1)
        V = np.append(0, V)
        self.inputs = WTAInputs(self.num_weapons, self.num_targets, Pk, V)

        self.visualization = WTAVisualizer(self.target_positions)

        self.running_while_loop = False
        self.convdiff = 1
        ## Assigning agent numbers
        # In order for the agent to assign weapon/agent id by themselves, we will rely on the ROS namespace which
        # will be declared in the launch file. The namespace can be used to identify the agent and also establish the
        # topics to publish and subscribe to.

        self.my_namespace = rospy.get_namespace()  # number of weapons/agent. Obtained from ROS Param
        # self.my_namespace = "robot_1" # adding for  debugging # number of weapons/agent. Obtained from ROS Param
        self.my_number = int(self.my_namespace.replace("/", "").split("_")[-1])  # number of weapons/agent. Obtained from ROS Param

        ## This line will generate a list of all the primal agents we have. Originally intended to have 2 primal and
        # 1 dual agents for each target. Weapons list if similar but the agent on which this node will run on will be
        # removed
        self.my_primal_variable_idx = range(self.my_number * self.num_targets, (self.my_number + 1) * self.num_targets)
        self.my_dual_variable_idx = self.my_number
        self.weapon_list = range(0, self.num_weapons)
        self.weapon_list.pop(self.my_number)
        self.attrition_list = []
        self.attrition_dict = dict.fromkeys(self.weapon_list, rospy.get_time()) # create a dictionary of agents and times for the last time they heard from agents
        self.last_checked = rospy.get_time()

        self.last_primal_comm = rospy.get_time()
        self.last_dual_comm = rospy.get_time()
        self.primal_comm_delay = np.random.uniform(low=self.delay_lower_bound, high=self.delay_upper_bound)
        self.dual_comm_delay = np.random.uniform(low=self.delay_lower_bound, high=self.delay_upper_bound)

        self.agent_position = Point()
        if self.is_simulated:
            rospy.Subscriber("odom", Odometry, self.odom_callback)
        else:
            rospy.Subscriber("nwu/pose_stamped", PoseStamped, self.mocapPoseCallback)

        for i in [x for x in xrange(self.num_weapons) if x != self.my_number]:
            primal_topic = "/primal_" + str(i)
            rospy.Subscriber(primal_topic, Float32MultiArray, self.primal_callback, i, queue_size=1)
            dual_topic = "/dual_" + str(i)
            rospy.Subscriber(dual_topic, Float64, self.dual_callback, i, queue_size=1)
            # Kat - agents only need to share their primal block values and dual scalar value. 
        pub_topic = "/primal_" + str(self.my_number)
        self.primal_pub = rospy.Publisher(pub_topic, Float32MultiArray, queue_size=10)
        pub_topic = "/dual_" + str(self.my_number)
        self.dual_pub = rospy.Publisher(pub_topic, Float64, queue_size=10)

        if self.publishing_plotting:
            self.convdiff_pub = rospy.Publisher("convdiff", Float64, queue_size=10)
            self.self_convdiff_pub = rospy.Publisher("self_convdiff", Float64, queue_size=10)
            self.primal_plotting = rospy.Publisher("primal_plotting", Float32MultiArray, queue_size=10)
            self.dual_plotting = rospy.Publisher("dual_plotting", Float64, queue_size=10)
            self.assignment_pub = rospy.Publisher("assignment", Int64, queue_size=10)

        self.goal_pose_pub = rospy.Publisher("goal_pose", PoseStamped, queue_size=10)

        self.delta = rospy.get_param("delta", 0.01)  # dual regularization parameter
        self.rho = rospy.get_param("rho", self.delta / (self.delta ** 2 + 2))  # dual step-size
        self.gamma = rospy.get_param("gamma", 1)  # primal step-size

        self.n = self.num_weapons * self.num_targets  # size of primal variable
        self.m = self.num_weapons  # size of dual variable
        pBlocks = self.num_targets * np.arange(self.num_weapons)

        # initialize variables
        self.mu = np.zeros(self.m)  # shared in primal and dual updates
        self.x = .5 * np.ones(self.n)  # shared in primal and dual updates
        self.stop_optimization = False
        self.update_dual_flag = False
        self.delay = 1

        # initialize algorithm
        from DACOA.algorithm import DACOA
        self.opt = DACOA(self.delta, self.gamma, self.rho, self.n, self.m, self.inputs)  # TODO: add documentation
        self.opt.defBlocks(pBlocks, np.arange(self.m))  # TODO: add documentation
        self.opt.useScalars()  # TODO: add documentation
        self.a = self.opt.xBlocks[self.my_number]  # lower boundary of primal block (included)
        self.b = self.opt.xBlocks[self.my_number + 1]  # upper boundary of primal block (not included)

    def primal_callback(self, msg, vehicle_num):
        if vehicle_num in self.weapon_list:
            self.weapon_list.pop(self.weapon_list.index(vehicle_num))

        self.attrition_dict[vehicle_num] = rospy.get_time()
        # print str(vehicle_num) + "      " +str(self.attrition_dict[vehicle_num])
        if not self.weapon_list:
            self.delay = 0.05
            self.update_dual_flag = True
            self.weapon_list = range(0, self.num_weapons)
            self.weapon_list.pop(self.my_number)
            if self.attrition_list:
                self.weapon_list = [x for x in self.weapon_list if x not in self.attrition_list]

        self.stop_optimization = True

        # while self.running_while_loop:
        #     time.sleep(0.001)
            # print "Robot " + str(self.my_number) + " waiting for the while loop"

        idx = range(vehicle_num * self.num_targets, (vehicle_num + 1) * self.num_targets)
        self.x[idx] = msg.data
        self.stop_optimization = False

    def dual_callback(self, msg, vehicle_num):
        self.attrition_dict[vehicle_num] = rospy.get_time() #last heard from vehicle
        self.mu[vehicle_num] = msg.data
        self.weapon_list = range(0, self.num_weapons)
        self.weapon_list.pop(self.my_number)
        if self.attrition_list:
            self.weapon_list = [x for x in self.weapon_list if x not in self.attrition_list]

    def optimization(self):
        self.convdiff = 1
        k = 0
        while self.convdiff > 10 ** -8 and not self.stop_optimization and not rospy.is_shutdown() and k < self.k_max:
            self.running_while_loop = True
            if self.update_dual_flag:
                self.update_duals()
                self.update_dual_flag = False

            xUpdated = self.opt.singlePrimal(self.x, self.mu, self.my_number, self.inputs)
            self.convdiff = la.norm(self.x - xUpdated)

            self_convdiff_msg = Float64()
            self_convdiff_msg.data = la.norm(self.x[self.my_primal_variable_idx] - xUpdated[self.my_primal_variable_idx])

            self.x = np.copy(xUpdated)
            k += 1
            time.sleep(self.delay)

            primal_msg = Float32MultiArray()
            primal_msg.data = self.x[self.my_primal_variable_idx]
            # pub_prob = np.random.uniform(low=0, high=1)
            # if pub_prob <= self.primal_pub_probability:
            if rospy.get_time() - self.last_primal_comm > self.primal_comm_delay:
                # print( "Robot " + str(self.my_number) + " primal prob " + str(self.primal_comm_delay))
                # print( "Robot " + str(self.my_number) + " prob " + str(pub_prob))
                self.primal_pub.publish(primal_msg)
                for i in [x for x in xrange(self.num_weapons) if x != self.my_number]:
                    self.visualization.visualize_communication(self.my_number, i, self.agent_position, dual=False, brighten=True)

                self.last_primal_comm = rospy.get_time()
                self.primal_comm_delay = np.random.uniform(low=self.delay_lower_bound, high=self.delay_upper_bound)

            assignment = np.unravel_index(np.argmax(self.x[self.my_primal_variable_idx]), self.x.shape)  # get the index
            # print "Robot " + str(self.my_number) + " is going to destroy target " + str(assignment[0])
            # print "Robot " + str(self.my_number) + " primal " + str(self.x[self.my_primal_variable_idx] )
            # print "Robot " + str(self.my_number) + " convergence " + str(self.convdiff) + " number of steps " + str(k)
            setpoint_msg = self.pose_msg_from_dict(self.target_positions[assignment[0] - 1])
            self.goal_pose_pub.publish(setpoint_msg)

            if self.publishing_plotting:
                self.primal_plotting.publish(primal_msg)
                convdiff_msg = Float64()
                convdiff_msg.data = self.convdiff
                self.convdiff_pub.publish(convdiff_msg)
                assignment_msg = Int64()
                assignment_msg.data = assignment[0]
                self.assignment_pub.publish(assignment_msg)
                self.self_convdiff_pub.publish(self_convdiff_msg)

            if rospy.get_time() - self.last_checked > self.attrition_check_threshold:
                self.check_attrition()
                self.last_checked = rospy.get_time()

            self.running_while_loop = False

    def update_duals(self):
        muUpdated = self.opt.singleDual(self.x, self.mu[self.my_number], self.my_number, self.inputs)
        self.mu[self.my_number] = np.copy(muUpdated)
        pub_msg = Float64()
        pub_msg.data = self.mu[self.my_number]

        if self.publishing_plotting:
            self.dual_plotting.publish(pub_msg)

        # pub_prob = np.random.uniform(low=0, high=1)
        # if pub_prob <= self.dual_pub_probability:
        if rospy.get_time() - self.last_dual_comm > self.dual_comm_delay:
            self.dual_pub.publish(pub_msg)
            for i in [x for x in xrange(self.num_weapons) if x != self.my_number]:
                self.visualization.visualize_communication(self.my_number, i, self.agent_position, dual=True, brighten=True)

            self.last_dual_comm = rospy.get_time()
            self.dual_comm_delay = np.random.uniform(low=self.delay_lower_bound, high=self.delay_upper_bound)


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

    def odom_callback(self, msg):
        self.agent_position = msg.pose.pose.position
        self.agent_position.z = 0
        ns = self.my_namespace + "position"
        self.visualization.visualize_robot(self.agent_position, ns, self.is_simulated)
        self.visualization.visualize_target()
        for i in [x for x in xrange(self.num_weapons) if x != self.my_number]:
            self.visualization.visualize_communication(self.my_number, i, self.agent_position, dual=False, brighten=False)
            self.visualization.visualize_communication(self.my_number, i, self.agent_position, dual=True, brighten=False)

    def mocapPoseCallback(self, msg):
        self.agent_position = msg.pose.position
        self.agent_position.z = 0
        ns = self.my_namespace + "position"
        self.visualization.visualize_robot(self.agent_position, ns, self.is_simulated)
        self.visualization.visualize_target()
        for i in [x for x in xrange(self.num_weapons) if x != self.my_number]:
            self.visualization.visualize_communication(self.my_number, i, self.agent_position,dual=False, brighten=False)

    def check_attrition(self):
        for agent in self.attrition_dict:
            agent_time = self.attrition_dict[agent]
            if rospy.get_time() - agent_time > self.attrition_threshold:
                print "Robot " + str(self.my_number) + ": Oh no! They got to Agent " + str(agent) + " I havent heard in " + str(round(rospy.get_time() - agent_time, 2)) + " seconds"
                idx = range(agent * self.num_targets, (agent + 1) * self.num_targets)
                self.x[idx] = np.zeros(self.num_targets)
                self.mu[agent] = 0
                if agent not in self.attrition_list:
                    self.attrition_list.append(agent)
                    self.weapon_list.pop(self.weapon_list.index(agent))


if __name__ == '__main__':
    rospy.init_node('Distributed_WTA', anonymous=False)
    wtaDemo = WTAOptimization()
    loop_rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        wtaDemo.optimization()
        loop_rate.sleep()

#!/usr/bin/env python
import rospy
import numpy as np
import scipy.linalg as la
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
import inputs
import communicate

class WTAOptimization():
    def __init__(self):

        self.num_weapons = rospy.get_param("num_weapons", 3)
        self.num_targets = rospy.get_param("num_targets", 2)
        my_namespace = rospy.get_namespace()
        self.my_number = int(my_namespace[-2])
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
        pub_topic = "/primal_" + str(self.my_number)
        self.primal_pub = rospy.Publisher(pub_topic, Float32MultiArray, queue_size=10)
        pub_topic = "/dual_" + str(self.my_number)
        self.dual_pub = rospy.Publisher(pub_topic, Float32MultiArray, queue_size=10)

        self.delta = rospy.get_param("delta", 0.0001) # dual regularization parameter
        self.rho = rospy.get_param("delta", 2.1*(6-np.sqrt(3))/(6*self.delta)) # primal step-size
        self.gamma = rospy.get_param("delta", (1/2)*(1/120018)) # dual step-size
        self.beta_adj = rospy.get_param("beta_adjust", 1)        # used for analysis in this particular problem; set to 1.

        # TODO: @Kat, @Kyle or @Prashant, make sure this is all valid
        self.Np = self.num_weapons * self.num_targets
        self.Nd = self.num_weapons
        self.mu = np.zeros(self.Nd)

        self.x = 10*np.ones((self.Np))
        self.Xd = 10*np.ones(self.Nd)
        self.stop_optimization = False
        # self.optimization()

    def primal_callback(self, msg, vehicle_num):

        if vehicle_num in self.weapon_list:
            self.weapon_list.pop(self.weapon_list.index(vehicle_num))
        elif not self.weapon_list:
            self.update_duals()
            self.weapon_list = range(0, self.num_weapons)
            self.weapon_list.pop(self.my_number)

        self.stop_optimization = True
        self.x[vehicle_num] = msg.data

    def dual_callback(self, msg, vehicle_num):
        print("In dual callback", vehicle_num)

        # TODO: Update the dual variable

    def optimization(self):

        dcount = np.zeros((self.Np, self.Nd))
        t = np.zeros(self.Nd)
        convdiff = [1]
        k = 0
        while convdiff[k] > 10 ** -8 and not self.stop_optimization and not rospy.is_shutdown():

            print("in while loop")
            pGradient = inputs.gradPrimal(self.x, self.mu, self.my_number, self.Np, self.beta_adj)
            pUpdate = self.x[self.my_number] - self.gamma * pGradient
            self.x[self.my_number] = inputs.projPrimal(pUpdate)
            k +=1

        if convdiff[k] > 10 ** -8:
            pub_msg = Float32MultiArray()
            pub_msg.data = self.x
            self.primal_pub.publsh(pub_msg)

    def update_duals(self):

        dGradient = inputs.gradDual(self.Xd[self.my_number],self.mu[self.my_number], self.my_number, self.delta)
        dUpdate = self.mu[self.my_number] + self.rho * dGradient
        self.mu[self.my_number] = dUpdate




if __name__ == '__main__':
    rospy.init_node('test_node', anonymous=True)
    obj = WTAOptimization()
    rospy.spin()
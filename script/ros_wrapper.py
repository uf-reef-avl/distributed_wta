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
            # Kat - agents only need to share their primal block values and dual scalar value. 
        pub_topic = "/primal_" + str(self.my_number)
        self.primal_pub = rospy.Publisher(pub_topic, Float32MultiArray, queue_size=10)
        pub_topic = "/dual_" + str(self.my_number)
        self.dual_pub = rospy.Publisher(pub_topic, Float32MultiArray, queue_size=10)

        self.delta = rospy.get_param("delta", 0.01) # dual regularization parameter
        self.rho = rospy.get_param("rho", self.delta/(self.delta ** 2 + 2)) # dual step-size
        self.gamma = rospy.get_param("gamma", 1) # primal step-size

        # TODO: @Kat, @Kyle or @Prashant, make sure this is all valid
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
        self.opt = DACOA(self.delta,self.gamma,self.rho, self.n, self.m)
        self.opt.defBlocks(pBlocks,np.arange(self.m))
        self.opt.useScalars()
        # self.optimization()
        
        self.a=self.opt.xBlocks[self.my_number]  #lower boundary of primal block (included)
        self.b=self.opt.xBlocks[self.my_number+1] #upper boundary of primal block (not included)

    def primal_callback(self, msg, vehicle_num):

        if vehicle_num in self.weapon_list:
            self.weapon_list.pop(self.weapon_list.index(vehicle_num))
        elif not self.weapon_list:
            self.update_dual_flag = True    #TODO: is this only set to true after an update is received from all agents?
            self.weapon_list = range(0, self.num_weapons) +
            self.weapon_list.pop(self.my_number)

        self.stop_optimization = True
        self.x[vehicle_num] = msg.data  # TODO: this needs to update a block of x, rather than a scalar.

    def dual_callback(self, msg, vehicle_num):
        print("In dual callback", vehicle_num)
        self.weapon_list = range(0, self.num_weapons)
        self.weapon_list.pop(self.my_number)
        self.mu[vehicle_num] = msg.data
        # TODO: Update the dual variable
        # TODO: If another dual variable is received, the update_dual_flag and counter (weapon_list?) need to be reset.

    def optimization(self):

        convdiff = [1]
        k = 0
    
        
        while convdiff[k] > 10 ** -8 and not self.stop_optimization and not rospy.is_shutdown():
            if self.update_dual_flag:
                self.update_duals()
                self.update_dual_flag = False
            
            xUpdated = self.opt.singlePrimal(self.x, self.mu, self.my_number)
            convdiff.append(la.norm(self.x,xUpdated))
            self.x = xUpdated
            k +=1
            # TODO for Prashant: How are we going to restart the optimization? Set a max k to get out of the loop


        if convdiff[k] < 10 ** -8: # Publish the primal agents a lot more frequently... Do it using a probabilty?
            pub_msg = Float32MultiArray()
            pub_msg.data = self.x[a:b] # publishes just the block
            self.primal_pub.publsh(pub_msg)

            # Decide on how to get the assignments

    def update_duals(self):
        
        muUpdated = self.opt.singleDual(self.x,self.mu[self.my_number],self.my_number)
        self.mu[self.my_number] = muUpdated

        #Publish the Dual agents here




if __name__ == '__main__':
    rospy.init_node('test_node', anonymous=True)
    obj = WTAOptimization()
    rospy.spin()
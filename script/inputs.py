#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 13 14:40:26 2020

@author: kat.hendrickson
"""

import numpy as np

class WTAInputs():
    def __init__(self):

        self.A = np.array([[1, 1, 0, 0, 0, 0],
              [0, 0, 1, 1, 0, 0],
              [0, 0, 0, 0, 1, 1],])
        self.b = np.array([1, 1, 1])

        self.Pk = np.array([[0.25, 0.75],
                       [0.6, 0.5],
                       [0.75, 0.25]])
        self.Q = 1. - self.Pk
        self.L = np.log(self.Q)
        self.V = np.array([1., 1.])  # signifies the importance of the targets

    def gradPrimal(self, optInputs,x, mu, agent):
        Nd = optInputs.m
        N_targets = int(optInputs.n / optInputs.m)

        weapon = int(agent / N_targets)
        target = agent % N_targets
        sumterm = 0.
        for k in range(Nd):
            sumterm += self.L[k, target] * x[k * N_targets + target]

        gradient = self.V[target] * self.L[weapon, target] * np.exp(sumterm) + np.inner(mu, self.A[:, agent])
        return gradient

    def gradDual(self, optInputs, x, mu, agent):
        gradient = np.inner(self.A[agent, :], x) - self.b[agent] - optInputs.delta * mu
        return gradient

    def projPrimal(self, x):
        if x > 1.:
            x = 1.
        if x < 0.:
            x = 0.
        x_hat = np.copy(x)
        return x_hat

    def projDual(self, mu):
        if mu < 0:
            mu = 0
        mu_hat = mu
        return mu_hat


# 3 weapons, 2 targets
# Np = 6
# Nd = 3
# Problem Parameters




# # num_weapons = rospy.get_param("~num_weapons", 3) # number of weapons/agent. Obtained from ROS Param
# # num_targets = rospy.get_param("~num_targets", 2) # number of weapons/agent. Obtained from ROS Param
# num_weapons = 3
# num_targets = 2

# A = np.empty([num_weapons, num_targets * num_weapons], dtype=float)
#
# for i in range(0,num_weapons):
#     col_vec = np.zeros((1, num_weapons))
#     col_vec[0,i] = 1
#     block = np.repeat(col_vec, num_targets)
#     A[i,] = block
#
# b= np.array([1, 1, 1])



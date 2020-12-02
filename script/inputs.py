#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 13 14:40:26 2020

@author: kat.hendrickson
"""

import numpy as np
# 5 weapons, 2 targets
# Np = 10
# Nd = 5
# Problem Parameters


A = np.array([[1, 1, 0, 0, 0, 0, 0, 0, 0, 0], 
              [0, 0, 1, 1, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, 1, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 1, 1, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 1, 1],])
b= np.array([1, 1, 1, 1, 1])
Pk = np.array([ [0.5, 0.5],
				[0.5, 0.5],
				[0.5, 0.5],
				[0.5, 0.5],
				[0.5, 0.5]])
Q = 1. - Pk
L = np.log(Q)
V = np.array([2., 1.])

def gradPrimal(x,mu,agent,Np,beta_adj):
    Nd = len(mu)
    N_targets = int(Np/Nd)
    #xi=x[agent]
    sumterm=0
    #for j in range(Np):
    #    if j != (agent) :
    #        sumterm=sumterm + 4*(xi - x[j])
    weapon = int(agent/N_targets)
    target = agent % N_targets
    sumterm = 0.
    for k in range(Nd):
        sumterm += L[k, target]*x[k*N_targets+target]

    gradient= V[target]*L[weapon, target]*np.exp(sumterm) + mu @ A[:,agent] # This wont run on 2.7
    return gradient

def gradDual(x,mu,agent,delta):
    gradient = A[agent,:] @ x - b[agent] - delta*mu
    return gradient

def projPrimal(x):
    if x > 1.:
        x = 1.
    if x < 0.:
        x = 0.
    x_hat = x
    return x_hat

def projDual(mu):
    mu[mu<0]=0
    mu_hat=mu
    return mu_hat

        
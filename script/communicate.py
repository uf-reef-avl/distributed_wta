#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov  2 20:09:38 2020

@author: kat.hendrickson
"""

import numpy as np

def comm(Xp, Xd):
    comm_rate = 1 
    Np = np.size(Xp, axis=1)
    Nd = np.size(Xd, axis=1)
    
    B=np.random.rand(Np, Np+Nd)
    X_new = np.concatenate((Xp,Xd),axis=1)
    dup = np.zeros((Np,Nd))
    
    for i in range(Np):
        for j in range(Np+Nd):
            if i != j:
                if B[i,j] <= comm_rate:
                    B[i,j] = 1
                    X_new[i,j] = Xp[i,i]
                    if j >= Np:
                        dup[i,j-Np] = 1
    
    Xp_new = X_new[:,0:Np]
    Xd_new = X_new[:,Np:Np+Nd]
    
    return Xp_new, Xd_new, dup

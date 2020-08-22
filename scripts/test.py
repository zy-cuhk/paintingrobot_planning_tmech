#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys, os
from math import *
import numpy as np
from robotic_functions.aubo_kinematics import *
import numpy.matlib

def manipulator_T_computation(paint_T, paintinggun_T):

    paintinggun_T_rot=paintinggun_T[0:3,0:3]
    paintinggun_T_tran=paintinggun_T[0:3,3]

    inv_paintinggun_T_rot=paintinggun_T_rot.T
    inv_paintinggun_T_tran=-np.dot(inv_paintinggun_T_rot, paintinggun_T_tran)

    manipulator_T = np.matlib.identity(4, dtype=float)
    manipulator_T[0:3, 0:3] = inv_paintinggun_T_rot
    for i in range(3):
        manipulator_T[i, 3] = inv_paintinggun_T_tran[i]
    
    manipulator_T1=manipulator_T.tolist()
    manipulator_T_list=[]
    for i in range(len(manipulator_T1)):
        for j in range(len(manipulator_T1[0])):
            manipulator_T_list.append(manipulator_T1[i][j])
            
    return manipulator_T_list


paint_T=np.array([[1.0,0.0,0.0,0],[0.0,1.0,0.0,0.0],[0,0,1.0000,0.0],[0,0,0,1.0000]])
paintinggun_T=np.array([[1.0,0.0,0.0,-0.535],[0.0,cos(pi/4),-sin(pi/4),0.0],[0,sin(pi/4),cos(pi/4),0.2500],[0,0,0,1.0000]])
manipulator_T_list=manipulator_T_computation(paint_T, paintinggun_T)
print(manipulator_T_list)



# xmax approximates to 0.45-0.5
# ymax approximates to 0
# zmax approximates 1.54-1.6
# r_max=sqrt(xmax^2+ymax^2+z_max^2), which approximates to 1.7


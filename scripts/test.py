#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys, os
import scipy.io as io
from math import *
import numpy as np
from robotic_functions.aubo_kinematics import *
import numpy.matlib

from collections import defaultdict

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


mat_path="/home/zy/catkin_ws/src/paintingrobot_related/paintingrobot_underusing/paintingrobot_planning_tmech/matlab/second_scan_data/second_scan_data3.mat"
data = io.loadmat(mat_path)
unconnected_waypaths=data['renovation_cells_waypaths']  
# print("unconnected_waypaths[0,0:6] is:",unconnected_waypaths[0][0][0][0][0][0:6])  
list1=[]
list1.append(unconnected_waypaths[0][0][0][0][0][0:6].tolist())
list2=unconnected_waypaths[0][0][0][0][1][0:6].tolist()
print("list2 is:",list2)
list2.reverse()
print("reverse list2 is:", list2)
list1.append(list2)
print("list1 is:",list1)

# dict1={}
# dict2={}
# for i in range(2):
#     dict1[0][i]=unconnected_waypaths[0][0][0][0][i][0:6]
#     # dict1.update({i+1:unconnected_waypaths[0][0][0][0][i][0:6]})
#     # dict2.update(dict1)
# print(dict1)
d=defaultdict(defaultdict)
for i in range(2):
    d[0][i]=unconnected_waypaths[0][0][0][0][0][0:6].tolist()
    print("d is:",d[0][i])

a=2
b=2
print("a/b is: ",int(a/b))
print("a%b is:",int(a%b))

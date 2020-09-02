#!/usr/bin/env python
# -*- coding: utf-8 -*-
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

import scipy.io
mat_path="/home/zy/catkin_ws/src/paintingrobot_related/paintingrobot_underusing/paintingrobot_planning_tmech/matlab/second_scan_data/second_scan_data3.mat"
data = scipy.io.loadmat(mat_path)  # 读取mat文件
renovation_cells_waypaths=data['renovation_cells_waypaths']
renovation_waypaths_onecell=renovation_cells_waypaths[0][0][0][0]
print(renovation_waypaths_onecell)
scipy.io.savemat('/home/zy/catkin_ws/src/paintingrobot_related/paintingrobot_underusing/paintingrobot_planning_tmech/scripts/data1.mat',{'renovation_waypaths_onecell':renovation_waypaths_onecell})  

# def planningresult_visualization1(waypaths_list)
#     fig = plt.figure()
#     ax1 = plt.axes(projection='3d')

#     z = np.linspace(0,13,100)
#     x = 5*np.sin(z)
#     y = 5*np.cos(z)
#     ax1.plot3D(x,y,z,'gray')   

#     zd = 13*np.random.random(100)
#     xd = 5*np.sin(zd)
#     yd = 5*np.cos(zd)
#     ax1.scatter3D(xd,yd,zd, cmap='Blues')  
#     plt.show()

# import rospy, sys, os
# import scipy.io as io
# from math import *
# import numpy as np
# from robotic_functions.aubo_kinematics import *
# import numpy.matlib
# from collections import defaultdict

# def manipulator_T_computation(paint_T, paintinggun_T):
#     paintinggun_T_rot=paintinggun_T[0:3,0:3]
#     paintinggun_T_tran=paintinggun_T[0:3,3]
#     inv_paintinggun_T_rot=paintinggun_T_rot.T
#     inv_paintinggun_T_tran=-np.dot(inv_paintinggun_T_rot, paintinggun_T_tran)
#     manipulator_T = np.matlib.identity(4, dtype=float)
#     manipulator_T[0:3, 0:3] = inv_paintinggun_T_rot
#     for i in range(3):
#         manipulator_T[i, 3] = inv_paintinggun_T_tran[i]
#     manipulator_T1=manipulator_T.tolist()
#     manipulator_T_list=[]
#     for i in range(len(manipulator_T1)):
#         for j in range(len(manipulator_T1[0])):
#             manipulator_T_list.append(manipulator_T1[i][j])
#     return manipulator_T_list


# paint_T=np.array([[1.0,0.0,0.0,0],[0.0,1.0,0.0,0.0],[0,0,1.0000,0.0],[0,0,0,1.0000]])
# paintinggun_T=np.array([[1.0,0.0,0.0,-0.535],[0.0,cos(pi/4),-sin(pi/4),0.0],[0,sin(pi/4),cos(pi/4),0.2500],[0,0,0,1.0000]])
# manipulator_T_list=manipulator_T_computation(paint_T, paintinggun_T)
# print(manipulator_T_list)


# xmax approximates to 0.45-0.5
# ymax approximates to 0
# zmax approximates 1.54-1.6
# r_max=sqrt(xmax^2+ymax^2+z_max^2), which approximates to 1.7


# mat_path="/home/zy/catkin_ws/src/paintingrobot_related/paintingrobot_underusing/paintingrobot_planning_tmech/matlab/second_scan_data/second_scan_data3.mat"
# data = io.loadmat(mat_path)
# unconnected_waypaths=data['renovation_cells_waypaths']  
# print("unconnected_waypaths[0,0:6] is:",unconnected_waypaths[0][0][0][0][0][0:6])  
# list1=[]
# list1.append(unconnected_waypaths[0][0][0][0][0][0:6].tolist())
# list2=unconnected_waypaths[0][0][0][0][1][0:6].tolist()
# print("list2 is:",list2)
# list2.reverse()
# print("reverse list2 is:", list2)
# list1.append(list2)
# print("list1 is:",list1)

# dict1={}
# dict2={}
# for i in range(2):
#     dict1[0][i]=unconnected_waypaths[0][0][0][0][i][0:6]
#     # dict1.update({i+1:unconnected_waypaths[0][0][0][0][i][0:6]})
#     # dict2.update(dict1)
# print(dict1)

# d=defaultdict(defaultdict)
# for i in range(2):
#     d[0][i]=unconnected_waypaths[0][0][0][0][0][0:6].tolist()
#     print("d is:",d[0][i])
# a=2
# b=2
# print("a/b is: ",int(a/b))
# print("a%b is:",int(a%b))

# dict1=defaultdict(defaultdict)
# dict2={0:2}
# for i in range(len(dict2)):
#     dict1[i]=dict2[i]
# print("the size of dict is: ", dict1)


#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys, os
from math import *
import numpy as np
import numpy.matlib
import scipy.io as io
import time
from robotic_functions.aubo_kinematics import *
from collections import defaultdict


def manipulator_T_computation(paint_T, paintinggun_T):
    paintinggun_T_rot=paintinggun_T[0:3,0:3]
    paintinggun_T_tran=paintinggun_T[0:3,3]

    inv_paintinggun_T_rot=paintinggun_T_rot.T
    inv_paintinggun_T_tran=-np.dot(inv_paintinggun_T_rot, paintinggun_T_tran)

    inv_paintinggun_T = np.matlib.identity(4, dtype=float)
    inv_paintinggun_T[0:3, 0:3] = inv_paintinggun_T_rot
    for i in range(3):
        inv_paintinggun_T[i, 3] = inv_paintinggun_T_tran[i]
    
    manipulator_T=np.dot(paint_T,inv_paintinggun_T)
    # print("manipulator_T is:",manipulator_T)

    manipulator_T1=manipulator_T.tolist()
    manipulator_T_list=[]
    for i in range(len(manipulator_T1)):
        for j in range(len(manipulator_T1[0])):
            manipulator_T_list.append(manipulator_T1[i][j])

    return manipulator_T_list


def scheduled_selectedjoints_dict=manipulator_jointspace_tspsolver(selected_manipulatorbase_position,scheduled_selected_strokes_dict,paintinggun_T):
    "step 1: obtain scheduled waypoints list based on the selected strokes"
    scheduled_selected_waypoints_list=[]
    manipulator_strokes_number=0
    for i in range(len(scheduled_selected_strokes_dict)):
        for j in range(len(scheduled_selected_strokes_dict[i])):
            manipulator_strokes_number+=1
            scheduled_selected_waypoints_list.append(scheduled_selected_strokes_dict[i][j][0:3])
            if j==len(scheduled_selected_strokes_dict[i])-1:
                scheduled_selected_waypoints_list.append(scheduled_selected_strokes_dict[i][j][3:6])    
    "step 2: obtain the joint space solutions for each waypoint"
    for i in range(len(scheduled_selected_waypoints_list)):
        xyz0=scheduled_selected_waypoints_list[i][0:3]-selected_manipulatorbase_position[0:3]
        manipulator_base_orientation=selected_manipulatorbase_position[3:6]
        rot=mat_computation.rpy2r(manipulator_base_orientation).T
        xyz=np.dot(rot, xyz0).T

        rpy1=[0,pi/2,0]
        rpy2=[0,pi/2,pi]
        paint_T1 = mat_computation.Tmat(xyz,rpy1)
        manipulator_T_list1 = manipulator_T_computation(paint_T1, paintinggun_T)
        flag1, q_dict1 = aubo_computation.GetInverseResult_withoutref(manipulator_T_list1)
        paint_T2 = mat_computation.Tmat(xyz,rpy2)
        manipulator_T_list2 = manipulator_T_computation(paint_T2, paintinggun_T)
        flag2, q_dict2 = aubo_computation.GetInverseResult_withoutref(manipulator_T_list2)
    
    scheduled_selectedjoints_dict={}
    return scheduled_selectedjoints_dict




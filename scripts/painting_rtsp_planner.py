#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys, os
from math import *
import numpy as np
import numpy.matlib
import scipy.io as io
import time
# import kdtree
from robotic_functions.waypoints_paths_visualization_functions import *
from robotic_functions.aubo_kinematics import *

# import tf
# import moveit_commander
# from geometry_msgs.msg import PoseStamped, Pose, Point
# from visualization_msgs.msg import Marker, MarkerArray
# from nav_msgs.msg import Path
# from std_msgs.msg import ColorRGBA
# from geometry_msgs.msg import PoseStamped, Pose

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

if __name__ == "__main__":
    # input: renovation_cells_mobilebase_positions and renovation_cells_waypaths
    mat_path="/home/zy/catkin_ws/src/paintingrobot_related/paintingrobot_underusing/paintingrobot_planning_tmech/matlab/second_scan_data/second_scan_data3.mat"
    data = io.loadmat(mat_path)
    renovation_cells_waypaths=data['renovation_cells_waypaths']
    renovation_cells_mobilebase_positions=data['renovation_cells_mobilebase_positions']
    renovation_waypaths_orientation=data['renovation_waypaths_orientation']
    
    ## the matrix of painting endeffector link with respect to manipulator wrist3 link is shown as follows:
    paintinggun_T=np.array([[1.0,0.0,0.0,-0.535],[0.0,1.0,0.0,0.0],[0,0,1.0000,0.2500],[0,0,0,1.0000]])
    mat_computation=pose2mat()
    aubo_computation=Aubo_kinematics()

    # for i in range(len(renovation_cells_waypaths[0])):
    #     for j in range(len(renovation_cells_waypaths[0][i][0])):
    for i in range(1):
        for j in range(1):
            mobilebase_position=renovation_cells_mobilebase_positions[0][i][j][0:6]
            parameterx=0.430725381079
            parametery=-0.00033063639818
            theta_z=renovation_cells_mobilebase_positions[0][i][j][5]
            deltax=parameterx*cos(theta_z)-parametery*sin(theta_z)
            deltay=parameterx*sin(theta_z)+parametery*cos(theta_z)

            ## sampling manipulator base positions 
            manipulatorbase_position=np.zeros(6)
            manipulatorbase_position[0]=renovation_cells_mobilebase_positions[0][i][j][0]+deltax
            manipulatorbase_position[1]=renovation_cells_mobilebase_positions[0][i][j][1]+deltay
            manipulatorbase_position[3]=0
            manipulatorbase_position[4]=0
            manipulatorbase_position[5]=theta_z

            ## obtain renovation waypaths inside one cell
            time1=time.time()   
            rmax=1.7
            renovaiton_waypaths_inworkspace=[]
            flag=np.zeros(len(renovation_cells_waypaths[0][i][0][j]), dtype=int)
            print("the number of waypaths is: ", len(renovation_cells_waypaths[0][i][0][j]))

            for times in range(1):
                manipulatorbase_position[2]=1.3155+0.6*times
                for k in range(len(renovation_cells_waypaths[0][i][0][j])):
                    ## check the coverage state of waypaths 
                    if flag[k]==0:
                        ## obtain cartesian waypaths inside manipulator workspace 
                        delat_vector1=renovation_cells_waypaths[0][i][0][j][k][0:3]-manipulatorbase_position[0:3]
                        distance1=sqrt(delat_vector1[0]**2+delat_vector1[1]**2+delat_vector1[2]**2)
                        delat_vector2=renovation_cells_waypaths[0][i][0][j][k][3:6]-manipulatorbase_position[0:3]
                        distance2=sqrt(delat_vector2[0]**2+delat_vector2[1]**2+delat_vector2[2]**2)
                        if distance1<rmax and distance2<rmax:
                            flag[k]=1
                            ## eliminate waypaths without satisfied joint space solutions
                            for m in range(2):
                                waypath_p=renovation_cells_waypaths[0][i][0][j][k][3*m:3*m+3]
                                waypath_orientation=renovation_waypaths_orientation[0][i][0,0:3]
                                manipulator_base_p=manipulatorbase_position[0:3]
                                manipulator_base_orientation=manipulatorbase_position[3:6]
                                
                                rot=mat_computation.rpy2r(manipulator_base_orientation)
                                rot=rot.T
                                xyz=np.dot(rot, waypath_p-manipulator_base_p).T
                                rpy1=[0,pi/2,0]
                                rpy2=[0,pi/2,pi]

                                paint_T1 = mat_computation.Tmat(xyz,rpy1)
                                manipulator_T_list1 = manipulator_T_computation(paint_T1, paintinggun_T)
                                flag1, q_dict1 = aubo_computation.GetInverseResult_withoutref(manipulator_T_list1)

                                paint_T2 = mat_computation.Tmat(xyz,rpy2)
                                manipulator_T_list2 = manipulator_T_computation(paint_T2, paintinggun_T)
                                flag2, q_dict2 = aubo_computation.GetInverseResult_withoutref(manipulator_T_list2)
                                
                                if flag1==True or flag2==True:
                                    flag_point = 1
                                    # for i in range(len(q_dict1)+len(q_dict2)):
                                    #    q_dict=[]
                                else:
                                    flag_point = 0
                                    q_dict=[]
                                flag[k] = flag[k]&flag_point
                print("the coverage number is:",int(sum(flag)))

            ## using cartesian space tsp solver to schedule these suitable waypaths
            renovation_waypaths_onecell = renovation_cells_waypaths[0][i][0][j]
            print(len(renovation_waypaths_onecell))
            








            ## using joint space tsp solver to obtain suitable joints value of scheduled waypaths 


            ## update the coverage states of waypaths


            ## exit condition: waypaths are all coverage status 


            time2 = time.time()
            print("cost time for simple computation is:",time2-time1)






    print("the time cost is:", time2-time1)














    # the intial framework for our problem  
    # while(1):
        # 1. sample climbing mechanism joints 
        # 2. for each climbing mechanism joints
            # 2.1 obtain the corresponding sampled manipulator base positions based on the kinematic model
            # 2.2 obtain the waypaths inside the workspace of sampled manipulator base positions with octotree
            # 2.3 connect these waypaths with Cartesian-space tsp
            # 2.4 obtain the joint-space tsp solver
        # 3. compute the motion cost combining climbing mechanism and manipulator 
        # 4. compute the painting awards
        # 5. based on step 3 and step 4, the joint list is computed.
        # the painting cost is the motion cost 
        # the painting award is the net painting area 

    # the modified framework for our problem  
    # while(1):
        # 1. sample climbing mechanism joints 
        # 2. for each climbing mechanism joints
            # 2.1 obtain the corresponding sampled manipulator base positions based on the kinematic model
            # 2.2 select the waypaths inside the workspace of sampled manipulator base positions 
            # 2.3 select again to obtain the waypaths on which waypoints has colision-free inverse kinematic solutions [] 
            # 2.3 connect these selected waypaths with Cartesian-space tsp solver
            # 2.4 obtain the joint-space tsp solver
        # 3. compute the motion cost combining climbing mechanism and manipulator 
        # 4. compute the painting awards
        # 5. based on step 3 and step 4, the joint list is computed.
        # the painting cost is the motion cost 
        # the painting award is the net painting area 






















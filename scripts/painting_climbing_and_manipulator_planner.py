#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys, os
from math import *
import numpy as np
import numpy.matlib
import scipy.io as io
import time
from robotic_functions.waypoints_paths_visualization_functions import *
from robotic_functions.aubo_kinematics import *
from manipulator_catersian_path_tspsolver import *
from collections import defaultdict


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

def sample_climbing_joints(renovation_mobilebase_position_onecell):
    "obtain mobile platform position and some kinematic parameters"
    mobilebase_position=renovation_mobilebase_position_onecell
    parameterx=0.430725381079
    parametery=-0.00033063639818
    theta_z=renovation_mobilebase_position_onecell[5]
    deltax=parameterx*cos(theta_z)-parametery*sin(theta_z)
    deltay=parameterx*sin(theta_z)+parametery*cos(theta_z)

    "sampling manipulator base positions"
    candidate_manipulatorbase_num=1
    candidate_manipulatorbase_position=np.zeros((candidate_manipulatorbase_num,6))
    for i in range(candidate_manipulatorbase_num):
        candidate_manipulatorbase_position[i][0]=renovation_mobilebase_position_onecell[0]+deltax
        candidate_manipulatorbase_position[i][1]=renovation_mobilebase_position_onecell[1]+deltay
        candidate_manipulatorbase_position[i][2]=0.0+0.6*i
        candidate_manipulatorbase_position[i][3]=0
        candidate_manipulatorbase_position[i][4]=0
        candidate_manipulatorbase_position[i][5]=theta_z

    return candidate_manipulatorbase_position

def obtain_waypaths_insideclimbingworkspace(candidate_manipulatorbase_position,renovation_waypaths_onecell):
    rmax=1.7
    climbingjoints_coverage_number=np.zeros(len(candidate_manipulatorbase_position))
    cartersianwaypaths_incandidateclimbingjoints=defaultdict(defaultdict)

    for candidate_num in range(len(candidate_manipulatorbase_position)):
        coverage_waypaths_num=0
        for k in range(1):
        # for k in range(len(renovation_waypaths_onecell)):
            "obtain cartesian waypaths inside manipulator workspace" 
            delat_vector1=renovation_waypaths_onecell[k][0:3]-candidate_manipulatorbase_position[candidate_num][0:3]
            distance1=sqrt(delat_vector1[0]**2+delat_vector1[1]**2+delat_vector1[2]**2)
            delat_vector2=renovation_waypaths_onecell[k][3:6]-candidate_manipulatorbase_position[candidate_num][0:3]
            distance2=sqrt(delat_vector2[0]**2+delat_vector2[1]**2+delat_vector2[2]**2)
            if distance1<rmax and distance2<rmax:
                "remove waypaths without satisfied joint space solutions"
                flag_point=[0,0]
                # q_dict=defaultdict(defaultdict)
                # waypoints_q_dict=defaultdict(defaultdict)
                for m in range(2):
                    # waypath_p=renovation_waypaths_onecell[k][3*m:3*m+3]
                    # manipulator_base_p=candidate_manipulatorbase_position[0:3]
                    # waypath_orientation=renovation_waypaths_orientation[0][i][0,0:3]

                    xyz0=renovation_waypaths_onecell[k][3*m:3*m+3]-candidate_manipulatorbase_position[candidate_num][0:3]
                    manipulator_base_orientation=candidate_manipulatorbase_position[candidate_num][3:6]
                    rot=mat_computation.rpy2r(manipulator_base_orientation)
                    rot=rot.T
                    xyz=np.dot(rot, xyz0).T
                    print("candidate_manipulatorbase_position[candidate_num] is: ",candidate_manipulatorbase_position[candidate_num])
                    rpy1=[0,pi/2,0]
                    rpy2=[0,pi/2,pi]
                    paint_T1 = mat_computation.Tmat(xyz,rpy1)
                    manipulator_T_list1 = manipulator_T_computation(paint_T1, paintinggun_T)
                    flag1, q_dict1 = aubo_computation.GetInverseResult_withoutref(manipulator_T_list1)
                    paint_T2 = mat_computation.Tmat(xyz,rpy2)
                    manipulator_T_list2 = manipulator_T_computation(paint_T2, paintinggun_T)
                    flag2, q_dict2 = aubo_computation.GetInverseResult_withoutref(manipulator_T_list2)

                    # num=0
                    # for num1 in range(len(q_dict1)):
                    #     q_dict[num]=q_dict1[num1]
                    #     num+=1
                    # for num2 in range(len(q_dict2)):
                    #     q_dict[num]=q_dict2[num2]
                    #     num+=1
                        # waypoints_q_dict=q_dict

                    if flag1==True or flag2==True:
                        flag_point[m] = 1

                if flag_point[0]==1 and flag_point[1]==1:
                    cartersianwaypaths_incandidateclimbingjoints[candidate_num][coverage_waypaths_num]=renovation_waypaths_onecell[k][0:6]  
                    coverage_waypaths_num+=1
        climbingjoints_coverage_number[candidate_num]=coverage_waypaths_num

    return climbingjoints_coverage_number, cartersianwaypaths_incandidateclimbingjoints


def select_climbingjoints(candidate_manipulatorbase_position,climbingjoints_coverage_number, cartersianwaypaths_incandidateclimbingjoints):
    max_coverage_paths_number=0
    for i in range(len(candidate_manipulatorbase_position)):
        if climbingjoints_coverage_number[i]>max_coverage_paths_number:
            max_coverage_paths_number=climbingjoints_coverage_number[i]
            max_coverage_index=i
    selected_manipulatorbase_position=candidate_manipulatorbase_position[max_coverage_index]
    selected_cartersian_waypaths=[]
    for i in range(len(cartersianwaypaths_incandidateclimbingjoints[max_coverage_index])):
        list1=cartersianwaypaths_incandidateclimbingjoints[max_coverage_index][i]
        selected_cartersian_waypaths.append(list1)

    return selected_manipulatorbase_position, selected_cartersian_waypaths




if __name__ == "__main__":
    "input: renovation_cells_mobilebase_positions and renovation_cells_waypaths" 
    mat_path="/home/zy/catkin_ws/src/paintingrobot_related/paintingrobot_underusing/paintingrobot_planning_tmech/matlab/second_scan_data/second_scan_data3.mat"
    data = io.loadmat(mat_path)
    renovation_cells_waypaths=data['renovation_cells_waypaths']
    renovation_cells_mobilebase_positions=data['renovation_cells_mobilebase_positions']
    renovation_waypaths_orientation=data['renovation_waypaths_orientation']
    
    "the matrix of painting endeffector link with respect to manipulator wrist3 link is shown as follows:"
    paintinggun_T=np.array([[1.0,0.0,0.0,-0.535],[0.0,1.0,0.0,0.0],[0,0,1.0000,0.2500],[0,0,0,1.0000]])
    mat_computation=pose2mat()
    aubo_computation=Aubo_kinematics()

    "the algorithm framework is shown as follows:"
    # for i in range(len(renovation_cells_waypaths[0])):
    #     for j in range(len(renovation_cells_waypaths[0][i][0])):
    for i in range(1):
        for j in range(1):
            renovation_mobilebase_position_onecell=renovation_cells_mobilebase_positions[0][i][j][0:6]
            renovation_waypaths_onecell=renovation_cells_waypaths[0][i][0][j]
        
            "step 1: sample candidate climbing joint values and corresponding manipulator base positions"
            candidate_manipulatorbase_position = sample_climbing_joints(renovation_mobilebase_position_onecell)
            # while(1):
            for k in range(1):
                "step 2: obtain renovation waypaths inside the workspace of candidate manipulator base positions"
                climbingjoints_coverage_number, cartersianwaypaths_incandidateclimbingjoints = obtain_waypaths_insideclimbingworkspace(candidate_manipulatorbase_position,renovation_waypaths_onecell)
                
                "step 3: select the best climbing joints value"
                selected_manipulatorbase_position, selected_cartersian_waypaths = select_climbingjoints(candidate_manipulatorbase_position,climbingjoints_coverage_number, cartersianwaypaths_incandidateclimbingjoints)

                "step 4: using cartesian space tsp solver to schedule these suitable waypaths" 
                scheduled_strokes_dict = manipulator_catersian_path_tspsolver(selected_cartersian_waypaths)

                "step 5: using joint space tsp solver to obtain suitable joints value of scheduled waypaths" 

                "step 6: update states for the above variables" 
                # remove the selected manipulator base position from candidate manipulator base positions 
                # remove the covered painting waypaths from painting waypaths list
                # add the selected manipulator base position and covered painting waypaths into the planning list


                "step 7: exit condition: waypaths are all coverage status"
                # uncovered painting waypaths number is zero


                selected_waypoints_list=[]
                manipulator_strokes_number=0
                for i in range(len(scheduled_strokes_dict)):
                    for j in range(len(scheduled_strokes_dict[i])):
                        manipulator_strokes_number+=1
                        selected_waypoints_list.append(scheduled_strokes_dict[i][j][0:3])
                        if j==len(scheduled_strokes_dict[i])-1:
                            selected_waypoints_list.append(scheduled_strokes_dict[i][j][3:6])
                io.savemat('/home/zy/catkin_ws/src/paintingrobot_related/paintingrobot_underusing/paintingrobot_planning_tmech/scripts/data1.mat',{'renovation_waypaths_onecell':renovation_waypaths_onecell,'selected_cartersian_waypaths':selected_cartersian_waypaths,'selected_waypoints_list':selected_waypoints_list})  









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


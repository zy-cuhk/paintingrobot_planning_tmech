#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys, os
from math import *
import numpy as np
import numpy.matlib
import moveit_commander
import scipy.io as io
import tf
import time

from geometry_msgs.msg import PoseStamped, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseStamped, Pose
from robotic_functions.waypoints_paths_visualization_functions import *


if __name__ == "__main__":
    rospy.init_node("paintingrobot_planningresults_visualization", anonymous=True)
    ratet=100
    rate = rospy.Rate(ratet)
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

    mat_path="/home/zy/catkin_ws/src/paintingrobot_related/paintingrobot_underusing/paintingrobot_planning_tmech/matlab/second_scan_data/second_scan_data3.mat"
    data = io.loadmat(mat_path)
    renovation_cells_waypaths=data['renovation_cells_waypaths']
    renovation_cells_waypoints=data['renovation_cells_waypoints']
    renovation_cells_mobilebase_positions=data['renovation_cells_mobilebase_positions']
    
    count=0
    for i in range(len(renovation_cells_waypaths[0])):
        for j in range(len(renovation_cells_waypaths[0][i][0])):
            for k in range(len(renovation_cells_waypaths[0][i][0][j])):
                p1=renovation_cells_waypaths[0][i][0][j][k][0:3]
                p2=renovation_cells_waypaths[0][i][0][j][k][4:6]
                count=count+1
                # print("points on cells_waypaths are: ",p1,p2)
    # print("the waypaths number is: ", count)

    count=0
    for i in range(len(renovation_cells_waypoints[0])):
        for j in range(len(renovation_cells_waypoints[0][i])):
            p=renovation_cells_waypoints[0][i][j][0:3]
            count=count+1
            # print("waypoints inside each cell are: ",p)
    # print("the waypoint number is: ", count)

    count=0
    for i in range(len(renovation_cells_mobilebase_positions[0])):
        for j in range(len(renovation_cells_mobilebase_positions[0][i])):
            p=renovation_cells_mobilebase_positions[0][i][j][0:3]
            count=count+1
            # print("mobile platform positions for cells are: ",p)
    # print("the mobile platform points number are: ", count)
            

    frame='map'
    visualization_num=1
    plane_num=0
    cell_num=0
    waypath_num=0

    plane_num_count1=0
    waypoint_num=0

    plane_num_count2=0
    mobilepoints_num=0
    while not rospy.is_shutdown():
        "visualization of waypaths inside cells"
        painting_waypaths_one_cell = renovation_cells_waypaths[0][plane_num][0][cell_num][waypath_num][0:6]
        painting_waypaths_one_cell = painting_waypaths_one_cell.reshape(2,3)

        marker1,visualization_num=path1_visualization(painting_waypaths_one_cell,frame,visualization_num)
        marker_pub.publish(marker1)
        visualization_num=visualization_num+1

        waypath_num=waypath_num+1
        if waypath_num>=len(renovation_cells_waypaths[0][plane_num][0][cell_num]):
            waypath_num=0
            cell_num=cell_num+1
        if cell_num>=len(renovation_cells_waypaths[0][plane_num][0]):
            cell_num=0
            waypath_num=0
            plane_num=plane_num+1
        if plane_num>=len(renovation_cells_waypaths[0]):
            plane_num=0
            cell_num=0
            waypath_num=0
        print("plane_num and cell number is:",plane_num, cell_num)

        "visualization of waypoints"
        scale1=np.array([0.02,0.02,0.02])
        color1=np.array([1.0,0.0,0.0])
        list1 = renovation_cells_waypoints[0][plane_num_count1][waypoint_num][0:3] 
        painting_waypoints_one_cell=np.array([list1[0],list1[1],list1[2],1,0,0,0])
        # print("painting_waypoints_one_cell is:",painting_waypoints_one_cell)
        marker1,visualization_num=targetpositions_visualization(painting_waypoints_one_cell, frame, visualization_num, scale1, color1)
        # marker_pub.publish(marker1)
        visualization_num=visualization_num+1
        waypoint_num=waypoint_num+1
        if waypoint_num>=len(renovation_cells_waypoints[0][plane_num_count1]):
            waypoint_num=0
            plane_num_count1=plane_num_count1+1
        if plane_num_count1>=len(renovation_cells_waypoints[0]):
            plane_num_count1=0
            waypoint_num=0

        "visaulization of mobile platform positions"
        scale2=np.array([0.5,0.5,0.05])
        color2=np.array([1.0,0.0,0.0])
        # print("plane_num_count2 is:",plane_num_count2)
        # print("mobilepoints_num is:",mobilepoints_num)
        list1 = renovation_cells_mobilebase_positions[0][plane_num_count2][mobilepoints_num][0:3] 
        mobilebase_position_one_cell=np.array([list1[0],list1[1],list1[2],1,0,0,0])
        # print("mobilebase_position_one_cell is:",mobilebase_position_one_cell)
        marker1,visualization_num=targetpositions_visualization(mobilebase_position_one_cell, frame, visualization_num, scale2, color2)
        marker_pub.publish(marker1)
        visualization_num=visualization_num+1
        mobilepoints_num=mobilepoints_num+1
        if mobilepoints_num>=len(renovation_cells_mobilebase_positions[0][plane_num_count2]):
            mobilepoints_num=0
            plane_num_count2=plane_num_count2+1
        if plane_num_count2>=len(renovation_cells_mobilebase_positions[0]):
            plane_num_count2=0
            mobilepoints_num=0
            # visualization_num=0

        if  plane_num_count2>=len(renovation_cells_mobilebase_positions[0]) and plane_num>=len(renovation_cells_waypaths[0]) and plane_num_count1>=len(renovation_cells_waypoints[0]):
            visualization_num=0

        rate.sleep()

        
















    



            
        
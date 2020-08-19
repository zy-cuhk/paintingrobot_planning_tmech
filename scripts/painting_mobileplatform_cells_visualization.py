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
    for i in range(renovation_cells_waypaths[0]):
        for j in range(renovation_cells_waypaths):



    count=0
    for i in range(len(renovation_effective_waypaths[0])):
        for j in range(len(renovation_effective_waypaths[0][i])):
            p1=renovation_effective_waypaths[0][i][j][0:3]
            p2=renovation_effective_waypaths[0][i][j][3:6]
            # print("points on waypaths are: ", p1, p2)
            count=count+1
    print("the point number is: ",count)

    count1=0
    for i in range(len(renovation_effective_waypoints[0])):
        for j in range(len(renovation_effective_waypoints[0][i])):
            waypoint=renovation_effective_waypoints[0][i][j][0:3]
            # print("waypoints are: ",waypoint)
            count1=count1+1
    print("the waypoints number is:", count1)

    frame='map'
    visualization_num=1
    plane_num_count=0
    waypath_num=0
    plane_num_count1=0
    waypoint_num=0
    while not rospy.is_shutdown():
        "visualization of waypaths"
        painting_waypaths_one_cell = renovation_effective_waypaths[0][plane_num_count][waypath_num][0:6]
        painting_waypaths_one_cell = painting_waypaths_one_cell.reshape(2,3)
        # print("painting_waypaths_one_cell is:",painting_waypaths_one_cell)

        marker1,visualization_num=path1_visualization(painting_waypaths_one_cell,frame,visualization_num)
        marker_pub.publish(marker1)
        visualization_num=visualization_num+1

        waypath_num=waypath_num+1
        if waypath_num>=len(renovation_effective_waypaths[0][plane_num_count]):
            waypath_num=0
            plane_num_count=plane_num_count+1
        if plane_num_count>=len(renovation_effective_waypaths[0]):
            plane_num_count=0
            waypath_num=0

        "visualization of waypoints"
        scale1=np.array([0.05,0.05,0.05])
        color1=np.array([1.0,0.0,0.0])
        list1 = renovation_effective_waypoints[0][plane_num_count1][waypoint_num][0:3] 
        painting_waypoints_one_cell=np.array([list1[0],list1[1],list1[2],1,0,0,0])
        print("painting_waypoints_one_cell is:",painting_waypoints_one_cell)
        marker1,visualization_num=targetpositions_visualization(painting_waypoints_one_cell, frame, visualization_num, scale1, color1)
        marker_pub.publish(marker1)
        waypoint_num=waypoint_num+1

        if waypoint_num>=len(renovation_effective_waypoints[0][plane_num_count1]):
            waypoint_num=0
            plane_num_count1=plane_num_count1+1
        if plane_num_count1>=len(renovation_effective_waypoints[0]):
            plane_num_count1=0
            waypoint_num=0

        rate.sleep()

        
















    



            
        
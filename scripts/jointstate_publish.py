#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from math import *
from std_msgs.msg import String,Float64,Bool
import time
import numpy
import os,sys
import socket

from robotic_functions.aubo_kinematics import *
from sensor_msgs.msg import JointState
import yaml
import numpy as np
import numpy.matlib

from scipy.io import loadmat
class aubo_state():
    def __init__(self):
        self.pub_state_ = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.sub_state_ = rospy.Subscriber("/joint_states", JointState, self.sub_state, queue_size=10)
        self.aubo_joints_value=[0.0,0.0,0.0,0.0,0.0,0.0]

    def Init_node(self):
        rospy.init_node("aubo_jointvalue_viewpoints_generation")
    def deg_to_rad(self,tuplelist):
        dd = []
        for i in tuplelist:
            dd.append(i * pi / 180)
        return tuple(dd)
    def rad_to_degree(self,tuplelist):
        dd=[]
        for i in tuplelist:
            dd.append(i*180/math.pi)
        return dd
    def pub_state(self,robot_state):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.header.frame_id = 'map'
        js.name = ["base_joint1", "base_joint2","mobilebase_joint","rodclimbing_joint1",'rodclimbing_joint2',"shoulder_joint","upperArm_joint","foreArm_joint","wrist1_joint","wrist2_joint","wrist3_joint"]
        js.position = [robot_state[0],robot_state[1],robot_state[2],robot_state[3],robot_state[4],robot_state[5],robot_state[6],robot_state[7],robot_state[8],robot_state[9],robot_state[10]]
        js.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        js.effort = [0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub_state_.publish(js)
    def sub_state(self,msg):
        self.aubo_joints_value[0]=msg.position[4]
        self.aubo_joints_value[1]=msg.position[5]
        self.aubo_joints_value[2]=msg.position[6]
        self.aubo_joints_value[3]=msg.position[7]
        self.aubo_joints_value[4]=msg.position[8]
        self.aubo_joints_value[5]=msg.position[9]


def main():
    Aub=aubo_state()
    Aub.Init_node()
    ratet=0.5
    rate = rospy.Rate(ratet)
    temp=[0.0,0.0,0.0,0.0,0.0]
    q_ref1=[6.33,18.66,142.092,120.32,86.375,0.101]
    q_ref2=[0,0,0,0,0,0]
    q_ref_rad1=Aub.deg_to_rad(temp+q_ref1)
    q_ref_rad2=Aub.deg_to_rad(temp+q_ref2)
    
    while not rospy.is_shutdown():
        Aub.pub_state(q_ref_rad1)
        rate.sleep()
        Aub.pub_state(q_ref_rad2)
        rate.sleep()
    

if __name__ == '__main__':
    main()
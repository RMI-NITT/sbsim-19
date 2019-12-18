#!/usr/bin/env python
from __future__ import print_function
from bezier import *
from fitCurves import *
import numpy as np
import matplotlib.pyplot as plt

import sys
import physics as p
import pygame as pg
import pid
import rospy
import math as m
from geometry_msgs.msg import Pose, Twist
from sbsim.msg import goalmsg
import controller as c
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from sbsim.msg import dribble
from sbsim.msg import game
from sbsim.msg import path
import random as rnd
import time
import tf

flag = 0
bpose =  Pose()
d = 0
ball = p.ball(x = 0, y = 0)
robot = p.robot(x = 0,y= 0,ball=ball)
gs = 0
rpath=[]
def rpcallback(msg):
    global flag
    flag = 1
    global rpath
    rpath = msg


def botcallback(msg):
    global flag
    global robot
    global ball
    robot.x = msg.position.x
    robot.y = msg.position.y
    quat = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    robot.yaw = euler[2]

def gettraj(pts):
    beziers = fitCurve(pts,1)
    pathi = path()
    single = game()
    bz = []
    for b in beziers:
        for t in range(0, 51):
            bz.append(bezier.q(b, t/50.0).tolist())
    bz = np.array(bz)
    pli = []
    for i in range(len(bz)):
        single = game()
        single.kx = bz[i][0]
        single.ky = bz[i][1]
        pli.append(single)
    pathi.points = pli
    xdot = np.gradient(bz.T[0])
    ydot = np.gradient(bz.T[1])
    Xdot = [xdot,ydot]
    Xdot = np.array(Xdot)
    Xdot = Xdot.T
    pti = []
    for i in range(len(Xdot)):
        single = game()
        single.kx = Xdot[i][0]
        single.ky = Xdot[i][1]
        pti.append(single)
    return pathi,Xdot,pti

def conv(rpath):
    p=[]
    temp = [0,0]
    pt=rpath
    for i in range(len(pt.points)):
        s=game()
        s=pt.points[i]
        temp = [s.kx,s.ky]
        p.append(temp)
    return p

def run():
    global robot
    global ball
    global d
    global gs
    global flag
    rospy.init_node('trajectory_gen2',anonymous=True)
    traj_pub = rospy.Publisher('robot1n0/traj_vect',game, queue_size = 20)
    traj_publ = rospy.Publisher('robot1n0/traj_vect_list',path, queue_size = 20)
    ppathr1n0 = rospy.Publisher('robot1n0/path',path,queue_size = 20)
    rospy.Subscriber('robot1n0/pose', Pose, botcallback)
    rospy.Subscriber('robot1n0/goalpoints',path,rpcallback)
    r = game()
    i = 0
    flag = 0
    rate = rospy.Rate(30)
    while(True):
        while(flag == 0):
            a = 0
        if a == 0:
            p = conv(rpath)
            p.reverse()
            a = 1
            pts = [robot.x,robot.y]
            p.append(pts)
            p.reverse()
            p=np.array(p)
            if len(p) > 1:
                X,Xdot,pti = gettraj(p)
                ppathr1n0.publish(X)
                traj_publ.publish(pti)
        if flag == 1:
            vx = Xdot[i][0]
            vy = Xdot[i][1]
            r.kx = vx
            r.ky = vy
            r.tag = 0
            traj_pub.publish(r)
            i += 1            
        rate.sleep()



if __name__ == '__main__':
    run()

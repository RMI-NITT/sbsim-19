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
ball1n0 = p.ball(x = 0, y = 0)
robot1n0 = p.robot(x = 0,y= 0,ball=ball1n0)
ball1n1 = p.ball(x = 0, y = 0)
robot1n1 = p.robot(x = 0,y= 0,ball=ball1n1)
ball2n0 = p.ball(x = 0, y = 0)
robot2n0 = p.robot(x = 0,y= 0,ball=ball2n0)
ball2n1 = p.ball(x = 0, y = 0)
robot2n1 = p.robot(x = 0,y= 0,ball=ball2n1)

#traj_pub1n0 = rospy.Publisher('robot1n0/traj_vect',game, queue_size = 20)
traj_publ1n0 = rospy.Publisher('robot1n0/traj_vect_list',path, queue_size = 20)
ppathr1n0 = rospy.Publisher('robot1n0/path',path,queue_size = 20)


#traj_pub1n1 = rospy.Publisher('robot1n1/traj_vect',game, queue_size = 20)
traj_publ1n1 = rospy.Publisher('robot1n1/traj_vect_list',path, queue_size = 20)
ppathr1n1 = rospy.Publisher('robot1n1/path',path,queue_size = 20)

#traj_pub2n0 = rospy.Publisher('robot2n0/traj_vect',game, queue_size = 20)
traj_publ2n0 = rospy.Publisher('robot2n0/traj_vect_list',path, queue_size = 20)
ppathr2n0 = rospy.Publisher('robot2n0/path',path,queue_size = 20)

#traj_pub2n1 = rospy.Publisher('robot2n1/traj_vect',game, queue_size = 20)
traj_publ2n1 = rospy.Publisher('robot2n1/traj_vect_list',path, queue_size = 20)
ppathr2n1 = rospy.Publisher('robot2n1/path',path,queue_size = 20)

gs = 0
rpath1n0=path()
rpath1n1=path()
rpath2n0=path()
rpath2n1=path()
def rp10callback(msg):
    global flag
    global rpath1n0
    global robot1n0
    global ppathr1n0
    global traj_publ1n0
    flag = 1
    global rpath1n0
    rpath1n0 = msg
    get_traj_all(robot1n0,rpath1n0,ppathr1n0,traj_publ1n0)

def bot10callback(msg):
    global flag
    global robot1n0
    global ball1n0
    robot1n0.x = msg.position.x
    robot1n0.y = msg.position.y
    quat = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    robot1n0.yaw = euler[2]
def rp11callback(msg):
    global rpath1n1
    global flag
    global robot1n1
    global ppathr1n1
    global traj_publ1n1
    flag = 1
    global rpath1n1
    rpath1n1 = msg
    get_traj_all(robot1n1,rpath1n1,ppathr1n1,traj_publ1n1)
def bot11callback(msg):
    global flag
    global robot1n1
    global ball1n1
    robot1n1.x = msg.position.x
    robot1n1.y = msg.position.y
    quat = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    robot1n1.yaw = euler[2]
def rp20callback(msg):
    global rpath2n0
    global flag
    global robot2n0
    global ppathr2n0
    global traj_publ2n0
    flag = 1
    global rpath2n0
    rpath2n0 = msg
    get_traj_all(robot2n0,rpath2n0,ppathr2n0,traj_publ2n0)
def bot20callback(msg):
    global flag
    global robot2n0
    global ball2n0
    robot2n0.x = msg.position.x
    robot2n0.y = msg.position.y
    quat = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    robot2n0.yaw = euler[2]
def rp21callback(msg):
    global rpath2n1
    global flag
    global robot2n1
    global ppathr2n1
    global traj_publ2n1
    flag = 1
    global rpath2n1
    rpath2n1 = msg
    get_traj_all(robot2n1,rpath2n1,ppathr2n1,traj_publ2n1)
def bot21callback(msg):
    global flag
    global robot2n1
    global ball2n1
    robot2n1.x = msg.position.x
    robot2n1.y = msg.position.y
    quat = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    robot2n1.yaw = euler[2]

def gettraj(pts):
    beziers = fitCurve(pts,1)
    pathi = path()
    single = game()
    bz = []
    for b in beziers:
        for t in range(0, 51):
            bz.append(bezier.q(b, t/50.0).tolist())
    bz = np.array(bz)
    print(bz)
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

def get_traj_all(robot,rpath,ppathr,traj_publ):
    global flag
    r = game()
    i = 0
    if len(rpath.points) > 0:
        p = conv(rpath)
        p.reverse()
        a = 1
        pts = [robot.x,robot.y]
        p.append(pts)
        p.reverse()
        p=np.array(p)
        print(p)
        if len(p) > 1:
            X,Xdot,pti = gettraj(p)
            ppathr.publish(X)
            traj_publ.publish(pti)

def run():
    rospy.init_node('trajectory_gen2',anonymous=True)
    rospy.Subscriber('robot1n0/pose', Pose, bot10callback)
    rospy.Subscriber('robot1n1/pose', Pose, bot11callback)
    rospy.Subscriber('robot2n0/pose', Pose, bot20callback)
    rospy.Subscriber('robot2n1/pose', Pose, bot21callback)
    rospy.Subscriber('robot1n0/goalpoints',path,rp10callback)
    rospy.Subscriber('robot1n1/goalpoints',path,rp11callback)
    rospy.Subscriber('robot2n0/goalpoints',path,rp20callback)
    rospy.Subscriber('robot2n1/goalpoints',path,rp21callback)
    rospy.spin()

if __name__ == '__main__':
    run()


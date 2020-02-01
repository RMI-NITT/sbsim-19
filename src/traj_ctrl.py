#!/usr/bin/env python
import sys
import physics as p
import pygame as pg
import numpy as np
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
import numpy as np
import matplotlib.pyplot as plt
import time
import tf

"""
program to control robot to make it stay on the trajectory
publishes twist commands
"""

flag = 0
bpose =  Pose()
d = 0
ball = p.ball(x = 0, y = 0)
robot = p.robot(x = 0,y= 0,ball=ball)
gs = 0
r10cs = 1
r11cs = 1
r20cs = 1
r21cs = 1
r10path = path()
r11path = path()
r20path = path()
r21path = path()
r10tlist = path()
r11tlist = path()
r20tlist = path()
r21tlist = path()
rpose = [p.robot(x = 0,y= 0,ball=ball),p.robot(x = 0,y= 0,ball=ball),p.robot(x = 0,y= 0,ball=ball),p.robot(x = 0,y= 0,ball=ball)]
rpath = [[],[],[],[]]
rvects = [[],[],[],[]]

def closestpt_search(path,r):
    min_dist = 10000
    index = -1
    for i in range(len(path)):
        dist = np.sqrt((path[i][0]- r.x)**2 + (path[i][1] - r.y)**2)
        if dist < min_dist:
            min_dist = dist
            index = i
    return path[index],index,min_dist

def r10tcallback(msg):
    global r10cs
    global r10tlist
    global rvects
    if r10cs == 2:
        r10tlist = msg
        rr = []
        for p in r10tlist.points:
            rr.append([p.kx,p.ky])
        rvects[0] = rr 
    return 0

def r11tcallback(msg):
    global r11cs
    global r11tlist
    global rvects
    if r11cs == 2:
        r11tlist = msg
        rr = []
        for p in r11tlist.points:
            rr.append([p.kx,p.ky])
        rvects[1] = rr 
    return 0

def r20tcallback(msg):
    global r20cs
    global r20tlist
    global rvects
    if r20cs == 2:
        r20tlist = msg
        rr = []
        for p in r20tlist.points:
            rr.append([p.kx,p.ky])
        rvects[2] = rr 
    return 0

def r21tcallback(msg):
    global r21cs
    global r21tlist
    global rvects
    if r21cs == 2:
        r21tlist = msg
        rr = []
        for p in r21tlist.points:
            rr.append([p.kx,p.ky])
        rvects[3] = rr 
    return 0

def r1n0pathcbk(msg):
    global r10path
    global rpath
    global rvects
    if r10cs == 2:
        r10path = msg
        rr = []
        for p in r10path.points:
            rr.append([p.kx,p.ky])
        rpath[0] = rr

def r1n1pathcbk(msg):
    global r11path
    global rpath
    if r11cs == 2:
        r11path = msg
        rr = []
        for p in r11path.points:
            rr.append([p.kx,p.ky])
        rpath[1] = rr

def r2n0pathcbk(msg):
    global r20path
    global rpath
    if r20cs == 2:
        r20path = msg
        rr = []
        for p in r20path.points:
            rr.append([p.kx,p.ky])
        rpath[2] = rr

def r2n1pathcbk(msg):
    global r21path
    global rpath
    if r21cs == 2:
        r21path = msg
        rr = []
        for p in r21path.points:
            rr.append([p.kx,p.ky])
        rpath[3] = rr

def r10selcallback(msg):
    global r10cs
    r10cs = msg.data
    return 0

def r11selcallback(msg):
    global r11cs
    r11cs = msg.data
    return 0

def r20selcallback(msg):
    global r20cs
    r20cs = msg.data
    return 0

def r21selcallback(msg):
    global r21cs
    r21cs = msg.data
    return 0

def r10posecallback(msg):
    global rpose
    if r10cs == 2:
        rpose[0].x = msg.position.x
        rpose[0].y = msg.position.y
        (_,_,rpose[0].theta) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

def r11posecallback(msg):
    global rpose
    if r11cs == 2:
        rpose[1].x = msg.position.x
        rpose[1].y = msg.position.y
        (_,_,rpose[1].theta) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

def r20posecallback(msg):
    global rpose
    if r20cs == 2:
        rpose[2].x = msg.position.x
        rpose[2].y = msg.position.y
        (_,_,rpose[2].theta) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

def r21posecallback(msg):
    global rpose
    if r21cs == 2:
        rpose[3].x = msg.position.x
        rpose[3].y = msg.position.y
        (_,_,rpose[3].theta) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

def ballposecallback(msg):
    global bpose
    bpose.position.x = msg.position.x
    bpose.position.y = msg.position.y

def findexc(i,l):
    if i > l-2:
        return 5
    else:
        return 1

def findexv(i,l):
    if i > l-2:
        return 0
    else:
        return 1


def run():
    global rpath
    global rvects
    global r10cs
    global r11cs
    global r20cs
    global r21cs
    global rpose
    rospy.init_node('trajectory_tracking_controller',anonymous=True)
    rospy.Subscriber('robot1n0/cselect',Int32,r10selcallback)
    rospy.Subscriber('/ballpose',Pose,ballposecallback)
    rospy.Subscriber('robot1n1/cselect',Int32,r11selcallback)
    rospy.Subscriber('robot2n0/cselect',Int32,r20selcallback)
    rospy.Subscriber('robot2n1/cselect',Int32,r21selcallback)
    rospy.Subscriber('robot1n0/traj_vect_list',path,r10tcallback)
    rospy.Subscriber('robot1n1/traj_vect_list',path,r11tcallback)
    rospy.Subscriber('robot2n0/traj_vect_list',path,r20tcallback)
    rospy.Subscriber('robot2n1/traj_vect_list',path,r21tcallback)
    rospy.Subscriber('robot1n0/path',path,r1n0pathcbk)
    rospy.Subscriber('robot1n1/path',path,r1n1pathcbk)
    rospy.Subscriber('robot2n0/path',path,r2n0pathcbk)
    rospy.Subscriber('robot2n1/path',path,r2n1pathcbk)
    rospy.Subscriber('robot1n0/pose',Pose,r10posecallback)
    rospy.Subscriber('robot1n1/pose',Pose,r11posecallback)
    rospy.Subscriber('robot2n0/pose',Pose,r20posecallback)
    rospy.Subscriber('robot2n1/pose',Pose,r21posecallback)
    traj_pub_r10 = rospy.Publisher('robot1n0/traj_vect',game, queue_size = 20)
    traj_pub_r11 = rospy.Publisher('robot1n1/traj_vect',game, queue_size = 20)
    traj_pub_r20 = rospy.Publisher('robot2n0/traj_vect',game, queue_size = 20)
    traj_pub_r21 = rospy.Publisher('robot2n1/traj_vect',game, queue_size = 20)
    rate = rospy.Rate(30)
    kpc = 0
    kpv = 1
    kph1 = 0.1
    kdh1 = 0.1
    heading_error_1 = 0
    while(True):
        if r10cs == 2:
            '''
            this section sets the point for the bot to point at(heading control)
            '''
            r10hp = Pose()
            r10hp.position.x = 0
            r10hp.position.y = 0
            r10vel = game()
            r10vel.kx = 0
            r10vel.ky = 0
            #pd Controller for heading
            thtg = m.atan2((r10hp.position.y-rpose[0].y),(r10hp.position.x-rpose[0].x))
            """if(( thetaset_1 >=(m.pi)) and (thetaset_1 < (0.95*m.pi))):
                thetaset_1= abs(thetaset_1)
            diff_1 = (thetaset_1 - rpose[0].theta) - heading_error_1
            heading_error_1 = (thetaset_1 - rpose[0].theta)
            r10vel.thetad = kph1*heading_error_1 + kdh1*diff_1"""
            
            while(thtg<0):
                if thtg<0:
                    thtg = 6.28+thtg
                else:
                    break
            while(thtg>6.28):
                if thtg>6.28:
                    thtg = thtg - 6.28
                else:
                    break
            if (abs(thtg-rpose[0].theta)>0.1 and thtg <= 6.28 and thtg >= 0):
                if(abs(thtg-rpose[0].theta)<(6.28-abs(thtg-rpose[0].theta))):
                    f = (thtg-rpose[0].theta)/abs(thtg-rpose[0].theta)
                    e = abs(thtg-rpose[0].theta)
                    r10vel.thetad = (e*kph1)*(f)
                else:
                    f = (thtg-rpose[0].theta)/abs(thtg-rpose[0].theta)
                    e = 6.28 - abs(thtg-rpose[0].theta)
                    r10vel.thetad = -((e*kph1))*(f)        
            else:
                r10vel.thetad = 0
            if len(rpath[0])> 0 and len(rvects[0]) > 0:
                l = len(rpath[0])
                cptr10,indexr10,mindistr10 = closestpt_search(rpath[0],rpose[0])
                """
                cptr10 closest point on trajectory from robot
                indexr10 index of closest point on the trajectory
                mindistr10 distance of closest point on the trajetory
                vectr10 tangent of trajectory curve
                comp1x component 1 x tangential component
                comp2x component 2 x correctional component
                comp1y component 1 y tangential component
                comp2y component 2 y correctional component
                """
                vectr10 = rvects[0][indexr10]
                comp1x = findexv(indexr10,l)*kpv*vectr10[0]
                comp1y = findexv(indexr10,l)*kpv*vectr10[1]
                if(not (cptr10[0] - rpose[0].x)==0 and not(cptr10[1] - rpose[0].y)==0):
                    comp2x = kpc*findexc(indexr10,l)*(cptr10[0] - rpose[0].x)**3/abs((cptr10[0] - rpose[0].x)*10)
                    comp2y = kpc*findexc(indexr10,l)*(cptr10[1] - rpose[0].y)**3/abs((cptr10[1] - rpose[0].y)*10)
                else:
                    comp2x = kpc*findexc(indexr10,l)*(cptr10[0] - rpose[0].x)
                    comp2y = kpc*findexc(indexr10,l)*(cptr10[1] - rpose[0].y)
            
                r10vel.kx = comp1x + comp2x
                r10vel.ky = comp1y + comp2y
                

            norm = np.sqrt(r10vel.kx**2 + r10vel.ky**2)
            #if not norm == 0:
            #    r10vel.kx /= 0.5*norm
            #    r10vel.ky /= 0.5*norm
            #    r10vel.tag = 0
                #if mindistr10 < 1:
                    #rpath[0].pop(indexr10)
            traj_pub_r10.publish(r10vel)
            

        if r11cs == 2:
            if len(rpath[1])> 0 and len(rvects[1]) > 0:
                r11vel = game()
                l = len(rpath[1])
                cptr11,indexr11,mindistr11 = closestpt_search(rpath[1],rpose[1])
                vectr11 = rvects[1][indexr11]
                comp1x = findexv(indexr11,l)*kpv*vectr11[0]
                comp2x = kpc*findexc(indexr11,l)*(cptr11[0] - rpose[1].x)
                comp1y = findexv(indexr11,l)*kpv*vectr11[1]
                comp2y = kpc*findexc(indexr11,l)*(cptr11[1] - rpose[1].y)
                r11vel.kx = comp1x + comp2x
                r11vel.ky = comp1y + comp2y
                norm = np.sqrt(r11vel.kx**2 + r11vel.ky**2)
                if not norm == 0:
                    r11vel.kx /= 0.5*norm
                    r11vel.ky /= 0.5*norm
                    r11vel.tag = 1
                    #if mindistr11 < 1:
                    #    rpath[0].pop(indexr11)
                    traj_pub_r11.publish(r11vel)
        if r20cs == 2:
            if len(rpath[2])> 0 and len(rvects[2]) > 0:
                r20vel = game()
                l = len(rpath[2])
                cptr20,indexr20,mindistr20 = closestpt_search(rpath[2],rpose[2])
                vectr20 = rvects[2][indexr20]
                comp1x = findexv(indexr20,l)*kpv*vectr20[0]
                comp2x = kpc*findexc(indexr20,l)*(cptr20[0] - rpose[2].x)
                comp1y = findexv(indexr20,l)*kpv*vectr20[1]
                comp2y = kpc*findexc(indexr20,l)*(cptr20[1] - rpose[2].y)
                r20vel.kx = comp1x + comp2x
                r20vel.ky = comp1y + comp2y
                norm = np.sqrt(r20vel.kx**2 + r20vel.ky**2)
                if not norm == 0:
                    r20vel.kx /= 0.5*norm
                    r20vel.ky /= 0.5*norm
                    r20vel.tag = 2
                    #if mindistr20 < 1:
                    #    rpath[2].pop(indexr20)
                    traj_pub_r20.publish(r20vel)
        if r21cs == 2:
            if len(rpath[3])> 0 and len(rvects[3]) > 0:
                r21vel = game()
                l = len(rpath[3])
                cptr21,indexr21,mindistr21 = closestpt_search(rpath[3],rpose[3])
                vectr21 = rvects[3][indexr21]
                comp1x = findexv(indexr21,l)*kpv*vectr21[0]
                comp2x = kpc*findexc(indexr21,l)*(cptr21[0] - rpose[3].x)
                comp1y = findexv(indexr21,l)*kpv*vectr21[1]
                comp2y = kpc*findexc(indexr21,l)*(cptr21[1] - rpose[3].y)
                r21vel.kx = comp1x + comp2x
                r21vel.ky = comp1y + comp2y
                norm = np.sqrt(r21vel.kx**2 + r21vel.ky**2)
                if not norm == 0:
                    r21vel.kx /= 0.5*norm
                    r21vel.ky /= 0.5*norm
                    r21vel.tag = 3
                    #if mindistr21 < 1:
                    #    rpath[3].pop(indexr21)
                    traj_pub_r21.publish(r21vel)
        rate.sleep()
            

if __name__ == '__main__':
    run()

#!/usr/bin/env python
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
import random as rnd
import time
import tf

flag = 1
bpose =  Pose()
btwist = Twist()
d = 0
ball = p.ball(x = 0, y = 0)
robot = p.robot(x = 0,y= 0,ball=ball)
gs = 0

def callback(msg):
    global flag
    flag = 0

def bcallback(msg):
    global bpose
    bpose = msg

def btcallback(msg):
    global btwist
    btwist = msg

def gcallback(msg):
    global gs
    gs = msg

def dcallback(msg):
    global d
    d = msg

def updatebtwist(a,b):
    b.xd = a.linear.x
    b.yd = a.linear.y


def updatebpose(a,b):
    b.x = a.position.x
    b.y = a.position.y

def botcallback(msg):
    global robot
    global ball
    robot.x = msg.position.x
    robot.y = msg.position.y
    quat = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    robot.yaw = euler[2]

def summa():
    global robot
    global ball
    global d
    global gs
    rospy.init_node('autogk',anonymous=True)
    r1 = rospy.Publisher('bot/ptg',goalmsg, queue_size = 10)
    rospy.Subscriber('ballpose', Pose, bcallback)
    rospy.Subscriber('bot1/pose', Pose, botcallback)    
    rospy.Subscriber('balltwist', Twist, btcallback)
    rospy.Subscriber('game/dribbler', Int32, dcallback)
    rospy.Subscriber('game/status', Int32, gcallback)
    rospy.Subscriber('bot4/reached',Int32,callback)
    updatebpose(bpose,ball)
    updatebtwist(btwist,ball)
    r = goalmsg()
    rate = rospy.Rate(10)
    while(True):
        r.posetogo.position.x = 250
        r.posetogo.position.y = 0
        r.posetogo.orientation.z = 0  
        r.posetogo.orientation.w = 1
        r.status = 1
        r1.publish(r)
        rate.sleep()
        while(True):
            updatebpose(bpose,ball)
            updatebtwist(btwist,ball)
            if not(ball.xd == 0):
                if d == 0: 
                    m0 = ball.yd/ball.xd
                else:
                    m0 = (ball.y-robot.y)/(ball.x-robot.x)
                th = m.atan(m0)
                th = 3.14 + th
                q = m.tan(th/2)
                k = 420
                xb= ball.x
                yb= ball.y
                xtg = k
                ytg = m0*(k-xb)+yb
                if abs(ytg)>=400:
                    ytg = 0
                r.posetogo.position.x = xtg
                r.posetogo.position.y = ytg
                r.posetogo.orientation.z = q  
                r.posetogo.orientation.w = 1
                r.status = 1
                r1.publish(r)
                if gs == 1 or gs == 2 or gs == 3:
                    break
            rate.sleep()
            

if __name__ == '__main__':
    summa()
    


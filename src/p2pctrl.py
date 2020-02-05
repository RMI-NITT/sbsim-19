#!/usr/bin/env python
import sys
import physics as p
import rospy
import math as m
from geometry_msgs.msg import Pose, Twist
from sbsim.msg import goalmsg
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from sbsim.msg import dribble
from sbsim.msg import game

"""
if gmsg.status == 0 bot is stationary
if gmsg.status == 1 bot moves to location in gmsg
if gmsg.status == 2 ball kicked at target location


if rxycs == 1 p2p control pass
if rxycs == 2 trajectory command bypass
if rxycs == 3 extremal control
"""

flag=0
ctrl=game()
traj_ctrl = game()
ctrl.kick=0
ball = p.ball(x = 0,y = 0)
bpose = Pose()
btwist = Twist()
rpose = [Pose(),Pose(),Pose(),Pose()]
rtwist =[Twist(),Twist(),Twist(),Twist()]
r10msg = goalmsg()
r11msg = goalmsg()
r20msg = goalmsg()
r21msg = goalmsg() 
r10cs = 1
r11cs = 1
r20cs = 1
r21cs = 1
r1f = 1
r2f = 1
r3f =1
r4f =1
tag=0
gs = 0

r1=[]
r2=[]

pid_ctrl= rospy.Publisher('ctrl',game, queue_size=10)

def dribcallback(msg): 
    global r1
    global r2
    if(msg.data==1):
        r1[0].dribble=1
        r1[1].dribble=0
        r2[0].dribble=0
        r2[1].dribble=0
    elif(msg.data==2):
        r1[0].dribble=0
        r1[1].dribble=1
        r2[0].dribble=0
        r2[1].dribble=0
    elif(msg.data==3):
        r1[0].dribble=0
        r1[1].dribble=0
        r2[0].dribble=1
        r2[1].dribble=0
    elif(msg.data==4):
        r1[0].dribble=0
        r1[1].dribble=0
        r2[0].dribble=0
        r2[1].dribble=1
    else:
        r1[0].dribble=0
        r1[1].dribble=0
        r2[0].dribble=0
        r2[1].dribble=0


def pid(xtg,ytg,bot,thtg):
    prod = 0.10
    kpp = 0.3
    kpv = prod/kpp
    kpr = 0.02
    global tag
    global ctrl
    vrx = (kpp * (xtg - bot.x))
    vry = (kpp * (ytg - bot.y))
    kx = (kpv * (vrx / 10 - bot.xd))
    ky = (kpv * (vry / 10 - bot.yd))
    kx+=vrx/10.0
    ky+=vry/10.0
    speed=m.sqrt(kx**2 + ky**2)
    if(speed!=0):
        c=kx/speed
        s=ky/speed
    if(speed>2.0):
        kx=c*2.0
        ky=s*2.0
        
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
    if (abs(thtg-bot.theta)>0.1 and thtg <= 6.28 and thtg >= 0):
        if(abs(thtg-bot.theta)<(6.28-abs(thtg-bot.theta))):
            f = (thtg-bot.theta)/abs(thtg-bot.theta)
            e = abs(thtg-bot.theta)
            thetad = (e*kpr)*(f)
        else:
            f = (thtg-bot.theta)/abs(thtg-bot.theta)
            e = 6.28 - abs(thtg-bot.theta)
            thetad = -((e*kpr))*(f)        
    else:
        thetad = 0
    ctrl.kx =kx
    ctrl.ky=ky
    ctrl.thetad=thetad
    ctrl.tag=tag
    pid_ctrl.publish(ctrl)

def r10tcallback(msg):
    global r10cs
    global pid_ctrl
    if r10cs == 2:
        pid_ctrl.publish(msg)
    return 0

def r11tcallback(msg):
    global r11cs
    global pid_ctrl
    if r11cs == 2:
        pid_ctrl.publish(msg)
    return 0

def r20tcallback(msg):
    global r20cs
    global pid_ctrl
    if r20cs == 2:
        pid_ctrl.publish(msg)
    return 0

def r21tcallback(msg):
    global r21cs
    global pid_ctrl
    if r21cs == 2:
        pid_ctrl.publish(msg)
    return 0

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

def balltwistcallback(msg):
    global ball
    global btwist
    btwist=msg
    updatebtwist(btwist,ball)

def ballposecallback(msg):
    global ball
    global bpose
    bpose = msg
    updatebpose(bpose,ball)

def updatebtwist(a,b):
    b.xd =a.linear.x
    b.yd = a.linear.y

def updatebpose(a,b):
    b.x = a.position.x
    b.y = a.position.y
    
def r10callback(msg):
    global r10msg
    global r1f
    r10msg = msg
    r1f = 1
    return 0

def r11callback(msg):
    global r11msg
    global r2f
    r11msg = msg
    r2f = 1
    return 0

def r20callback(msg):
    global r3f
    global r20msg
    r20msg = msg
    r3f = 1
    return 0

def r21callback(msg):
    global r4f
    global r21msg
    r21msg  = msg
    r4f = 1
    return 0 

def r10posecallback(msg):
    global rpose
    global r1
    rpose[0]=msg
    updaterpose(rpose[0],r1[0])

def r11posecallback(msg):
    global rpose
    global r1
    rpose[1]=msg
    updaterpose(rpose[1],r1[1])

def r20posecallback(msg):
    global rpose
    global r2
    rpose[2]=msg
    updaterpose(rpose[2],r2[0])

def r21posecallback(msg):
    global rpose
    global r2
    rpose[3]=msg
    updaterpose(rpose[3],r2[1])

def r10twistcallback(msg):
    global rtwist
    global r1
    global tag
    global gs
    global r10cs
    tag=0
    rtwist[0]=msg
    updatertwist(rtwist[0],r1[0])
    if(gs==0 and r10cs == 1):
        control(r10msg,r1[0],ball)

def r11twistcallback(msg):
    global rtwist
    global r1
    global tag
    global gs
    global r11cs
    tag=1
    rtwist[0]=msg
    updatertwist(rtwist[1],r1[1])
    if(gs==0 and r11cs == 1):
        control(r11msg,r1[1],ball)

def r20twistcallback(msg):
    global rtwist
    global r2
    global tag
    global gs
    global r20cs
    tag=2
    rtwist[0]=msg
    updatertwist(rtwist[2],r2[0])
    if(gs==0 and r20cs == 1):
        control(r20msg,r2[0],ball)

def r21twistcallback(msg):
    global rtwist
    global r2
    global tag
    global gs   
    global r21cs
    tag=3
    rtwist[0]=msg
    updatertwist(rtwist[3],r2[1])
    if(gs==0 and r21cs == 1):
        control(r21msg,r2[1],ball)

def updaterpose(a,b):
    b.x = a.position.x
    b.y = a.position.y
    b.theta= 2*m.atan(a.orientation.z)
    
def updatertwist(a,b):
    b.xd = a.linear.x 
    b.yd = a.linear.y
    b.thetad = a.linear.z

def rulecheck(msg):
    global gs
    gs = msg.data
    return 0

def control(gmsg,robot,ball):
    global ctrl
    ctrl.kick=0
    if gmsg.status == 1 or gmsg.status == 2:
        th = 2*m.atan(gmsg.posetogo.orientation.z)
        if robot.dribble==1 and gmsg.status==2 and abs(robot.x - gmsg.posetogo.position.x)<40 and abs(robot.y - gmsg.posetogo.position.y)<40 and abs(robot.theta - 2*m.atan(gmsg.posetogo.orientation.z))< 1:
            ctrl.kick=1
            gmsg.status = 0
        pid(gmsg.posetogo.position.x,gmsg.posetogo.position.y,robot,thtg=th)
        ctrl.kick=0

    if gmsg.status == 0:
        pid(robot.x,robot.y,robot,thtg=robot.theta)
   
def run():
    rospy.Subscriber('game/status',Int32,rulecheck)
    rospy.Subscriber('game/dribbler',Int32,dribcallback)
    rospy.Subscriber('robot1n0/ptg',goalmsg,r10callback) # for native to node p2p control
    rospy.Subscriber('robot1n1/ptg',goalmsg,r11callback) # for native to node p2p control
    rospy.Subscriber('robot2n0/ptg',goalmsg,r20callback) # for native to node p2p control
    rospy.Subscriber('robot2n1/ptg',goalmsg,r21callback) # for native to node p2p control
    rospy.Subscriber('robot1n0/traj_vect',game,r10tcallback) # for the trajectory node bypass
    rospy.Subscriber('robot1n1/traj_vect',game,r11tcallback) # for the trajectory node bypass
    rospy.Subscriber('robot2n0/traj_vect',game,r20tcallback) # for the trajectory node bypass
    rospy.Subscriber('robot2n1/traj_vect',game,r21tcallback) # for the trajectory node bypass
    rospy.Subscriber('ballpose', Pose,ballposecallback)
    rospy.Subscriber('balltwist',Twist,balltwistcallback)
    rospy.Subscriber('robot1n0/pose',Pose,r10posecallback)
    rospy.Subscriber('robot1n1/pose',Pose,r11posecallback)
    rospy.Subscriber('robot2n0/pose',Pose,r20posecallback)
    rospy.Subscriber('robot2n1/pose',Pose,r21posecallback)
    rospy.Subscriber('robot1n0/twist',Twist,r10twistcallback)
    rospy.Subscriber('robot1n1/twist',Twist,r11twistcallback)
    rospy.Subscriber('robot2n0/twist',Twist,r20twistcallback)
    rospy.Subscriber('robot2n1/twist',Twist,r21twistcallback)
    rospy.Subscriber('robot1n0/cselect',Int32,r10selcallback)
    rospy.Subscriber('robot1n1/cselect',Int32,r11selcallback)
    rospy.Subscriber('robot2n0/cselect',Int32,r20selcallback)
    rospy.Subscriber('robot2n1/cselect',Int32,r21selcallback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('p2p', anonymous=True)
    r1.append(p.robot(x= 0.0,y=0.0, yaw  = 0, ball = ball))
    r1.append(p.robot(x= 0.0,y= 0.0, yaw  = 0, ball = ball))
    r2.append(p.robot(x= 0.0,y=0.0, yaw  = 3.14, ball = ball))
    r2.append(p.robot(x= 0.0,y=0.0, yaw  = 3.14, ball = ball))
    run()


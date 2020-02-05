#!/usr/bin/env python
import rospy
import physics_new as p
import math as m
from geometry_msgs.msg import Pose, Twist
from sbsim.msg import goalmsg
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from sbsim.msg import dribble
from sbsim.msg import game

d = 0

r1f = 1
r2f = 1
r3f =1
r4f =1
ball = p.ball(x = 0,y = 0)

r1=[]
r2=[]

gs = 0
 
def dribbletest(r1,r2,r3,r4):
    global d
    if r1.dribble == 1:
        d = 1
    elif r2.dribble == 1:
        d = 2
    elif r3.dribble == 1:
        d = 3
    elif r4.dribble == 1:
        d = 4
    else:
        d =0
    return 0


def robotpubinit(t,n):
    namepose = 'robot'+str(t)+'n'+str(n)+'/pose'
    nametwist = 'robot'+str(t)+'n'+str(n)+'/twist'
    namer = 'robot'+str(t)+'n'+str(n)+'/reached'
    return rospy.Publisher(namepose,Pose, queue_size = 10),rospy.Publisher(namer,Int32,queue_size = 10),rospy.Publisher(nametwist,Twist,queue_size=10)





def ctrlcallback(msg):
    global r1
    global r2
    global ball
    if(msg.tag==0):
        r1[0].movebot(kx=msg.kx,ky=msg.ky,ball=ball,thetad=msg.thetad)
        if(msg.kick==1):
            r1[0].dribble=0
            r1[0].kick(ball,2)
            
    if(msg.tag==1):
        r1[1].movebot(kx=msg.kx,ky=msg.ky,ball=ball,thetad=msg.thetad)
        if(msg.kick==1):
            r1[1].dribble=0
            r1[1].kick(ball,2)
            
    if(msg.tag==2):
        r2[0].movebot(kx=msg.kx,ky=msg.ky,ball=ball,thetad=msg.thetad)
        if(msg.kick==1):
            r2[0].dribble=0
            r2[0].kick(ball,2) 
    if(msg.tag==3):
        r2[1].movebot(kx=msg.kx,ky=msg.ky,ball=ball,thetad=msg.thetad)
        if(msg.kick==1):
            r2[1].dribble=0
            r2[1].kick(ball,2)
            

#callback of subscriber to intelligence

#def freekick

#def throwin

#def goal

def updaterpose(a,b):
    a.position.x = b.x
    a.position.y = b.y
    a.orientation.z = m.tan(b.theta/2)
    a.orientation.w = 1
    if b.distdribbled != 0:
        return b.distdribbled

def updatertwist(a,b):
    a.linear.x = b.xd
    a.linear.y = b.yd
    a.linear.z = b.theta   

def updatebtwist(a,b):
    a.linear.x = b.xd
    a.linear.y = b.yd
    a.linear.z = 0   

def updatebpose(a,b):
    a.position.x = b.x
    a.position.y = b.y
    a.orientation.w = 1

def rulecheck(msg):
    global gs
    gs = msg.data
    return 0

def reset(t1,t2,r1,r2,r3,r4,ball):
    global gs
    r1 = p.robot(x= t1[0][0],y= t1[0][1], yaw  = 0, ball = ball)
    r2 = p.robot(x= t1[1][0],y= t1[1][1], yaw  = 0, ball = ball)
    r3 = p.robot(x= t2[0][0],y= t2[0][1], yaw  = 3.14, ball = ball)
    r4 = p.robot(x= t2[1][0],y= t2[1][1], yaw  = 3.14, ball = ball)
    ball = p.ball(x = 0,y = 0)
    gs = 0


def gamefun(t1,t2):
    global d
    global gs
    global r1f
    global r2f
    global r3f
    global r4f
    global r1
    global r2
    global ball
    rospy.Subscriber('game/status',Int32,rulecheck)
    rospy.Subscriber('ctrl',game,ctrlcallback)
    pubball = rospy.Publisher('ballpose', Pose, queue_size=10)
    pubbtwist = rospy.Publisher('balltwist', Twist, queue_size=10)
    drib = rospy.Publisher('game/dribbler', Int32, queue_size=10)
    yis = rospy.Publisher('game/dribdist', Float64, queue_size=10)
    pr1 = []
    tr1=[]
    pr2 = []
    tr2=[]
    a,r1r,t = robotpubinit(1,0)
    pr1.append(a)
    tr1.append(t)
    a,r2r,t = robotpubinit(1,1)
    pr1.append(a)
    tr1.append(t)
    a,r3r,t = robotpubinit(2,0)
    pr2.append(a)
    tr2.append(t)
    a,r4r,t = robotpubinit(2,1)
    pr2.append(a)
    tr2.append(t)
    btwist = Twist()
    rate = rospy.Rate(60)
    while True:
        bpose = Pose()
        r1.append(p.robot(x= t1[0][0],y= t1[0][1], yaw  = 0, ball = ball))
        r1.append(p.robot(x= t1[1][0],y= t1[1][1], yaw  = 0, ball = ball))
        r2.append(p.robot(x= t2[0][0],y= t2[0][1], yaw  = 3.14, ball = ball))
        r2.append(p.robot(x= t2[1][0],y= t2[1][1], yaw  = 3.14, ball = ball))

        rpose = [Pose(),Pose(),Pose(),Pose()]
        rtwist= [Twist(),Twist(),Twist(),Twist()]
        updatebpose(bpose,ball)
        updatebtwist(btwist,ball)
        updaterpose(rpose[0],r1[0])
        updaterpose(rpose[1],r1[1])
        updaterpose(rpose[2],r2[0])
        updaterpose(rpose[3],r2[1])
        updatertwist(rtwist[0],r1[0])
        updatertwist(rtwist[1],r1[1])
        updatertwist(rtwist[2],r2[0])
        updatertwist(rtwist[3],r2[1])
        pr1[0].publish(rpose[0])
        pr1[1].publish(rpose[1])
        pr2[0].publish(rpose[2])
        pr2[1].publish(rpose[3])
        tr1[0].publish(rtwist[0])
        tr1[1].publish(rtwist[1])
        tr2[0].publish(rtwist[2])
        tr2[1].publish(rtwist[3])
        pubball.publish(bpose)
        pubbtwist.publish(btwist)
        
        while not rospy.is_shutdown():
            if gs == 0:
                p.collRR(r1[0],r2[0])
                p.collRR(r1[0],r2[1])
                p.collRR(r1[0],r1[1])
                p.collRR(r1[1],r2[0])
                p.collRR(r1[1],r2[1])
                p.collRR(r2[0],r2[1])
                p.walleffect(r1[0])
                p.walleffect(r1[1])
                p.walleffect(r2[0])
                p.walleffect(r2[1])
                p.collRb(r1[0],ball)
                p.collRb(r1[1],ball)
                p.collRb(r2[0],ball)
                p.collRb(r2[1],ball)
                dribbletest(r1[0 ],r1[1],r2[0],r2[1])
                ball.finalptcotrl(200,200)
                ball.updatestate(d)
                updatebpose(bpose,ball)
                updatebtwist(btwist,ball)
                x1 = updaterpose(rpose[0],r1[0])
                x2 = updaterpose(rpose[1],r1[1])
                x3 = updaterpose(rpose[2],r2[0])
                x4 = updaterpose(rpose[3],r2[1])
                x = [x1,x2,x3,x4]
                y = max(x)
                yis.publish(y)
                r1r.publish(r1f)
                r2r.publish(r2f)
                r3r.publish(r3f)
                r4r.publish(r4f)
                pr1[0].publish(rpose[0])
                pr1[1].publish(rpose[1])
                pr2[0].publish(rpose[2])
                pr2[1].publish(rpose[3])
                pubball.publish(bpose)
                pubbtwist.publish(btwist)
                tr1[0].publish(rtwist[0])
                tr1[1].publish(rtwist[1])
                tr2[0].publish(rtwist[2])
                tr2[1].publish(rtwist[3])
                drib.publish(d)
                rate.sleep()
            else:
                dribbletest(r1[0],r1[1],r2[0],r2[1])
                ball.finalptcotrl(200,200)
                updatebpose(bpose,ball)
                updatebtwist(btwist,ball)
                x1 = updaterpose(rpose[0],r1[0])
                x2 = updaterpose(rpose[1],r1[1])
                x3 = updaterpose(rpose[2],r2[0])
                x4 = updaterpose(rpose[3],r2[1])
                x = [x1,x2,x3,x4]
                y = max(x)
                yis.publish(y)
                r1r.publish(r1f)
                r2r.publish(r2f)
                r3r.publish(r3f)
                r4r.publish(r4f)
                pr1[0].publish(rpose[0])
                pr1[1].publish(rpose[1])
                pr2[0].publish(rpose[2])
                pr2[1].publish(rpose[3])
                tr1[0].publish(rtwist[0])
                tr1[1].publish(rtwist[1])
                tr2[0].publish(rtwist[2])
                tr2[1].publish(rtwist[3])
                pubball.publish(bpose)
                pubbtwist.publish(btwist)
                drib.publish(d)
                rate.sleep()
                break



if __name__ == '__main__':
    rospy.init_node('state_server',anonymous=True)
    posa = [[-180,0],[-400,-400]]
    posb = [[400,400],[400,-400]]
    try:
        gamefun(posa,posb)
    except rospy.ROSInterruptException:
        pass
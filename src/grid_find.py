#!/usr/bin/env python
import rospy 
import tf
from sbsim.msg import goalmsg
from geometry_msgs.msg import Twist,Pose
from std_msgs.msg import Float64
import math as ma
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler

k=0
check=0
theta=0
i=0
x_b=x_r=0
y_b=y_r=0
xd=yd=0
xo=yo=0
vel_b=vel_r=0
sol_pub=rospy.Publisher('robot1n0/ptg',goalmsg,queue_size=10)
dat=goalmsg()

def botcb(data):
    global x_r,y_r
    x_r=data.position.x
    y_r=data.position.y

def botwistcb(data):
    global vel_r
    xd_r=data.linear.x
    yd_r=data.linear.y
    vel_r=ma.sqrt(xd_r**2 + yd_r**2)

def ballcallback(msg):
    global x_b,y_b
    x_b=msg.position.x
    y_b=msg.position.y
    find_sol()

def balltwistcb(msg):
    global k,vel_b,xd,yd,i,check
    xd=msg.linear.x
    yd=msg.linear.y
    vel_b=ma.sqrt(xd**2 + yd**2)
    if(not(xd)==0):
        k=yd/xd
        check=ma.atan2(yd,xd)
        
        

def find_sol():
    global k,vel_r,vel_b,x_r,y_r,x_b,y_b,xd,yd,dat,xc,i
    xc=x_b
    yc=y_b
    thtg = ma.atan2((y_b-y_r),(x_b-x_r))
    q = quaternion_from_euler(0,0,thtg)
    if(vel_b):
        if(not(vel_r)):
            vel_r=0.5
        else:
            pass
        m=y_b - k*x_b
        a=(1+k**2)*((1/vel_b)**2 - (1/vel_r)**2)
        b=2*(k*(m-y_b)-x_b)/(vel_b**2) - 2*(k*(m-y_r)-x_r)/(vel_r**2)
        c=x_b**2 - x_r**2 + (m-y_b)**2 - (m-y_r)**2
        d=(b**2 - 4*a*c)
        if(d>=0):
            d=ma.sqrt(d)
            xc1=(d-b)/(2*a)
            yc1=k*xc1 + m
            xc2=-(d+b)/(2*a)
            yc2=k*xc2 + m
            if(xd==0):
                c=yd
            else:
                c=xd
            if((xc1-x_b)/abs(xc1-x_b)==c/abs(c)):
                xc=xc1
                yc=yc1
            elif((xc2-x_b)/abs(xc2-x_b)==c/abs(c)):
                xc=xc2
                yc=yc2
        dat.status=1
        dat.posetogo = Pose(Point(xc,yc,0), Quaternion(0,0,q[2],q[3]))
        if((xc<-340)or(xc>340)or(yc>303)or(yc<-303)):
            pass
        else :
            sol_pub.publish(dat)
    else:
        dat.status=1
        dat.posetogo = Pose(Point(x_b,y_b,0),Quaternion(0,0,q[2],q[3]))
        sol_pub.publish(dat)
        

def run():
    rospy.init_node('Solver', anonymous=True)
    rospy.Subscriber('robot1n0/pose',Pose,botcb)
    rospy.Subscriber('robot1n0/twist',Twist,botwistcb)
    rospy.Subscriber('balltwist',Twist,balltwistcb)
    rospy.Subscriber('ballpose',Pose,ballcallback)
    rospy.spin()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
    
    

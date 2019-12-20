#!/usr/bin/env python
import sys
import numpy as np
import math as m




class ball:
    """
    Ball object for motion model which includes friction and collision

    data members:
    x : x coordinate of ball
    y : y coordinate of ball
    xd : dx/dt of ball
    yd : dy/dt of ball
    xdd : d^2x/dt^2 of ball
    ydd : d^2y/dt^2 of ball
    r : radius of ball
    cr : coefficient of restitution
    nu : coefficient of friction
    vthresh : max speed
    mflag : motion flag
    i : flag
    """
    #cm to pixel conversion for factor for dimensions=2.52
    def __init__(self,x=0,y=0,xd=0,yd=0,xdd=0,ydd=0):
        self.x =x
        self.y = y
        self.xd =xd
        self.yd = yd
        self.cr = 1
        self.r = 5.67
        self.nu = 1
        self.speed = 0
        self.vthresh = 1
        self.mflag = 0
        self.i = 0

    def updatestate(self,d):
        """
        d : flag for seeing if dribbled or not. d = 0 if dribbling and not dribbling if not 0
        performs the equations of motion onto the ball in discrete sense
        no acceleration and no friction (can be addded later)

        updates state variables x and y
        """
        if d == 0:
            self.x += self.xd
            self.y += self.yd
            self.speed = m.sqrt(self.xd*self.xd + self.yd*self.yd)
            
            if (self.xd>0):
                self.dirx =1
            elif (self.xd<0):
                self.dirx =-1
            else:
                self.dirx =0
            if (self.yd>0):
                self.diry =1
            elif (self.yd<0):
                self.diry =-1
            else:
                self.diry =0


            if(self.dirx ==0 and self.diry ==0):
                self.friction()
            elif (self.diry == 0 and self.dirx!=0):
                self.frictionx()
            elif (self.dirx == 0 and self.diry!=0):
                self.frictiony()

    def frictiony(self):
        """
        slowdown to ball along x
        """
        if self.speed != 0:
            self.xd -= (self.xd/self.speed)*self.nu/10

    def frictionx(self):
        """
        slowdown to ball along y
        """
        if self.speed != 0: 
            self.yd -= (self.yd/self.speed)*self.nu/10

    def friction(self):
        """
        slowdown to ball along both x and y
        """
        if self.speed != 0: 
            self.xd -= ((self.xd/self.speed)*self.nu)/10
            self.yd -= ((self.yd/self.speed)*self.nu)/10



        




class robot:
    """
    Bot object for motion model which includes friction, collision, dribble and kick

    data members:
    x : x coordinate of bot in pixels
    y : y coordinate of bot in pixels
    theta : orientation of ball in radians
    xd : dx/dt of bot in pps
    yd : dy/dt of bot in pps
    thetad : dtheta/dt of bot in radians/s
    xdd : d^2x/dt^2 of bot in pps^2
    ydd : d^2y/dt^2 of bot in pps^2
    r : radius of bot in pixels
    cr : coefficient of restitution
    nu : coefficient of friction
    dirx,diry : direction flags (1 if motion along +ve axis,-1 if along -ve axis 0 if stationary along axis)
    dribble : flag to see if dribbling
    ball : ball object associated to robot
    vthresh : max speed
    mflag : motion flag
    i : flag
    """
    def __init__(self,x,y,ball,xd=0,yd=0,xdd=0,ydd=0,yaw=0):
        self.x =x
        self.y = y
        self.xd =xd
        self.yd = yd
        self.xdd = xdd
        self.ydd = ydd
        self.theta = yaw
        self.thetad = 0
        self.cr = 0.5
        self.r = 46.6
        self.nu = 1.5
        self.vthresh = 3
        self.dirx =0
        self.diry =0
        self.speed =0
        self.dribble = 0
        self.ball = ball
        self.distdribbled = 0


    def updatestate(self):
        """
        equations of motion of bot
        updates state variables
        """
        tmvx = self.xd
        tmvy = self.yd
        self.x+=self.xd
        self.y+=self.yd
        self.xd = self.xd + self.xdd
        self.yd = self.yd + self.ydd
        self.theta = self.theta + self.thetad
        if (self.theta > 6.28):
            self.theta -= 6.28
        if (self.theta < 0):
            self.theta += 6.28
        self.thetad = 0
        self.speed =m.sqrt(self.xd*self.xd+self.yd*self.yd)
        if self.xdd==0:
            self.xd = tmvx
        if self.ydd==0:
            self.yd = tmvy
        if self.dribble == 1:
            self.distdribbled+=self.speed

    def impulse(self,acx,acy):
        """
        action of impulse acx and acy on bot
        acx,acy : delta(xd),delta(yd)
        """
        self.xdd = acx
        self.ydd = acy
        self.updatestate()
        self.xdd = 0
        self.ydd = 0

    def rotate(self,thetad):
        """
        update rotation speed
        thetad : assigned raotation speed
        """
        self.thetad = thetad

    def setspeed(self,vx,vy):
        self.xd = vx
        self.yd = vy
        self.updatestate()
        self.xd=0
        self.yd=0
    
    def movebot(self,kx,ky,ball,thetad=0):
        """
        applies impulse to robot along kx,ky along with rotation thetad also considers of dribbling or not
        impulse model must be changed to velocity model
        """
        if (self.xd>0):
            self.dirx =1
        elif (self.xd<0):
            self.dirx =-1
        else:
            self.dirx =0
        if (self.yd>0):
            self.diry =1
        elif (self.yd<0):
            self.diry =-1
        else:
            self.diry =0

        self.rotate(thetad)
        self.setspeed(kx,ky)
        if self.dribble == 1:
            ball.xd = self.xd
            ball.yd = self.yd
            ball.xdd = self.xdd 
            ball.ydd = self.ydd
            ball.x = self.x + (self.r+ball.r)*m.cos(self.theta)
            ball.y = self.y + (self.r+ball.r)*m.sin(self.theta)

    def kick(self,ball,mag):
        """
        kicks ball in the direction of facing if the ball is in dribbled state at a relative speed of mag
        """
        print("Ball kicked")
        ball.xd = self.xd + mag*m.cos(self.theta)
        ball.yd = self.yd + mag*m.sin(self.theta)
        ball.speed = m.sqrt(ball.xd*ball.xd +ball.yd*ball.yd)
        self.dribble = 0
        kc = (self.x-ball.x)/(self.r+ball.r)
        ks = (self.y-ball.y)/(self.r+ball.r)
        if kc>0:
            ball.x = self.x-(self.r+ball.r)*kc-2
        elif kc < 0:
            ball.x = self.x-(self.r+ball.r)*kc+2
        if ks > 0:
            ball.y = self.y-(self.r+ball.r)*ks+2
        elif ks < 0:
            ball.y = self.y-(self.r+self.r)*ks-2








def restricang(thtg):
    """
    0 to 2pi range restriction of angle
    """
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
    return thtg

def dist(x1,y1,x2,y2):
    """
    returns distance between points (x1,y1) and (x2,y2)
    """
    return m.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))

def disto(a,b):
    """
    returns distance when given 2 objects either ball and bot or bot and bot
    """
    return m.sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y))

def angcs(x1,y1,x2,y2):
    """
    returns the sine s and cosine c of the angle made with x axis given 2 points (x1,y1) and (x2,y2)
    """
    c = (x2 - x1)/dist(x1,y1,x2,y2)
    s = (y2 - y1)/dist(x1,y1,x2,y2)
    return [c,s]

def angcso(a,b):
    """
    returns sine and cosine between 2 objects
    """
    if(disto(a,b)!=0):
        c = (a.x - b.x)/disto(a,b)
        s = (a.y - b.y)/disto(a,b)
    else:
        c=1
        s=0
    return [c,s]

def wallcheck(a):
    """
    checks for collision with walls returns flags if collision occurs with any wall

    ox is 1 if right and -1 if collision with left wall
    oy is 1 if top and -1 if collision with bottom wall
    """
    if a.x + a.r >= 340:
        ox = 1
    elif a.x - a.r <= -340:
        ox = -1
    else:
        ox = 0
    if a.y + a.r >= 303:
        oy = 1
    elif a.y - a.r <= -303:
        oy = -1
    else:
        oy = 0
    return [ox,oy]

def colcheck(a,b):
    """
    checks if 2 objects are in collision returns 1 if collision occurs 0 if not
    """
    if disto(a,b)<=1.0+(a.r+b.r): #1.0 bias for reverse dribbling 
        return 1
    else: return 0


def collRb(R,b):
    """
    collision behavior modelling between robot and ball R is robot object and b is ball object
    """
    vthresh = 2
    [c,s] = angcso(R,b)
    if(colcheck(R,b)==1):
        if R.dribble == 1 or (abs(R.xd - b.xd) < vthresh and abs(R.yd - b.yd) < vthresh and c<=-0.97 ):
            # dribble ball
            R.dribble = 1
            b.xd = 0
            b.yd = 0
            b.xdd = 0 
            b.ydd = 0
            b.x = R.x + (R.r+b.r)*m.cos(R.theta)
            b.y = R.y + (R.r+b.r)*m.sin(R.theta)
            #print 'ball in possession'
        else:
            R.distdribbled = 0
            R.dribble = 0
            if (R.speed)<=5:
                b.cr = 0.2
            else:
                b.cr = 0.5
            if (b.speed)<=0.1:
                b.cr = 0.6
                b.i = 0
            else:
                b.cr = 0.6
            ux = R.xd - b.xd
            uy = R.yd - b.yd
            kc = (R.x-b.x)/(R.r+b.r)
            ks = (R.y-b.y)/(R.r+b.r)
            if kc>0:
                b.x = R.x-(R.r+b.r)*kc-1
            elif kc<0:
                b.x = R.x-(R.r+b.r)*kc+1
            if ks>0:
                b.y = R.y-(R.r+b.r)*ks+1
            elif ks<0:
                b.y = R.y-(R.r+b.r)*ks-1
        
            vpa = (ux*kc+uy*ks)
            vpd = (ux*ks*uy*kc)
            vx = b.cr*(vpa*kc+vpd*ks)
            vy = b.cr*(vpa*ks+vpd*kc)
            b.xd = vx + R.xd
            b.yd = vy + R.yd
            b.cr = 1

            #b.updatestate(R.dribble)
    else:
        R.distdribbled = 0
        R.dribble = 0
        if (b.xd>0):
            b.dirx =1
        elif (b.xd<0):
            b.dirx =-1
        else:
            b.dirx =0
        if (b.yd>0):
            b.diry =1
        elif (b.yd<0):
            b.diry =-1
        else:
            b.diry =0
    return R.dribble
        #b.updatestate(R.dribble)  


def collRR(a,b):
    """
    BUGGY BUGGY BUGGY BUGGY
    collision behavior for robot with robot
    """
    if(colcheck(a,b)==1):
        ux = a.xd - b.xd
        uy = a.yd - b.yd
        kc = (a.x-b.x)/(a.r+b.r)
        ks = (a.y-b.y)/(a.r+b.r)
        vpa = ux*kc+uy*ks
        vpd = ux*ks+uy*kc
        vxb = 0.2*(vpa*kc+vpd*ks)
        vyb = 0.2*(vpa*ks+vpd*kc)
        b.xd = vxb + a.xd
        b.yd = vyb + a.yd
        a.xd = vyb + a.xd
        a.yd = vxb + a.yd
        kc = (a.x-b.x)/(a.r+b.r)
        ks = (a.y-b.y)/(a.r+b.r)
        if kc>0:
            b.x = a.x-(a.r+b.r)*kc-1
        elif kc<0:
            b.x = a.x-(a.r+b.r)*kc+1
        if ks>0:
            b.y = a.y-(a.r+b.r)*ks+1
        elif ks<0:
            b.y = a.y-(a.r+b.r)*ks-1
    return [a.xd,a.yd,b.xd,b.yd]


def walleffect(a):
    """
    collision behavior with wall given object either ball or robot
    """
    [ox,oy] = wallcheck(a)
    if ox == 1 or ox == -1:
        a.xd = -a.xd*0.2
        a.x = ox*340-ox*a.r
    if oy == 1 or oy == -1:
        a.yd = -a.yd*0.2
        a.y = oy*303-oy*a.r
    return [a.xd,a.yd]



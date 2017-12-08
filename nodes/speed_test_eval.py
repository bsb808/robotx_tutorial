#!/usr/bin/env python
import rospy
from math import *
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import time

def  mdeglat(lat):
    '''
    Provides meters-per-degree latitude at a given latitude
    
    Args:
      lat (float): latitude

    Returns:
      float: meters-per-degree value
    '''
    latrad = lat*2.0*pi/360.0 ;

    dy = 111132.09 - 566.05 * cos(2.0*latrad) \
         + 1.20 * cos(4.0*latrad) \
         - 0.002 * cos(6.0*latrad)
    return dy

def mdeglon(lat):
    '''
    Provides meters-per-degree longitude at a given latitude

    Args:
      lat (float): latitude in decimal degrees

    Returns:
      float: meters per degree longitude
    '''
    latrad = lat*2.0*pi/360.0 
    dx = 111415.13 * cos(latrad) \
         - 94.55 * cos(3.0*latrad) \
	+ 0.12 * cos(5.0*latrad)
    return dx

def find_slope_int(x,y,xstart,ystart,xgoal,ygoal):
    if (xgoal-xstart)<1e-6:
        m = 1.0e6;
    else:
        m = (ygoal-ystart)/(xgoal-xstart)
    if m==0.0:
        yint = ystart
        xint = x
    elif abs(m) > 9.0e5:
        yint = y
        xint = xstart
    else:
        xint = (m/(m**2+1))*(y-ystart+m*xstart+x/m)
        yint = ystart+m*(xint-xstart)
    d2line = sqrt((x-xint)**2+(y-yint)**2)

    d2start = sqrt((xstart-xint)**2+(ystart-yint)**2)
    d2goal = sqrt((xgoal-xint)**2+(ygoal-yint)**2)
    dline =  sqrt((xgoal-xstart)**2+(ygoal-ystart)**2)
    online = False
    if (d2start <= dline) and (d2goal <= dline):
        online = True
    dx = x-xint
    if abs(dx) < 1e-6:
        dx = 0.0
    a2line = atan2(y-yint,dx)
    return (m, xint, yint, d2line, a2line, online)
    
def callback(data):
    global etime
    global t0
    global a2line10
    global testthen
    global testfthen
    global complete
    global pub

    # Locaiton of buoys
    g1 = [36.59661518,	-121.88827949]
    r1 = [36.59670531,	-121.88827819]
    g2 = [36.59661309,	-121.88805593]
    r2 = [36.59670322,	-121.88805463]

    mperdeg = (mdeglat(g1[0])+mdeglon(g1[0]))/2.0
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    #print("Current Position: %.8f, %.8f"%(x,y))

    m1, xint1, yint1, d2line1, a2line1, online1 = find_slope_int(x,y,
                                                        g1[1],g1[0],
                                                        r1[1],r1[0])
    #print m1, xint1, yint1, d2line1*mperdeg, a2line1, online1
    m2, xint2, yint2, d2line2, a2line2, online2 = find_slope_int(x,y,
                                                        g2[1],g2[0],
                                                        r2[1],r2[0])
    #print m2, xint2, yint2, d2line2*mperdeg, a2line2, online2 
    # did we cross start line?
    testnow = ((a2line1 > pi/2.0) or (a2line1 < -pi/2.0))
    testfnow = ((a2line2 > pi/2.0) or (a2line2 < -pi/2.0))
    #testnow = ((a2line1 < pi/2.0 ) and  (a2line1 > -pi/2.0))
    if testthen == None:
        testthen = testnow
    if testfthen == None:
        testfthen = testfnow
    #print (not testnow), online1, testthen
    '''if ( ((a2line10 > pi/2.0) or (a2line10 < -pi/2.0)) and 
         online1 and 
         ((a2line1 < pi/2.0 ) and  (a2line1 > -pi/2.0) ) ):
         '''
    if ((not testnow) and testthen and online1):
        print "Crossed start line - starting timer..."
        t0 = rospy.get_time()
        complete = False
        #a2line10 = a2line1        

    if t0:
        #print (not testfnow), testfthen, online2
        if ((not testfnow) and testfthen and online2):
            print ("Course complete!")   
            etime = rospy.get_time()-t0
            print "Elapsed time = %.2f seconds"%etime
            # publish
            msg = Float32()
            msg.data = etime
            pub.publish(msg)
            complete = True

            
        if not complete:
            etime = rospy.get_time()-t0
            print "Elapsed time = %.2f seconds"%etime
    else:
        print "Waiting for crossing start line to start timer..."
    testthen = testnow
    testfthen = testfnow

             



def listener():
    global t0
    t0 = None
    global a2line10
    a2line10 = 0.0
    global testthen 
    testthen = None
    global testfthen
    testfthen = None
    global complete
    complete = False
    global pub
    rospy.init_node('speed_test_eval', anonymous=True)

    rospy.Subscriber("nav_odom", Odometry, callback)
    pub = rospy.Publisher('etime',Float32,queue_size=10)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

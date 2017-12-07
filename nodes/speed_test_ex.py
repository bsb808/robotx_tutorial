#!/usr/bin/env python

# System imports
import sys
import time
from math import *
import numpy as np

# Rospy import
import rospy

# Import message and service types
from kingfisher_msgs.msg import Course

# Initialize the node and wait a moment
print('Initializing ROS node')
rospy.init_node('python_example')
time.sleep(1)

# Make an array of commands for the vessel to follow.  
# The first column is the speed
# second is the yaw
# and third is the amount of time to follow that course

# FOR THE TUTORIAL - YOU SHOULD ONLY NEED TO MODIFY THIS VARIABLE!
cmds = np.array([ [1.0, 0.0, 5.0],
                  [1.0, 1.57, 5.0],
                  [1.0, 3.14, 5.0],
                  [1.0, -1.57, 5.0] ])
                
# Setup publication - send course command to our PID control (/pid_control node)
cmdPub = rospy.Publisher('/cmd_course',Course,queue_size=10);
time.sleep(1); # Wait to ensure publisher is registered

# Create an empty message for publication
cmdMsg = Course()

# Loop throug the commands, one at a time
rate = rospy.Rate(10)  # Publishing rate in Hz
                
for ii in range(cmds.shape[0]):
    cmdMsg.speed=cmds[ii,0]
    cmdMsg.yaw=cmds[ii,1]
    dt = cmds[ii,2]
    print("Sending course %d of %d: speed=%.2f [m/s], yaw=%.2f [rad] for %.2f sec"%(ii,cmds.shape[0],cmdMsg.speed,cmdMsg.yaw,dt))
    t0 = time.time()   # time for calc. elapsed time
    etime = 0.0
    while (dt >= etime):
        cmdPub.publish(cmdMsg)
        rate.sleep()
        etime = time.time()-t0

# Send a last command to zero the speed and yaw
cmdMsg.speed = 0.0
cmdMsg.yaw = 0.0
print("Zeroing the course")
cmdPub.publish(cmdMsg)

print("That's all folks...")
    
    




#!/usr/bin/env python

import numpy as np
from numpy import pi
from math import sin, cos
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
#import pandas as pd

# ~~ Setting and Constants ~~
numReadings   = 160
threshold     =   9.0
preturn_thresh = 5.0
plotCarts     =   1

# ~~ Variables ~~
global lastOflow
global a_0, a_1, a_2, b_1, b_2
global forward_speed, yaw_rate
lastOflow = [ 0 for i in range( numReadings ) ]

car_state = ''
len_hist = 500

def oflow_cb( msg ):
    global lastOflow
    """ Process the laser scan message """
    lastOflow = msg.data  #lastDepthLaserScan = [ elem/25.50 for elem in msg.data ] # scale [0,255] to [0,10]

minAng = 0
maxAng = 2*pi
rospy.init_node( 'scan_plot' , anonymous = True )

# rospy.Subscriber( "wfi/horiz/image_scan" , Float32MultiArray , image_scan_cb )
rospy.Subscriber( "/optic_flow_node/tang_optic_flow", Float32MultiArray , oflow_cb )

try:
    while ( not rospy.is_shutdown() ):
        lastOflowNP = np.asarray( lastOflow )

        plt.clf() # Clear all figures

        plt.figure(1)
		#plt.subplot(3,1,1)
		# Horizontal Depth Scan: Array Index vs. Depth [m]
		# plt.figure(num=1, figsize=(9, 6), dpi=80, facecolor='w', edgecolor='k')
        plt.plot( lastOflowNP , 'b.' )
        plt.hold( False )
        plt.xlim( [ 0 , 160 ] )
        plt.ylim( [ -0.5 , 0.5] )
        plt.xlabel("Gamma")
        plt.title("Optic FLOW")


        plt.pause( 0.01 )
    plt.show()
except KeyboardInterrupt:
	pass

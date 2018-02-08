#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Float64MultiArray
from arm_util import *
import signal
import sys

#-----------------------------------------------------------------
#SIGINT handler
def sigint_handler(signal, frame):
    sys.exit(0)

def callback(inp):
    global pot_val1,pot_val2
    pot_val1=inp.data[0]
    pot_val2=inp.data[1]
    
if __name__ == '__main__':

    signal.signal(signal.SIGINT, sigint_handler)
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("Pot_Val", Float64MultiArray, callback)
    pub=rospy.Publisher("Theta_Phi",Float64MultiArray,queue_size=10)
    #Default Values
    pot_val1=1030
    pot_val2=90
    r_time = rospy.Rate(1)
    while True:
        L1,L2=get_act_lengths(pot_val1,pot_val2)
        theta,phi=L_to_Angle(L1,L2)
        print("Theta = "+str(theta)+" Phi = "+str(phi))
        pot_val=Float64MultiArray(data=[theta,phi])
        pub.publish(pot_val)
        r_time.sleep()



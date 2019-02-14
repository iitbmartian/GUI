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

def callback_pot1(inp):
    global pot_val_arr
    pot_val_arr[0]=inp.data[0]
    pot_val_arr[1]=inp.data[1]


def callback_pot2(inp):
    global pot_val_arr
    pot_val_arr[2]=inp.data[0]
    pot_val_arr[3]=inp.data[1]
    

if __name__ == '__main__':

    signal.signal(signal.SIGINT, sigint_handler)
    arguments = sys.argv[1:]
    arg_count = len(arguments)
    #if(arg_count!=1):
    #    print("Wrong usage of arguments")
    #    exit()
    #init_angle_encoder=float(sys.argv[1])
    #print(init_angle_encoder+1)
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("Pot_Val_Claw1", Float64MultiArray, callback_pot1)
    rospy.Subscriber("Pot_Val_Claw2", Float64MultiArray, callback_pot2)
    pub=rospy.Publisher("arm_angs",Float64MultiArray,queue_size=10)
    #Default Values
    pot_val_arr=np.zeros(4)
    pot_val_arr[0]=500
    pot_val_arr[1]=1700
    pot_val_arr[2]=245
    pot_val_arr[3]=611
    r_time = rospy.Rate(100)
    counter=1
    n=5
    arm_angs=np.zeros(6)
    while True:
        act_L=get_act_lengths(pot_val_arr)
        arm_angs+=L_to_Angle(act_L)
        # print('Counter:'+str(counter))
        # if(counter%200==0):
        #     print(act_L*100)
        #     print(repr(arm_angs))
        #     counter=0
        if(counter%n==0):
            arm_angs_pubval=Float64MultiArray(data=arm_angs/n)
            pub.publish(arm_angs_pubval)
            # print("Published")
            arm_angs=np.zeros(6)
        counter+=1

        r_time.sleep()



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

def callback_pot(inp):
    global pot_val1,pot_val2
    pot_val1=inp.data[0]
    pot_val2=inp.data[1]
    

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
    rospy.Subscriber("Pot_Val", Float64MultiArray, callback_pot)
    pub=rospy.Publisher("Theta_Phi",Float64MultiArray,queue_size=10)
    #Default Values
    pot_val1=1030
    pot_val2=90
    r_time = rospy.Rate(100)
    counter=1
    while True:
        L1,L2=get_act_lengths(pot_val2,pot_val1)
        theta,phi=L_to_Angle(L1,L2)
        if(counter%200==0):
            print("Theta = "+str(theta*180/np.pi)+" Phi = "+str(phi*180/np.pi))
            counter=0
        counter+=1
        pot_val=Float64MultiArray(data=[theta+1.22,phi-0.52])
        pub.publish(pot_val)
        r_time.sleep()



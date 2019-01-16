#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
import signal
from std_msgs.msg import String, Float32
import time
#---------------------------------------------------------------------------
#SIGINT handler
def sigint_handler(signal, frame):
    #Do something while breaking
    print("\nCtrl C Pressed")
    sys.exit(0)
#---------------------------------------------------------------------------
#Channel Params
signal.signal(signal.SIGINT, sigint_handler)
i=1
pub_str = rospy.Publisher('pubAgrimStr', String, queue_size=10)
pub_flt = rospy.Publisher('pubAgrimFlt', Float32, queue_size=10)
rospy.init_node('handler', anonymous=True)
while True:
    # Do something here
    print(i)
    pub_str.publish("Hello !")
    time.sleep(1)
    pub_flt.publish(i)
    time.sleep(2)
    i=i+0.001
# def talker():
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()


#!/usr/bin/env python
"""
Copyright (c) 2019 - present AppSeed.us
"""

import os
import sys

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
import signal

#pub = rospy.Publisher('Django_node', String, queue_size=10)
virtjoy_pub = rospy.Publisher('Virtjoy_pub', Joy, queue_size=10)
# channeling_pub = rospy.Publisher('channeling', Float32MultiArray, queue_size=10)
# raman_pub = rospy.Publisher('raman', Float32MultiArray, queue_size=10)
# microscope_pub = rospy.Publisher('microscope', Float32MultiArray, queue_size=10)
# stewart_pub = rospy.Publisher('stewart', Float32MultiArray, queue_size=10)
node = rospy.init_node('talker')#, anonymous=True


# def microscope_callback(msg):
#     global _actuator_angle
#     _actuator_angle = msg.data[0]

# def stewart_callback(msg):
#     global _stewart_data
#     _stewart_data = msg.data

# global _actuator_angle
# _actuator_angle = 0
# microscope_sub = rospy.Subscriber('actuator', Float32MultiArray, microscope_callback)

# global _stewart_data# 1,...6, vertical motion, roll
# _stewart_data = [0,0,0,0,0,0,0,0]
# microscope_sub = rospy.Subscriber('stewart_text', Float32MultiArray, stewart_callback)

def sigint_handler(signal, frame):
    sys.exit(0)

#def send_msg(data):
#    global pub
#    pub.publish(data)

def send_joy(axes, buttons):
    global virtjoy_pub
    # print(axes)
    virtjoy_pub.publish(Joy(axes = axes, buttons = buttons))
# def send_channeling(array):
#     global channeling_pub
#     # print(array)
#     channeling_pub.publish(Float32MultiArray(data = array))
# def send_raman(array):
#     global raman_pub
#     # print(array)
#     raman_pub.publish(Float32MultiArray(data = array))
# def send_microscope(array):
#     global microscope_pub
#     # print(array)
#     microscope_pub.publish(Float32MultiArray(data = array))
# def send_stewart(array):
#     global stewart_pub
#     # print(array)
#     stewart_pub.publish(Float32MultiArray(data = array))

# def execute_file(data):
    # print("Here")
    # os.system('python ' +str(data)+'.py')


def main():
    os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'core.settings')
    try:
        from django.core.management import execute_from_command_line
    except ImportError as exc:
        raise ImportError(
            "Couldn't import Django. Are you sure it's installed and "
            "available on your PYTHONPATH environment variable? Did you "
            "forget to activate a virtual environment?"
        ) from exc
    execute_from_command_line(sys.argv)

if __name__ == '__main__':
    main()

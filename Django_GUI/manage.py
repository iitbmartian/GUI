#!/usr/bin/env python
import os
import sys
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
import signal

#pub = rospy.Publisher('Django_node', String, queue_size=10)
virtjoy_pub = rospy.Publisher('Virtjoy_pub', Joy, queue_size=10)
channeling_pub = rospy.Publisher('channeling', Float32MultiArray, queue_size=10)
raman_pub = rospy.Publisher('raman', Float32MultiArray, queue_size=10)
microscope_pub = rospy.Publisher('microscope', Float32MultiArray, queue_size=10)
stewart_pub = rospy.Publisher('stewart', Float32MultiArray, queue_size=10)
rospy.init_node('talker')#, anonymous=True

def sigint_handler(signal, frame):
    sys.exit(0)

#def send_msg(data):
#    global pub
#    pub.publish(data)

def send_joy(axes, buttons):
    global virtjoy_pub
    # print(array)
    virtjoy_pub.publish(Joy(axes = axes, buttons = buttons))
def send_channeling(array):
    global channeling_pub
    # print(array)
    channeling_pub.publish(Float32MultiArray(data = array))
def send_raman(array):
    global raman_pub
    # print(array)
    raman_pub.publish(Float32MultiArray(data = array))
def send_microscope(array):
    global microscope_pub
    # print(array)
    microscope_pub.publish(Float32MultiArray(data = array))
def send_channeling(array):
    global stewart_pub
    # print(array)
    stewart_pub.publish(Float32MultiArray(data = array))

# def execute_file(data):
    # print("Here")
    # os.system('python ' +str(data)+'.py')


if __name__ == "__main__":
    signal.signal(signal.SIGINT, sigint_handler)
    os.environ.setdefault("DJANGO_SETTINGS_MODULE", "mysite.settings")
    try:
        from django.core.management import execute_from_command_line
    except ImportError:
        # The above import may fail for some other reason. Ensure that the
        # issue is really that Django is missing to avoid masking other
        # exceptions on Python 2.
        try:
            import django
        except ImportError:
            raise ImportError(
                "Couldn't import Django. Are you sure it's installed and "
                "available on your PYTHONPATH environment variable? Did you "
                "forget to activate a virtual environment?"
            )
        raise
    execute_from_command_line(sys.argv)

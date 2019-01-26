#!/usr/bin/env python
import os
import sys
import rospy
from std_msgs.msg import String, Float32MultiArray
import signal

pub = rospy.Publisher('Django_node', String, queue_size=10)
virtjoy_pub = rospy.Publisher('Virtjoy_pub', Float32MultiArray, queue_size=10)
rospy.init_node('talker', anonymous=True)

def sigint_handler(signal, frame):
    sys.exit(0)

def send_msg(data):
    global pub
    pub.publish(data)

def send_joy(data):
    global virtjoy_pub
    virtjoy_pub.publish(Float32MultiArray(data=data))

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

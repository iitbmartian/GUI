#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3

def talker():
	topic = "cmd_vel"
	rospy.init_node('test_node')
	pub = rospy.Publisher(topic, Twist) # The publisher
	rate = rospy.Rate(10) # Check value

	while not rospy.is_shutdown():
		rospy.loginfo("Publishing at %s" % rospy.get_time())
		msg = Twist(Vector3(1.0, 0, 0), Vector3(0, 0, 0.4)) # Enter message of type Twist
		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
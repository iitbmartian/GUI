#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Float32MultiArray
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion, Pose
from visualization_msgs.msg import Marker
from arm_util import *
import signal
import sys

#-----------------------------------------------------------------
#SIGINT handler
def sigint_handler(signal, frame):
    sys.exit(0)


if __name__ == '__main__':

    signal.signal(signal.SIGINT, sigint_handler)
    seq=0
    rospy.init_node('Path_node_conv', anonymous=True)
    pub=rospy.Publisher("Path_msg",Marker,queue_size=10)
    #Default Values
    r_time = rospy.Rate(1)
    while True:
        marker=Marker()
        marker.header.stamp=  rospy.Time.now()
        marker.header.seq=seq
        seq+=1
        marker.header.frame_id = "/base_link"
        marker.type = marker.MESH_RESOURCE
        marker.mesh_resource = "package://aspha18_urdf/meshes/complete.stl";
        marker.action = marker.ADD
        marker.scale.x=50
        marker.scale.y=50
        marker.scale.z=50
        marker.color.g = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.2
        marker.color.b = 0.2
        marker.pose.position.z=16
        marker.
        pub.publish(marker)
        print(marker)
        r_time.sleep()



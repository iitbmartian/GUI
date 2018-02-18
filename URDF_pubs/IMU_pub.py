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

def callback_IMU(inp):
    global seq, pub2, pub
    orientation=np.zeros(3)
    orientation[0]=inp.data[0]
    orientation[1]=inp.data[1]
    orientation[2]=inp.data[2]
    marker=Marker()
    marker.header.stamp=  rospy.Time.now()
    marker.header.seq=seq
    seq+=1
    marker.header.frame_id = "/kinect2_rgb_optical_frame"
    marker.type = marker.ARROW
    qt_arr=quaternion_from_euler(orientation[1],-orientation[2]+1.57,orientation[0])
    qt=Quaternion(qt_arr[0],qt_arr[1],qt_arr[2],qt_arr[3])
    marker.action = marker.ADD
    marker.pose.orientation = qt
    marker.scale.x=50
    marker.scale.y=4
    marker.scale.z=4
    marker.pose.position.y=-16
    marker.pose.position.z=0
    marker.pose.position.x=-0   


    marker.color.g = 1.0
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.b = 0.0
    pub.publish(marker) 


    marker_rover=Marker()
    marker_rover.header.stamp=  rospy.Time.now()
    marker_rover.header.seq=seq
    seq+=1
    marker_rover.header.frame_id = "/base_link"
    marker_rover.type = marker_rover.MESH_RESOURCE
    marker_rover.mesh_resource = "package://aspha18_urdf/meshes/complete.stl";
    marker_rover.action = marker_rover.ADD
    marker_rover.scale.x=30
    marker_rover.scale.y=30
    marker_rover.scale.z=30
    marker_rover.color.g = 0.2
    marker_rover.color.a = 1.0
    marker_rover.color.r = 0.2
    marker_rover.color.b = 0.2
    qt_arr2=quaternion_from_euler(orientation[0],orientation[1],orientation[2]+3.14)
    qt2=Quaternion(qt_arr2[0],qt_arr2[1],qt_arr2[2],qt_arr2[3])

    marker_rover.pose.orientation = qt2

    marker_rover.pose.position.z=16
    pub2.publish(marker_rover)   

if __name__ == '__main__':

    signal.signal(signal.SIGINT, sigint_handler)
    seq=0
    rospy.init_node('IMU_conv', anonymous=True)
    orientation=np.zeros(3)
    pub=rospy.Publisher("IMU_msg",Marker,queue_size=10)
    pub2=rospy.Publisher("Path_msg",Marker,queue_size=10)
    rospy.Subscriber("IMU", Float32MultiArray, callback_IMU)
    #Default Values
    rospy.spin()



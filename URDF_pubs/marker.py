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

def get_bearing(pos1, pos2):
	[lat1, long1] = pos1
	[lat2, long2] = pos2

	dLon = (long2 - long1)
	y = math.sin(dLon) * math.cos(lat2)
	x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1)* math.cos(lat2) * math.cos(dLon)
	return math.atan2(y, x)

def get_distance(pos1, pos2):
	[lat1, long1] = pos1
	[lat2, long2] = pos2

	R = 6371 # Radius of the earth in km
	dLat = math.pi*(lat2 - lat1)/180
	dLon = math.pi*(long2 - long1)/180
	a = math.sin(dLat/2) * math.sin(dLat/2) +math.cos(math.pi*(lat1)/180) * math.cos(math.pi*(lat2)/180) * math.sin(dLon/2) * math.sin(dLon/2)
	c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
	d = R * c # Distance in km
	
	return d

#-----------------------------------------------------------------
# Callback function for LatLon publisher
def gps_callback(inp):
	global tmarkerx,tmarkery
	curr_gps=[inp.data[0],inp.data[1]]
	theta=get_bearing(curr_gps,target_gps)
	d=get_distance(curr_gps,target_gps)
	tmarkerx=d*np.cos(theta)
	tmarkery=d*np.sin(theta)

if __name__ == '__main__':

    signal.signal(signal.SIGINT, sigint_handler)
    seq=0
    tseq=0
    rospy.init_node('Path_node_conv', anonymous=True)
    pub=rospy.Publisher("Path_msg",Marker,queue_size=10)
    tpub=rospy.Publisher("Target_msg",Marker,queue_size=10)
    rospy.Subscriber("LatLon", Float64MultiArray, gps_callback)
    # Global variables to store x,y of target marker
    tmarkerx=0
    tmarkery=0
    target_gps=[19,72]
    const_fac=1
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
        pub.publish(marker)
        #print(marker)
        tmarker=Marker()
        tmarker.header.stamp=  rospy.Time.now()
        tmarker.header.seq=tseq
        tseq+=1
        tmarker.header.frame_id = "/base_link"
        tmarker.type = tmarker.MESH_RESOURCE
        tmarker.mesh_resource = "package://aspha18_urdf/meshes/tGPS.stl";
        tmarker.action = tmarker.ADD
        tmarker.scale.x=1
        tmarker.scale.y=1
        tmarker.scale.z=1
        tmarker.color.g = 0.2
        tmarker.color.a = 1.0
        tmarker.color.r = 0.2
        tmarker.color.b = 0.2
        tmarker.pose.position.z=0
        tmarker.pose.position.x=tmarkerx
        tmarker.pose.position.y=tmarkery
        tpub.publish(tmarker)
        #print(tmarker)

        r_time.sleep()



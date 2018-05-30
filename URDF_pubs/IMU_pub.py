#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Float32MultiArray,Float64MultiArray
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion, Pose
from visualization_msgs.msg import Marker,MarkerArray
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
    qt_arr=quaternion_from_euler(0,-orientation[2],0)
    qt=Quaternion(qt_arr[0],qt_arr[1],qt_arr[2],qt_arr[3])
    marker.action = marker.ADD
    marker.pose.orientation = qt
    marker.scale.x=5
    marker.scale.y=0.4
    marker.scale.z=0.4
    marker.pose.position.y=-1
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
    marker_rover.scale.x=3
    marker_rover.scale.y=3
    marker_rover.scale.z=3
    marker_rover.color.g = 0.2
    marker_rover.color.a = 1.0
    marker_rover.color.r = 0.2
    marker_rover.color.b = 0.2
    qt_arr2=quaternion_from_euler(0,0,orientation[2]-1.57)
    qt2=Quaternion(qt_arr2[0],qt_arr2[1],qt_arr2[2],qt_arr2[3])

    marker_rover.pose.orientation = qt2

    marker_rover.pose.position.z=1
    pub2.publish(marker_rover)   

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
    global tseq,tpub
    
    curr_gps=[inp.data[0],inp.data[1]]
    num_targets=len(target_gps)
    markerArray = MarkerArray()


    for i in range(num_targets):
        theta=get_bearing(curr_gps,target_gps[i])
        #print(theta*180/np.pi)
        d=get_distance(curr_gps,target_gps[i])
        #print(d)
        tmarkerx=d*np.cos(-theta)*const_fac
        tmarkery=d*np.sin(-theta)*const_fac
        
        tmarker=Marker()
        tmarker.header.stamp=  rospy.Time.now()
        tmarker.header.seq=tseq
        tseq+=1
        tmarker.header.frame_id = "/base_link"
        tmarker.type = tmarker.MESH_RESOURCE

        tmarker.mesh_resource = "package://aspha18_urdf/meshes/Target_"+str(i+1)+".STL";
        tmarker.action = tmarker.ADD
        tmarker.scale.x=0.15
        tmarker.scale.y=0.15
        tmarker.scale.z=0.15
        tmarker.color.g = 0.2
        tmarker.color.a = 1.0
        tmarker.color.r = 1.0
        tmarker.color.b = 0.2
        tmarker.pose.position.z=5
        tmarker.pose.position.x=tmarkerx
        tmarker.pose.position.y=tmarkery
        tmarker.id=i
        markerArray.markers.append(tmarker)
    
    
    tpub.publish(markerArray)
    

if __name__ == '__main__':

    signal.signal(signal.SIGINT, sigint_handler)
    seq=0
    tseq=0
    orientation=np.zeros(3)
    target_gps=[[38.37205324,-110.70453928],[38.37190412,-110.70397175]]
    #target_gps=[19.13294634,72.91607273]
    const_fac=1000
    
    rospy.init_node('IMU_conv', anonymous=True)
    pub=rospy.Publisher("IMU_msg",Marker,queue_size=10)
    pub2=rospy.Publisher("Path_msg",Marker,queue_size=10)
    rospy.Subscriber("IMU", Float32MultiArray, callback_IMU)
    tpub=rospy.Publisher("Target_msg",MarkerArray,queue_size=10)
    rospy.Subscriber("LatLon", Float64MultiArray, gps_callback)
    # Global variables to store x,y of target marker
    #target_gps=[38.37190412,-110.70397175]
    
    #Default Values
    rospy.spin()



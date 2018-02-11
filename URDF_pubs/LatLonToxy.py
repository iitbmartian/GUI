#!/usr/bin/env python
from __future__ import division
import rospy
from std_msgs.msg import String,Float64MultiArray
import geometry_msgs.msg as gm
import nav_msgs.msg as nav_msgs
import signal
import sys
import numpy as np

#-----------------------------------------------------------------
#SIGINT handler
def sigint_handler(signal, frame):
    sys.exit(0)

def callback(inp):
    global prev_x,prev_y,start,path
    lat=inp.data[0]
    lon=inp.data[1]
    x,y=latLonToTileCoords(lat,lon)
    #print("X: "+str(x)+"Y:"+str(y))
    if((x!=prev_x) or(y!=prev_y)):
        pose = gm.Pose()
        pose.position = gm.Point(lat,lon,0.0)
        print(pose)
        #pose.pose.orientation = _to_quaternion(path_theta[i])
        path.poses.append(pose)
            
    prev_y=y
    prev_x=x
    if(start==0):
        start=1

def latLonToTileCoords(lat, lon):
  rho=np.pi/180
  lat_rad = lat*rho
  n = 2**14
  x = n*((lon+180)/360.0)
  y = n*(1-(np.log(np.tan(lat_rad)+(1/np.cos(lat_rad)))/np.pi))/2
  return x,y


if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigint_handler)
    prev_x=0
    prev_y=0
    path = gm.PoseArray()
    path.header.frame_id='base_link'
    start=0
    rospy.init_node('latlon_to_xy', anonymous=True)
    rospy.Subscriber("LatLon",Float64MultiArray , callback)
    pub=rospy.Publisher("tiles",gm.PoseArray,queue_size=10)
    rate = rospy.Rate(1) # 10hz
    while True:
        if(start!=0):
            pub.publish(path)
        rate.sleep()
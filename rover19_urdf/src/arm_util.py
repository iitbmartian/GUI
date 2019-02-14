#!/usr/bin/env python
from __future__ import division 
import numpy as np
import scipy.linalg as la
import math

def get_act_lengths(pot_arr):
    pot_max=np.zeros(4)
    pot_min=np.zeros(4)
    lenghts=np.zeros(4)
    #Elbow Actuator
    pot_max[0]=1750
    pot_min[0]=70
    lenghts[0]=-(pot_arr[0]-pot_min[0])/(pot_max[0]-pot_min[0])*(9.16)+11.7
    #Shoulder Actuator
    pot_max[1]=1770
    pot_min[1]=750
    lenghts[1]=-(pot_arr[1]-pot_min[1])/(pot_max[1]-pot_min[1])*(5.46)+8
    #Wrist Actuator
    pot_max[2]=700
    pot_min[2]=35 
    lenghts[2]=(pot_arr[2]-pot_min[2])/(pot_max[2]-pot_min[2])*(5)+2
    #Gripper Actuator
    pot_max[3]=1120 #3.8cm
    pot_min[3]=230 #1.6
    lenghts[3]=(pot_arr[3]-pot_min[3])/(pot_max[3]-pot_min[3])*(2.2)+1.6
    return lenghts/100

def L_to_Angle(act_L):
    arm_angs=np.zeros(6)
    #angle of shoulder link with base
    arm_angs[0]=2.41-np.arccos((721.25-(23+act_L[1]*100)**2)/(349.44))
    arm_angs[1]=np.pi-(2.55-np.arccos((1077.01-(23+act_L[0]*100)**2)/(737.3640117)))
    arm_angs[2]=np.pi-(3294-368*(24.7+act_L[2]*100)+14.2*((24.7+act_L[2]*100)**2)-0.186*((24.7+act_L[2]*100)**3))*3.14/180
    arm_angs[3]=-0.1788*(act_L[3]*100-2.86)
    arm_angs[4]=-0.005671*(act_L[3]*100-2.86)
    arm_angs[5]=-0.11382*(act_L[3]*100-2.86)
    # arm_angs[3]=(82.8+3.74*(16+act_L[3]*1000)-0.096*(16+act_L[3]*1000)**2+0.000785*((16+act_L[3]*1000)**3))*3.14/180
    
    return arm_angs

'''
(Shoulder)Pot1_Max:1729
(Shoulder)Pot1_Min:140 
(Elbow)Pot2_Max:1717
(Elbow)Pot2_Min:73  

Testing Code
#Get Angles from pot measurement
(estimated_theta,estimated_phi)=L_to_Angle(29,27)
print((estimated_theta,estimated_phi))
#Get Target Theta and Phi
(target_theta,target_phi)=get_target_angles(estimated_theta,estimated_phi,11,0)
print((target_theta,target_phi))
#Get the Final Actuator lengths to be set
print (get_actuator_lengths(target_theta,target_phi))
'''
'''
Example Usage
#Get Lenghts from pot measurement: Write a function
get_act_lengths(pot1,pot2)
# Get theta, phi from actuator lenghts
(estimated_theta,estimated_phi)=L_to_Angle(act1,act2)
#Get Target Theta and Phi
(target_theta,target_phi)=get_target_angles(estimated_theta,estimated_phi,dx,dy)
#Get the Final Actuator lengths to be set
get_actuator_lengths(target_theta,target_phi)
'''
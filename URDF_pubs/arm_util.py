#!/usr/bin/env python
from __future__ import division 
import numpy as np
import scipy.linalg as la
import math

def get_curr_pt(phi,theta):
    L1=55
    L2=35
    theta_r=(np.pi/180)*theta
    phi_r=(np.pi/180)*phi
    elb_pt_x=L1*np.cos(phi_r)
    elb_pt_y=L1*np.sin(phi_r)
    target_pt_x=elb_pt_x+L2*np.cos(theta_r)
    target_pt_y=elb_pt_y+L2*np.sin(theta_r)
    return (target_pt_x,target_pt_y)


def get_elb_pt(phi_r,theta_r):
    L1=55
    L2=35
    elb_pt_x=L1*np.cos(phi_r)
    elb_pt_y=L1*np.sin(phi_r)
    elb_pt=np.array([elb_pt_x,elb_pt_y])
    return elb_pt

def get_actuator_lengths(curr_theta,curr_phi):
    L1=55
    Lac1=30
    Lac2=6
    ac2_of1=30
    ac2_of2=6
    ac2_of3=4
    L2=35
    # Convert to radians
    curr_theta_r=(np.pi/180)*curr_theta
    curr_phi_r=(np.pi/180)*curr_phi
    
    
    # Get current elbow point
    curr_elb_pt_x=L1*np.cos(curr_phi_r)
    curr_elb_pt_y=L1*np.sin(curr_phi_r)
    
    # Get current actuator endpoint2
    curr_ac1_pt_x=Lac1*np.cos(curr_phi_r)
    curr_ac1_pt_y=Lac1*np.sin(curr_phi_r)
    
    # Get actuator endpoint1
    orig_ac1_pt_x=-5
    orig_ac1_pt_y=5
    
    #-----------------------------------------------------------------------------------
    # Hence, Length of actuator 1 will be the distance between orig and curr ac1 pts
    ac1_ep1=np.array([curr_ac1_pt_x,curr_ac1_pt_y])
    ac1_ep2=np.array([orig_ac1_pt_x,orig_ac1_pt_y])
    
    ac1_state=la.norm(ac1_ep1-ac1_ep2)
    #print("Length of Actuator 1 is:" + str(ac1_state*55))
    
    # Mathematically, it will be : sqrt((curr_pt+5/55)^2)
    #-----------------------------------------------------------------------------------
    
    # Temporary pt1 for actuator 2, the point closest to actuator2 end point2 on arm
    temp_ac2_pt1_x=curr_elb_pt_x+Lac2*np.cos(curr_theta_r)
    temp_ac2_pt1_y=curr_elb_pt_y+Lac2*np.sin(curr_theta_r)
    
    # Actuator2 end point 2, with an offset calculation
    curr_ac2_pt_x=temp_ac2_pt1_x+ac2_of3*np.cos(np.pi/2-curr_theta_r)
    curr_ac2_pt_y=temp_ac2_pt1_y-ac2_of3*np.sin(np.pi/2-curr_theta_r)
    
    # Temporary pt2 for actuator 2, the point closest to actuator2 end point1 on arm
    temp_ac2_pt2_x=curr_elb_pt_x+ac2_of1*np.cos(np.pi+curr_phi_r)
    temp_ac2_pt2_y=curr_elb_pt_y+ac2_of1*np.sin(np.pi+curr_phi_r)
    
    # Actuator2 end point 1, with an offset calculation
    orig_ac2_pt_x=temp_ac2_pt2_x+ac2_of2*np.cos(np.pi/2-curr_phi_r)
    orig_ac2_pt_y=temp_ac2_pt2_y-ac2_of2*np.sin(np.pi/2-curr_phi_r)
    
    curr_target_pt_x=curr_elb_pt_x+L2*np.cos(curr_theta_r)
    curr_target_pt_y=curr_elb_pt_y+L2*np.sin(curr_theta_r)

    #-----------------------------------------------------------------------------------
    # Length of Actuator 2
    ac2_ep1=np.array([curr_ac2_pt_x,curr_ac2_pt_y])
    ac2_ep2=np.array([orig_ac2_pt_x,orig_ac2_pt_y])
    
    ac2_state=la.norm(ac2_ep1-ac2_ep2)
    #print("Length of Actuator 2 is:" + str(ac2_state*55))
    #-----------------------------------------------------------------------------------
    
    return ((ac1_state),(ac2_state))
    #plt.plot([prev_target_pt_x,curr_target_pt_x],[prev_target_pt_y,curr_target_pt_y])

def L_to_Angle(ac1_L,ac2_L):
    
    ac1_arm=30
    ac1_of=5
    ac2_arm1=6
    ac2_of1=30
    ac2_of2=6
    ac2_of3=4

    cos_num=pow(ac1_L,2)-2*pow(ac1_of,2)-pow(ac1_arm,2)
    cos_den=np.sqrt(2)*2*ac1_of*ac1_arm
    if(cos_num>cos_den):
        print("Danger 1")
    else:
        estimated_phi=(np.arccos(cos_num/cos_den)*180/np.pi)-45
        #print(estimated_phi)
    
    estimated_phi_r=estimated_phi*np.pi/180
    K1=ac2_of1*np.cos(estimated_phi_r)-ac2_of2*np.sin(estimated_phi_r)
    K2=ac2_of1*np.sin(estimated_phi_r)+ac2_of2*np.cos(estimated_phi_r)
    a=ac2_arm1
    b=ac2_of3
    
    exp_num=pow(ac2_L,2)-pow(a,2)-pow(b,2)-pow(K1,2)-pow(K2,2)
    A=2*a*K1-2*b*K2
    B=2*b*K1+2*a*K2
    exp_den=np.sqrt(pow(A,2)+pow(B,2))
    if(exp_num>exp_den):
        print("Danger 2")
    else:
        expression=(exp_num)/(exp_den)
        if(A>exp_den):
            print("Danger 3")
        else:
            alpha=np.arcsin(A/exp_den)
            estimated_theta=(np.arcsin(expression)-alpha)*180/np.pi
            #print(estimated_theta)

    return (estimated_theta,estimated_phi)


def get_target_angles(theta,phi,dx,dy):
    L1=55
    L2=35    
    theta_0=theta*np.pi/180
    phi_0=phi*np.pi/180
    curr_vec=np.zeros(2)
    curr_vec[0]=phi_0
    curr_vec[1]=theta_0
    (curr_x,curr_y)=get_curr_pt(phi,theta)
    c1=np.array([0,0])
    c2=np.array([curr_x+dx,curr_y+dy])
    r1=L1
    r2=L2
    d_2=pow(la.norm(c1-c2),2)
    det=(pow((r1+r2),2)-d_2)*(-pow((r1-r2),2)+d_2)
    
    if(det>0):
    	K=0.25*np.sqrt(det)
    	K1=0.5*(pow(r1,2)-pow(r2,2))/d_2
    	K2=2*K/d_2
    	flip_vec=np.flip(np.array(c2-c1),axis=0)
    	flip_vec[1]=-flip_vec[1]
    	pt1=0.5*(c1+c2)+(c2-c1)*K1+flip_vec*K2
    	pt2=0.5*(c1+c2)+(c2-c1)*K1-flip_vec*K2
        
    else:
        print("Circles do not intersect")
        return (-1,-1)
        
    elb_pt=get_elb_pt(phi_0,theta_0)
    dist_1=la.norm(elb_pt-pt1)
    dist_2=la.norm(elb_pt-pt2)
    if(dist_1<dist_2):
        opt_pt=pt1
    else:
        opt_pt=pt2

    fin_vec=np.zeros(2)
    fin_vec[0]=np.arctan(opt_pt[1]/opt_pt[0])
    diff=c2-opt_pt
    fin_vec[1]=np.arctan(diff[1]/diff[0])
    return (fin_vec[1]*180/np.pi,fin_vec[0]*180/np.pi)

def get_act_lengths(pot1,pot2):
    Pot1_Max=1729
    Pot1_Min=140
    Pot2_Max=1717
    Pot2_Min=73  
    L1=((pot1-Pot1_Min)/(Pot1_Max-Pot1_Min))*10+25
    L2=((pot2-Pot2_Min)/(Pot2_Max-Pot2_Min))*10+25
    return (L1,L2)

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
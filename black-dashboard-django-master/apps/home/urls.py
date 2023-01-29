# -*- encoding: utf-8 -*-
"""
Copyright (c) 2019 - present AppSeed.us
"""

from django.urls import path, re_path
from apps.home import views
from apps.home.views import IPCamView, ROSCamView, CompressedROSCamView

from django.conf.urls import url

urlpatterns = [

    # The home page
    path('', views.index, name='home'),

    #pilot page
    path('piloting/', views.piloting, name='piloting'),
    path('robotic_arm/', views.robotic_arm, name='robotic_arm'),
    url(r'^ros_nav_feed', views.ros_nav_feed, name='ros_nav_feed'),
    url(r'^ros_arm_feed', views.ros_arm_feed, name='ros_arm_feed'),
    url(r'^ros_side1_feed', views.ros_side1_feed, name='ros_side1_feed'),
    url(r'^ros_side2_feed', views.ros_side2_feed, name='ros_side2_feed'),
    url(r'^video_feed', views.video_feed, name='video_feed'),
    url(r'^panorama_feed', views.panorama_feed, name='panorama_feed'),
    url(r'^front_left_feed', IPCamView.as_view(ip='192.168.1.61',flip=0), name='front_left_feed'),
    url(r'^front_feed', IPCamView.as_view(ip='192.168.1.65',flip=0), name='front_feed'),
    # url(r'^front_feed', CompressedROSCamView.as_view(ros_topic='/mrt/camera3/image_compressed'), name='front_feed'),
    # url(r'^front_feed', ROSCamView.as_view(ros_topic='/mrt/camera3/image_raw'), name='front_feed'),
    url(r'^front_right_feed', IPCamView.as_view(ip='192.168.1.62',flip=0), name='front_right_feed'),
    url(r'^rear_left_feed', IPCamView.as_view(ip='192.168.1.63',flip=0), name='rear_left_feed'),
    # url(r'^front_down_feed', ROSCamView.as_view(ros_topic='/mrt/camera3/image_raw'), name='front_down_feed'),
    url(r'^front_down_feed', IPCamView.as_view(ip='192.168.1.64',flip=0), name='front_down_feed'),
    url(r'^rear_right_feed', IPCamView.as_view(ip='192.168.1.64',flip=0), name='rear_right_feed'),

    url(r'^arm_feed', IPCamView.as_view(ip='192.168.1.75',flip=0), name='arm_feed'),
    url(r'^arm_gripper_up', ROSCamView.as_view(ros_topic='/mrt/camera1/image_raw'), name='arm_gripper_up'),
    url(r'^arm_gripper_side', ROSCamView.as_view(ros_topic='/mrt/camera2/image_raw'), name='arm_gripper_side'),


    # url(r'^biocamera1', ROSCamView.as_view(ros_topic='/mrt/bio/cam1',flip=0), name='biocamera1'),
    # url(r'^biocamera2', ROSCamView.as_view(ros_topic='/mrt/bio/cam2',flip=0), name='biocamera2'),
    # url(r'^microscope', ROSCamView.as_view(ros_topic='/mrt/bio/microscope',flip=0), name='microscope'),
    # url(r'^biocamera3', ROSCamView.as_view(ros_topic='/mrt/bio/cam3',flip=0), name='biocamera1'),

    # Matches any html file
    re_path(r'^.*\.*', views.pages, name='pages'),

]

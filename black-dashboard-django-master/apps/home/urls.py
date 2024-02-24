# -*- encoding: utf-8 -*-
"""
Copyright (c) 2019 - present AppSeed.us
"""

from django.urls import path, re_path
from apps.home import views
from apps.home.views import IPCamView, ROSCamView, CompressedROSCamView, VideoCamView

from django.conf.urls import url

rtsp_template = "rtsp://admin:rover2409@192.168.69.{ip}:554/cam/realmonitor?channel=1&subtype=0&unicast=true&proto=Onvif"
rtsp_ips = [rtsp_template.format(ip) for ip in ["200", "202", "203"]]

urlpatterns = [

    # The home page
    path('', views.index, name='home'),

    #pilot page
    path('piloting/', views.piloting, name='piloting'),
    path('robotic_arm/', views.robotic_arm, name='robotic_arm'),
    url(r'^video_feed', views.video_feed, name='video_feed'),
    url(r'^panorama_feed', views.panorama_feed, name='panorama_feed'),
    # url(r'^front_feed', CompressedROSCamView.as_view(ros_topic='/mrt/camera3/image_compressed'), name='front_feed'),
    # url(r'^front_feed', ROSCamView.as_view(ros_topic='/mrt/camera3/image_raw'), name='front_feed'),
    # url(r'^front_down_feed', ROSCamView.as_view(ros_topic='/mrt/camera3/image_raw'), name='front_down_feed'),
    url(r'^arm_gripper_up', ROSCamView.as_view(ros_topic='/mrt/camera1/image_raw'), name='arm_gripper_up'),
    url(r'^arm_gripper_side', ROSCamView.as_view(ros_topic='/mrt/camera2/image_raw'), name='arm_gripper_side'),

    url(r'^front_down_feed', VideoCamView.as_view(dev=rtsp_ips[2]), name='front'),
    url(r'^right_feed', VideoCamView.as_view(dev=rtsp_ips[0]), name='right'),
    url(r'^left_feed', VideoCamView.as_view(dev=rtsp_ips[1]), name='left'),
    # 200 right
    # 202 left
    # 203 down


    # url(r'^biocamera1', ROSCamView.as_view(ros_topic='/mrt/bio/cam1',flip=0), name='biocamera1'),
    # url(r'^biocamera2', ROSCamView.as_view(ros_topic='/mrt/bio/cam2',flip=0), name='biocamera2'),
    # url(r'^microscope', ROSCamView.as_view(ros_topic='/mrt/bio/microscope',flip=0), name='microscope'),
    # url(r'^biocamera3', ROSCamView.as_view(ros_topic='/mrt/bio/cam3',flip=0), name='biocamera1'),

    # Matches any html file
    re_path(r'^.*\.*', views.pages, name='pages'),

]

# -*- encoding: utf-8 -*-
"""
Copyright (c) 2019 - present AppSeed.us
"""

from django.urls import path, re_path
from apps.home import views
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
    url(r'^front_feed', views.front_feed, name='front_feed'),
    url(r'^front_down_feed', views.front_down_feed, name='front_down_feed'),
    url(r'^rear_left_feed', views.rear_left_feed, name='rear_left_feed'),
    url(r'^rear_right_feed', views.rear_right_feed, name='rear_right_feed'),
    url(r'^bio_cam1_feed'.views.bio_cam1_feed,name='bio_cam1_feed')
    url(r'^bio_cam2_feed'.views.bio_cam2_feed,name='bio_cam2_feed')
    url(r'^bio_cam3_feed'.views.bio_cam3_feed,name='bio_cam3_feed')
    url(r'^bio_cam4_feed'.views.bio_cam4_feed,name='bio_cam4_feed')
    # Matches any html file
    re_path(r'^.*\.*', views.pages, name='pages'),

]

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
    url(r'^ros_feed', views.ros_feed, name='ros_feed'),
    url(r'^ros_side1_feed', views.ros_side1_feed, name='ros_side1_feed'),
    url(r'^ros_side2_feed', views.ros_side2_feed, name='ros_side2_feed'),
    url(r'^video_feed', views.video_feed, name='video_feed'),
    url(r'^webcam_feed', views.webcam_feed, name='webcam_feed'),

    # Matches any html file
    re_path(r'^.*\.*', views.pages, name='pages'),

]

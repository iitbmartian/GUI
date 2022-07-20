"""mysite URL Configuration

The `urlpatterns` list routes URLs to views. For more information please see:
    https://docs.djangoproject.com/en/1.11/topics/http/urls/
Examples:
Function views
    1. Add an import:  from my_app import views
    2. Add a URL to urlpatterns:  url(r'^$', views.home, name='home')
Class-based views
    1. Add an import:  from other_app.views import Home
    2. Add a URL to urlpatterns:  url(r'^$', Home.as_view(), name='home')
Including another URLconf
    1. Import the include() function: from django.conf.urls import url, include
    2. Add a URL to urlpatterns:  url(r'^blog/', include('blog.urls'))
"""
from django.conf.urls import url
# from django.urls import path, include #for > Django 2.0
from django.contrib import admin
from mysite.views import index, bio, video_feed, webcam_feed


urlpatterns = [
    url(r'^$', bio, name='Welcome'),
    url(r'^admin/', admin.site.urls),
    url(r'^index/',index, name='index'),
    url(r'^bio/',bio, name='bio'),
    url(r'^video_feed', video_feed, name='video_feed'),
    url(r'^webcam_feed', webcam_feed, name='webcam_feed')
]

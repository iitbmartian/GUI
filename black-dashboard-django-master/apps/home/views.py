# -*- encoding: utf-8 -*-
"""
Copyright (c) 2019 - present AppSeed.us
"""

from django import template
from django.contrib.auth.decorators import login_required
from django.http import HttpResponse, HttpResponseRedirect, HttpRequest
from django.template import loader
from django.urls import reverse
from django.views.decorators.csrf import csrf_exempt


from django.shortcuts import render, render_to_response
from manage import send_joy#, send_channeling, send_microscope, send_stewart, send_raman #, _actuator_angle, _stewart_data
from django.core.exceptions import ValidationError
import sys
import pdb
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray


@login_required(login_url="/login/")
def index(request):
    context = {'segment': 'index'}

    html_template = loader.get_template('home/index.html')
    return HttpResponse(html_template.render(context, request))

@csrf_exempt
@login_required(login_url="/login/")
def pages(request):
    context = {}
    # All resource paths end in .html.
    # Pick out the html file name from the url. And load that template.
    try:

        load_template = request.path.split('/')[-1]

        if load_template == 'admin':
            return HttpResponseRedirect(reverse('admin:index'))
        context['segment'] = load_template

        if load_template == 'piloting':
            return piloting(request)

        html_template = loader.get_template('home/' + load_template)
        return HttpResponse(html_template.render(context, request))

    except template.TemplateDoesNotExist:

        html_template = loader.get_template('home/page-404.html')
        return HttpResponse(html_template.render(context, request))

    except Exception as e:
        print(e)
        html_template = loader.get_template('home/page-500.html')
        return HttpResponse(html_template.render(context, request))


@csrf_exempt
def piloting(request):
    if request.method == 'GET':
        return render(request, 'piloting.html', {'output': ''})
    elif request.method == 'POST':
        # direction=request.POST['action']
        # pdb.set_trace()
        # send_msg(direction)
        joy_arr=np.zeros(4)
        but_arr=np.zeros(10)
        if 'dataX_1' in request.POST:
            dataX_1 = -float(request.POST['dataX_1'])/50
            print("Data X_1 : " + str(dataX_1))
            joy_arr[0]=dataX_1
        if 'dataY_1' in request.POST:
            dataY_1= -float(request.POST['dataY_1'])/50
            print("   Data Y_1: " + str(dataY_1))
            joy_arr[1]=dataY_1
        if 'dataX_2' in request.POST:
            dataX_2 = -float(request.POST['dataX_2'])/50
            print("Data X_2 : " + str(dataX_2))
            joy_arr[2]=dataX_2
        if 'dataY_2' in request.POST:
            dataY_2= -float(request.POST['dataY_2'])/50
            print("   Data Y_2: " + str(dataY_2))
            joy_arr[3]=dataY_2
        if 'action' in request.POST:#for buttons add here
            if request.POST['action'] == 'Up':
              print("Up: Data Y_1 : " + str(1))
              joy_arr[1]=float(1)
            elif request.POST['action'] == 'Down':
              print("Down: Data Y_1 : " + str(-1))
              joy_arr[1]=float(-1)
            elif request.POST['action'] == 'Right':
              print("Right: Data X_1 : " + str(-1))
              joy_arr[0]=float(-1)
            elif request.POST['action'] == 'Left':
              print("Left: Data X_1 : " + str(1))
              joy_arr[0]=float(1)
            elif request.POST['action'] == 'Stop_All':
              print("Stop all")
            elif str(request.POST['action'])[:2] == 'Ac':
              print (str(request.POST['action']))
              but_arr[int(str(request.POST['action'])[2])*2 + int(str(request.POST['action'])[4:6] == 'Do') -2 ] = 1
            elif str(request.POST['action'])[:2] == 'Gr':
              print (str(request.POST['action']))
              but_arr[8 + int(str(request.POST['action'])[5:7] == 'Do') ] = 1

        send_joy(axes = joy_arr, buttons = but_arr)
        if 'action' in request.POST:
          if request.POST['action'] != 'Stop_All':
            rospy.sleep(0.2)#change time duration here
            send_joy(axes = np.zeros(4), buttons = np.zeros(10))
            print( str(request.POST['action']) + ': 0')
        # print(request.POST)
        return render(request,'piloting.html',{'output': "Success"})


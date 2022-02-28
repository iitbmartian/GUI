from django.shortcuts import render, render_to_response
from django.views.decorators.csrf import csrf_exempt
from manage import send_joy
from django.core.exceptions import ValidationError
import sys
import pdb
import numpy as np
import rospy
sys.path.append('/home/kavinubuntu/Desktop/GUI/Django_GUI')

@csrf_exempt
def index(request):
    if request.method == 'GET':
        return render(request, 'index.html', {'output': ''})
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
        return render(request,'index.html',{'output': "Success"})

@csrf_exempt
def bio(request):
    if request.method == 'GET':
        return render(request, 'bio.html', {'output': ''})
    elif request.method == 'POST':
        # direction=request.POST['action']
        # pdb.set_trace()
        # send_msg(direction)
        # joy_arr=np.zeros(4)
        but_arr=np.zeros(10)
        if 'channeling' in request.POST:#for buttons add here
            if request.POST['channeling'] == 'Stop_All':
              print("Stop all")
            elif str(request.POST['channeling']) == 'funnel':
              print(str(request.POST['channeling']))
              but_arr[0] = but_arr[0]
            elif str(request.POST['channeling'])[:2] == 'Gr':
              print (str(request.POST['channeling']))
              but_arr[8 + int(str(request.POST['channeling'])[5:7] == 'Do') ] = 1
        print(request.POST)
        # send_joy(buttons = but_arr)
        if 'action' in request.POST:
          if request.POST['action'] != 'Stop_All':
            rospy.sleep(0.2)#change time duration here
            # send_joy(axes = np.zeros(4), buttons = np.zeros(10))
            print( str(request.POST['action']) + ': 0')
        # print(request.POST)
        return render(request,'bio.html',{'output1': "Success", "servo_angle":"45"})


# def my_view(request):
#     if request.method == 'POST':
#         if 'dataX' in request.POST:
#             dataX = request.POST['dataX']
#             print(dataX)
#             # doSomething with pieFact here...
#             # return HttpResponse('success') # if everything is OK
#     # nothing went well
#     # return HttpRepsonse('FAIL!!!!!')

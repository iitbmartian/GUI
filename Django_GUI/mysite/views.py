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
        raman_arr = np.zeros(2)
        channel_arr = np.zeros(7)
        microscope_arr = np.zeros(2)
        stewart_arr = np.zeros(9)
        req = request.POST
        print(req)
        if 'channeling' in req:#for buttons add here
            if req['channeling'] == 'Stop_All':
              print("Stop all")
            elif str(req['channeling']) == 'funnel':
              print(str(req['channeling']))
              channel_arr[0] = 1
            elif str(req['channeling'])[:12] == 'push buttons':
              print (str(req['channeling']))
              channel_arr[int(str(req['channeling'])[12:13]) ] = 1
        if 'raman' in req:
            if str(req['raman']) == 'laser':
              print(str(req['raman']))
              raman_arr[0] = 1
            elif str(req['raman']) == 'servo':
              print(str(req['raman']))
              raman_arr[1] = 1
            send_raman(raman_arr)
            print( str(req['raman']) + ': 1')
        if 'microscope' in req:
            if 'dummy' in req.getlist('microscope'):
              print(str(req['microscope']))
              microscope_arr[1] = int(req['microscope'])
            else:
              print("val", req.getlist('microscope')[0])
            microscope_arr[0] = int(req.getlist('microscope')[0])
            send_microscope(microscope_arr)
        if 'stewart' in req:
            req_list = [int(x.encode('UTF8')) for x in req.getlist('stewart')[1:]]
            random = 0
            if not 'dummy' in req.getlist('stewart'):#random is the only visible button
                random = 1

            req_list.append(random)
            stewart_arr = req_list
            print(stewart_arr)
            send_stewart(stewart_arr)
            
            # elif str(req['raman']) == 'servo':
            #   print(str(req['raman']))
            #   raman_arr[1] = 1
            # send_raman(raman_arr)
            # print( str(req['raman']) + ': 1')


        print(req)
        # get_arr
        # send_joy(buttons = but_arr)
        if 'action' in req:
          if req['action'] != 'Stop_All':
            rospy.sleep(0.2)#change time duration here
            # send_joy(axes = np.zeros(4), buttons = np.zeros(10))
            print( str(req['action']) + ': 0')
        # print(req)
        return render(request,'bio.html',{"servo_angle":"45"})


# def my_view(request):
#     if request.method == 'POST':
#         if 'dataX' in request.POST:
#             dataX = request.POST['dataX']
#             print(dataX)
#             # doSomething with pieFact here...
#             # return HttpResponse('success') # if everything is OK
#     # nothing went well
#     # return HttpRepsonse('FAIL!!!!!')

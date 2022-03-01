from django.shortcuts import render, render_to_response
from django.http import HttpRequest
from django.views.decorators.csrf import csrf_exempt
from manage import send_joy, send_channeling, send_microscope, send_stewart, send_raman#, _actuator_angle, _stewart_data
from django.core.exceptions import ValidationError
import sys
import pdb
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
sys.path.append('/home/kavinubuntu/Desktop/GUI/Django_GUI')
_output = ['','','','']
# rospy.init_node('listener')#, anonymous=True
from manage import node
def microscope_callback(msg):
    global _actuator_angle
    _actuator_angle = msg.data[0]

def stewart_callback(msg):
    global _stewart_data
    _stewart_data = msg.data
    
global _actuator_angle
_actuator_angle = 0
microscope_sub = rospy.Subscriber('actuator', Float32MultiArray, microscope_callback)

global _stewart_data# 1,...6, vertical motion, roll
_stewart_data = [0,0,0,0,0,0,0,0]
microscope_sub = rospy.Subscriber('stewart_text', Float32MultiArray, stewart_callback)

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
        return render(request, 'bio.html', {"servo_angle":_actuator_angle, "val1":_stewart_data[0],"val2":_stewart_data[1],"val3":_stewart_data[2],"val4":_stewart_data[3],"val5":_stewart_data[4],"val6":_stewart_data[5],"vertical_motion":_stewart_data[6], "roll":_stewart_data[7]})
    elif request.method == 'POST':
        # direction=request.POST['action']
        # pdb.set_trace()
        # send_msg(direction)
        # joy_arr=np.zeros(4)
        raman_arr = np.zeros(2)
        channel_arr = np.zeros(7)
        microscope_arr = np.zeros(2)
        stewart_arr = np.zeros(9)
        # output = ['','','','']
        req = request.POST
        # print(req)
        global _actuator_angle, _stewart_data, _output
        if 'funnel' in req:#on-off buttons
            data = {k:int(str(v) == 'true') for k,v in req.dict().items()}
            channel_arr[0] = data['funnel']
            raman_arr[0] = data['laser']
            raman_arr[1] = data['servo']
            actuator = 0
            for i in ['+1','0','-1']:
                if data[i] == 1:
                    actuator = int(i)
            microscope_arr[0] = actuator
            stewart_arr[-1] = data['random']

        # print(_data)
        if 'channeling' in req:
            # if req['channeling'] == 'Stop_All':
            #   print("Stop all")
            if 'funnel' in str(req.getlist('channeling')):
              print(str(req['channeling']))
              channel_arr[0] = 1
            if str(req['channeling'])[:11] == 'push button':
              print (str(req.getlist('channeling')))
              channel_arr[int(str(req['channeling'])[12:13]) ] = 1
              _output[0] = str(_output[0])+'You pressed push button ' + str(req['channeling'])[12:13]
        send_channeling(channel_arr)
        if 'raman' in req:
            if 'laser' in str(req.getlist('raman')):
              print(str(req['raman']))
              raman_arr[0] = 1
            elif 'servo' in str(req.getlist('raman')):
              print(str(req['raman']))
              raman_arr[1] = 1
        send_raman(raman_arr)
        #     print( str(req['raman']) + ': 1')
        if 'microscope' in req:
            if 'dummy' in req.getlist('microscope'):
              print(str(req['microscope']))
              microscope_arr[1] = int('0'+str(req['microscope']))
            else:
              print("val", req.getlist('microscope'))
            microscope_arr[0] = int(str(req.getlist('microscope')[1])+'0')/10
            print(str(req.getlist('microscope')[1]))
        send_microscope(microscope_arr)
        if 'stewart' in req:
            req_list = [int('0'+x.encode('UTF8')) for x in req.getlist('stewart')[1:9]]
            # random = 0
            # if not 'dummy' in req.getlist('stewart'):#random is the only visible button
            #     random = 1

            # req_list.append(random)
            print(stewart_arr)
            stewart_arr[:-1] = req_list
            print(stewart_arr)
        send_stewart(stewart_arr)
            
            # elif str(req['raman']) == 'servo':
            #   print(str(req['raman']))
            #   raman_arr[1] = 1
            # send_raman(raman_arr)
            # print( str(req['raman']) + ': 1')

        print(req)#bool(channel_arr[0])"funnel":"0", "laser":bool(raman_arr[0]), "servo":bool(raman_arr[1]), "plus1":microscope_arr[0]==1, "zero": microscope_arr[0]==0,"minus1":microscope_arr[0]==-1,
        return render(request, 'bio.html', {"servo_angle":_actuator_angle, "val1":_stewart_data[0],"val2":_stewart_data[1],"val3":_stewart_data[2],"val4":_stewart_data[3],"val5":_stewart_data[4],"val6":_stewart_data[5],"vertical_motion":_stewart_data[6], "roll":_stewart_data[7]})


# def my_view(request):
#     if request.method == 'POST':
#         if 'dataX' in request.POST:
#             dataX = request.POST['dataX']
#             print(dataX)
#             # doSomething with pieFact here...
#             # return HttpResponse('success') # if everything is OK
#     # nothing went well
#     # return HttpRepsonse('FAIL!!!!!')

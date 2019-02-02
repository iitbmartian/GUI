from django.shortcuts import render, render_to_response
from django.views.decorators.csrf import csrf_exempt
from manage import send_msg, send_joy
from django.core.exceptions import ValidationError
import sys
import pdb
import numpy as np
sys.path.append('/home/kavinubuntu/Desktop/GUI/Django_GUI')

@csrf_exempt
def index(request):
    if request.method == 'GET':
        return render(request, 'index.html', {'output': ''})
    elif request.method == 'POST':
        # direction=request.POST['action']
        # pdb.set_trace()
        # send_msg(direction)
        msg_arr=np.zeros(4)
        if 'dataX_1' in request.POST:
            dataX_1 = request.POST['dataX_1']
            print("Data X_1 : " + str(dataX_1))
            msg_arr[0]=float(dataX_1)
        if 'dataY_1' in request.POST:
            dataY_1=request.POST['dataY_1']
            print("   Data Y_1: " + str(dataY_1))
            msg_arr[1]=float(dataY_1)
        if 'dataX_2' in request.POST:
            dataX_2 = request.POST['dataX_2']
            print("Data X_2 : " + str(dataX_2))
            msg_arr[2]=float(dataX_2)
        if 'dataY_2' in request.POST:
            dataY_2=request.POST['dataY_2']
            print("   Data Y_2: " + str(dataY_2))
            msg_arr[3]=float(dataY_2)

        send_joy(msg_arr)
        return render(request,'index.html',{'output': "Success"})


# def my_view(request):
#     if request.method == 'POST':
#         if 'dataX' in request.POST:
#             dataX = request.POST['dataX']
#             print(dataX)
#             # doSomething with pieFact here...
#             # return HttpResponse('success') # if everything is OK
#     # nothing went well
#     # return HttpRepsonse('FAIL!!!!!')
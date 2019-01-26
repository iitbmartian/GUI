from django.shortcuts import render, render_to_response
from django.views.decorators.csrf import csrf_exempt
from manage import send_msg,send_joy
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
        mssg_arr=np.zeros(2)
        if 'dataX' in request.POST:
            dataX = request.POST['dataX']
            print("Data X: " + str(dataX))
            mssg_arr[0]=float(dataX)
        if 'dataY' in request.POST:
            dataY=request.POST['dataY']
            print("   Data Y: " + str(dataY))
            mssg_arr[1]=float(dataY)

        send_joy(mssg_arr)
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
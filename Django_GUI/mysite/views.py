from django.shortcuts import render, render_to_response
from django.views.decorators.csrf import csrf_exempt
from manage import send_msg
from django.core.exceptions import ValidationError

import sys
sys.path.append('/home/rover-nuc/GUI')
# from django.template import RequestContext
@csrf_exempt
def index(request):
    if request.method == 'GET':
        return render(request, 'index.html', {'output': ''})
    elif request.method == 'POST':
        # print("Value Rxed")
        # raise ValidationError(request.POST)
        direction=request.POST['action']
        print(direction)
        send_msg(direction)
        # execute_file("handler")
        return render(request,'index.html',{'output': "Success"})
        # return render(request, 'output.html', )

        
from django.shortcuts import render, render_to_response
from django.http import HttpRequest
from django.http.response import StreamingHttpResponse
from django.views.decorators.csrf import csrf_exempt
from manage import send_joy, send_channeling, send_microscope, send_stewart, send_raman, send_biosensor
from django.core.exceptions import ValidationError
import sys
import pdb
import numpy as np
import rospy
import cv2
from std_msgs.msg import Float32MultiArray
sys.path.append('/home/kavinubuntu/Desktop/GUI/Django_GUI')
_output = ['','','','']
# rospy.init_node('listener')#, anonymous=True
from manage import node

def bio_sensor_callback(msg):
    global _bio_sensor_data
    array = msg.data 
    _bio_sensor_data = ["%.1f" % array[0],"%.1f" % array[1],"%.1f" % array[2]]


global _bio_sensor_data# temp:28-30 celsius; methane(RO):20:25; humidity: 15%;  
_bio_sensor_data = [29.0,21.5,15.0]
microscope_sub = rospy.Subscriber('biosensor', Float32MultiArray, bio_sensor_callback)

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
        return render(request, 'bio.html', {'humidity':_bio_sensor_data[2],'methane':_bio_sensor_data[1],'temp': _bio_sensor_data[0]})
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
        global _bio_sensor_data, _output
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
            if 'funnel' in req.getlist('channeling'):
              print(str(req.getlist('channeling')))
              channel_arr[0] = 1
            buttons = {'water pump funnel':1, 'water pump microscope':2, 'sample 1 raman':3, 'sample 2 raman':4, 'sample 1 microscope':5, 'sample 2 microscope':6}
            if str(req['channeling']) in buttons:
              print (str(req.getlist('channeling')))
              channel_arr[buttons[str(req['channeling'])] ] = 1
            # if str(req['channeling'])[:11] == 'push button':
            #   print (str(req.getlist('channeling')))
            #   channel_arr[int(str(req['channeling'])[12:13]) ] = 1
              # _output[0] = str(_output[0])+'You pressed push button ' + str(req['channeling'])[12:13]
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
        if 'biosensor' in req:
            send_biosensor(_bio_sensor_data)
        print(req)#bool(channel_arr[0])"funnel":"0", "laser":bool(raman_arr[0]), "servo":bool(raman_arr[1]), "plus1":microscope_arr[0]==1, "zero": microscope_arr[0]==0,"minus1":microscope_arr[0]==-1,
        return render(request, 'bio.html', {'humidity':_bio_sensor_data[2],'methane':_bio_sensor_data[1],'temp': _bio_sensor_data[0]})

#class for webcam/modify for each case
class VideoCamera(object):
    def __init__(self):
        self.video = cv2.VideoCapture(0)

    def __del__(self):
        self.video.release()

    def get_frame(self):
        success, image = self.video.read()
        # We are using Motion JPEG, but OpenCV defaults to capture raw images,
        # so we must encode it into JPEG in order to correctly display the
        # video stream.
        frame_flip = cv2.flip(image,1)
        ret, jpeg = cv2.imencode('.jpg', frame_flip)
        return jpeg.tobytes()

class IPWebCam(object):#TODO check; IP - Internet Protocol
    def __init__(self):
        self.url = "http://192.168.2.103:8080/shot.jpg"

    def __del__(self):
        cv2.destroyAllWindows()

    def get_frame(self):
        imgResp = urllib.request.urlopen(self.url)
        imgNp = np.array(bytearray(imgResp.read()),dtype=np.uint8)
        img= cv2.imdecode(imgNp,-1)
        # We are using Motion JPEG, but OpenCV defaults to capture raw images,
        # so we must encode it into JPEG in order to correctly display the
        # video stream
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces_detected = face_detection_webcam.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)
        for (x, y, w, h) in faces_detected:
            cv2.rectangle(img, pt1=(x, y), pt2=(x + w, y + h), color=(255, 0, 0), thickness=2)
        resize = cv2.resize(img, (640, 480), interpolation = cv2.INTER_LINEAR) 
        frame_flip = cv2.flip(resize,1)
        ret, jpeg = cv2.imencode('.jpg', frame_flip)
        return jpeg.tobytes()

def gen(camera):
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')


def video_feed(request):
    return StreamingHttpResponse(gen(VideoCamera()),
                    content_type='multipart/x-mixed-replace; boundary=frame')


def webcam_feed(request):
    return StreamingHttpResponse(gen(IPWebCam()),
                    content_type='multipart/x-mixed-replace; boundary=frame')

# def my_view(request):
#     if request.method == 'POST':
#         if 'dataX' in request.POST:
#             dataX = request.POST['dataX']
#             print(dataX)
#             # doSomething with pieFact here...
#             # return HttpResponse('success') # if everything is OK
#     # nothing went well
#     # return HttpRepsonse('FAIL!!!!!')

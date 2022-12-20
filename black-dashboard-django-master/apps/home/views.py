# -*- encoding: utf-8 -*-
"""
Copyright (c) 2019 - present AppSeed.us
"""

from django import template
from django.contrib.auth.decorators import login_required
from django.http import HttpResponse, HttpResponseRedirect, HttpRequest
from django.http.response import StreamingHttpResponse
from django.template import loader
from django.urls import reverse
from django.views.decorators.csrf import csrf_exempt


from django.shortcuts import render
from manage import send_joy#, send_channeling, send_microscope, send_stewart, send_raman #, _actuator_angle, _stewart_data
from django.core.exceptions import ValidationError
import sys
import pdb
import numpy as np
import rospy, cv2
import urllib
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
arm_ros_topic = 'camera_image/image_raw'
nav_ros_topic = 'camera/image_raw'
ros_side_topic1 = 'zed2/left/image_rect_color'
ros_side_topic2 = 'zed2/right/image_rect_color'
front_topic = "/mrt/camera/color/image_raw"
front_down_ip = "http://192.168.2.89:8080/shot.jpg?1"#"http://192.168.2.9:8080/video"
rear_right_topic = "/mrt/camera1/image_compressed"
rear_left_topic = "/mrt/camera2/image_compressed"

def bio_sensor_callback(msg):
    global _bio_sensor_data
    array = msg.data
    _bio_sensor_data = ["%.1f" % array[0],"%.1f" % array[1],"%.1f" % array[2]]


# global _bio_sensor_data# temp:28-30 celsius; methane(RO):20:25; humidity: 15%;
microscope_pub = rospy.Publisher('microscope', Float32MultiArray, queue_size=10)
collection_pub = rospy.Publisher('collection', Float32MultiArray, queue_size=10)
_bio_sensor_data = [29.0,21.5,15.0]
# microscope_sub = rospy.Subscriber('biosensor', Float32MultiArray, bio_sensor_callback)

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
        # print(load_template)
        if 'piloting' in load_template:
            print("Pilot page rendering")
            return piloting(request)
        if 'robotic' in load_template:
            print("Robotic Arm page rendering")
            return robotic_arm(request)
        if 'bio' in load_template:
            print("Bio page rendering")
            return bio(request)

        html_template = loader.get_template('home/' + load_template)
        return HttpResponse(html_template.render(context, request))

    except template.TemplateDoesNotExist:

        html_template = loader.get_template('home/page-404.html')
        return HttpResponse(html_template.render(context, request))

    except Exception as e:
        print(e)
        html_template = loader.get_template('home/page-500.html')
        return HttpResponse(html_template.render(context, request))

#for robotic arm: https://github.com/ros-planning/moveit/blob/master/moveit_ros/visualization/src/moveit_ros_visualization/moveitjoy_module.py#L449
# https://github.com/ros-planning/panda_moveit_config/blob/melodic-devel/launch/joystick_control.launch
# https://ros-planning.github.io/moveit_tutorials/doc/joystick_control_teleoperation/joystick_control_teleoperation_tutorial.html?highlight=joystick

# @login_required(login_url="/login/")
@csrf_exempt
def piloting(request):
    print(request)
    if request.method == 'GET':
        print("get")
        html_template = loader.get_template('home/piloting.html')
        return HttpResponse(html_template.render({'output': ''}, request))
        # return render(request, 'piloting.html', {'output': ''})
    elif request.method == 'POST':
        # direction=request.POST['action']
        # pdb.set_trace()
        # send_msg(direction)
        joy_arr=np.zeros(4)
        but_arr=np.zeros(10)
        but_arr[0] = 1 ## for joystick node that requires button pushed
        if 'dataX_1' in request.POST:
            dataX_1 = float(request.POST['dataX_1'])/50
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

        send_joy(axes = list(joy_arr), buttons = list(but_arr.astype(np.uint32)))
        # print("sending joy")
        if 'action' in request.POST:
          if request.POST['action'] != 'Stop_All':
            rospy.sleep(0.2)#change time duration here
            send_joy(axes = np.zeros(4), buttons = np.zeros(10))
            print( str(request.POST['action']) + ': 0')
        # print(request.POST)

        html_template = loader.get_template('home/piloting.html')
        return HttpResponse(html_template.render({'output': 'Success'}, request))
        # return render(request,'piloting.html',{'output': "Success"})

@csrf_exempt
def robotic_arm(request):
    print(request.method)
    if request.method == 'GET':
        print("get")
        html_template = loader.get_template('home/robotic_arm.html')
        return HttpResponse(html_template.render({'output': ''}, request))
        # return render(request, 'piloting.html', {'output': ''})
    elif request.method == 'POST':
        # direction=request.POST['action']
        # pdb.set_trace()
        # send_msg(direction)
        joy_arr=np.zeros(6)
        but_arr=np.zeros(10)
        but_arr[0] = 1 ## for joystick node that requires button pushed
        if 'dataX_1' in request.POST:
            dataX_1 = -float(request.POST['dataX_1'])/50
            print("Data X_1 : " + str(dataX_1))
            joy_arr[0]=dataX_1
        if 'dataY_1' in request.POST:
            dataY_1= float(request.POST['dataY_1'])/50
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
        if 'dataX_3' in request.POST:
            dataX_3 = -float(request.POST['dataX_3'])/50
            print("Data X_3 : " + str(dataX_3))
            joy_arr[4]=dataX_3
        if 'dataY_3' in request.POST:
            dataY_3= -float(request.POST['dataY_3'])/50
            print("   Data Y_3: " + str(dataY_3))
            joy_arr[5]=dataY_3

        if 'action' in request.POST:#for buttons add here
            if request.POST['action'] == 'Plan':
              print("Planning")
              but_arr[1]=1
            elif request.POST['action'] == 'Execute':
              print("Executing")
              but_arr[2]=1
            elif request.POST['action'] == 'Change Planning group':
              print("Changing Planning group")
              but_arr[3]=1
            elif request.POST['action'] == 'Stop_All':
              print("Stop all")

        send_joy(axes = list(joy_arr), buttons = list(but_arr.astype(np.uint32)))
        print("sending joy")
        if 'action' in request.POST:
          if request.POST['action'] != 'Stop_All':
            rospy.sleep(0.2)#change time duration here
            buttons = list(np.zeros(10, dtype=np.uint8))
            buttons[0] = 1
            send_joy(axes = list(np.zeros(6)), buttons = buttons)
            print( str(request.POST['action']) + ': 0')
        # print(request.POST)

        html_template = loader.get_template('home/robotic_arm.html')
        return HttpResponse(html_template.render({'output': 'Success'}, request))
        # return render(request,'piloting.html',{'output': "Success"})


@csrf_exempt
def bio(request):
    global _bio_sensor_data, _output, microscope_pub, collection_pub
    if request.method == 'GET':
        return render(request, 'home/bio-new.html', {'temp': _bio_sensor_data[0]})
    elif request.method == 'POST':
        # direction=request.POST['action']
        # pdb.set_trace()
        # send_msg(direction)
        # joy_arr=np.zeros(4)

        # raman_arr = np.zeros(2)
        # channel_arr = np.zeros(7)
        # stewart_arr = np.zeros(9)

        # output = ['','','','']
        req = request.POST
        print(req)
        collection_arr = np.zeros(3)
        microscope_arr = np.zeros(3)

        if 'collection' in req:
            buttons = {"gripper":0}

            req_list = [int('0{}'.format(x).encode('UTF8')) for x in req.getlist('collection')[1:3]]
            print("relist:", req_list)

            if 'dummy' in req.getlist('collection'):
              print(str(req['collection']))
              collection_arr[1:] = req_list
            else:
              print("val", req.getlist('collection'))

            if str(req.getlist('collection')[0]) in buttons:
              print (str(req.getlist('collection')))
              collection_arr[buttons[str(req.getlist('collection')[0])] ] = 1
            collection_pub.publish(Float32MultiArray(data=collection_arr))

        if 'microscope' in req:
            buttons = {"water_valve":1, "chem_valve":2}
            if 'dummy' in req.getlist('microscope'):
              print(str(req['microscope']))
              microscope_arr[0] = int('0'+str(req['microscope']))
            else:
              print("val", req.getlist('microscope'))
            print(str(req.getlist('microscope')[1]))
            for val in req.getlist('microscope'):
                if str(val) in buttons:
                    print (str(req.getlist('microscope')))
                    microscope_arr[buttons[str(val)] ] = 1
            microscope_pub.publish(Float32MultiArray(data=microscope_arr))
        # send_microscope(microscope_arr)

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
        # send_channeling(channel_arr)
        if 'raman' in req:
            if 'laser' in str(req.getlist('raman')):
              print(str(req['raman']))
              raman_arr[0] = 1
            elif 'servo' in str(req.getlist('raman')):
              print(str(req['raman']))
              raman_arr[1] = 1
        # send_raman(raman_arr)
        #     print( str(req['raman']) + ': 1')

        if 'stewart' in req:
            req_list = [int('0'+x.encode('UTF8')) for x in req.getlist('stewart')[1:9]]
            # random = 0
            # if not 'dummy' in req.getlist('stewart'):#random is the only visible button
            #     random = 1

            # req_list.append(random)
            print(stewart_arr)
            stewart_arr[:-1] = req_list
            print(stewart_arr)
        # send_stewart(stewart_arr)

            # elif str(req['raman']) == 'servo':
            #   print(str(req['raman']))
            #   raman_arr[1] = 1
            # send_raman(raman_arr)
            # print( str(req['raman']) + ': 1')

        # if 'biosensor' in req:
        #     send_biosensor(_bio_sensor_data)
        print(req)#bool(channel_arr[0])"funnel":"0", "laser":bool(raman_arr[0]), "servo":bool(raman_arr[1]), "plus1":microscope_arr[0]==1, "zero": microscope_arr[0]==0,"minus1":microscope_arr[0]==-1,
        return render(request, 'home/bio-new.html', {'humidity':_bio_sensor_data[2],'methane':_bio_sensor_data[1],'temp': _bio_sensor_data[0]})

#class for webcam/modify for each case
#class for webcam/modify for each case
class VideoCamera(object):
    def __init__(self):
        self.video = cv2.VideoCapture(0)

    def __del__(self):
        self.video.release()

    def get_frame(self):
        success, image = self.video.read()
        image = cv2.resize(image,[720,540])
        # We are using Motion JPEG, but OpenCV defaults to capture raw images,
        # so we must encode it into JPEG in order to correctly display the
        # video stream.
        frame_flip = cv2.flip(image,1)
        ret, jpeg = cv2.imencode('.jpg', frame_flip)
        return jpeg.tobytes()

class IPWebCam(object):#TODO check; IP - Internet Protocol; check web_video_server for rostopics
    def __init__(self,ip_address):
        self.url = ip_address#"http://192.168.2.103:8080/shot.jpg"

    def __del__(self):
        cv2.destroyAllWindows()

    def get_frame(self):
        imgResp = urllib.request.urlopen(self.url)
        imgNp = np.array(bytearray(imgResp.read()),dtype=np.uint8)
        img= cv2.imdecode(imgNp,-1)
        # We are using Motion JPEG, but OpenCV defaults to capture raw images,
        # so we must encode it into JPEG in order to correctly display the
        # video stream
        resize = cv2.resize(img, (640, 480), interpolation = cv2.INTER_LINEAR)
        frame_flip = cv2.flip(resize,1)
        ret, jpeg = cv2.imencode('.jpg', frame_flip)
        return jpeg.tobytes()

class RosCamera(object):
    def __init__(self, topic):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, Image, self.cam_callback)
        rospy.wait_for_message(topic, Image, timeout=15)

    # def __del__(self):
    #     self.video.release()

    def cam_callback(self, ros_image):
        # Convert ROS Image message to OpenCV image
        self.frame = self.bridge.imgmsg_to_cv2(ros_image)


    def get_frame(self):
        # We are using Motion JPEG, but OpenCV defaults to capture raw images,
        # so we must encode it into JPEG in order to correctly display the
        # video stream.
        image = cv2.resize(self.frame,(720,540))
        frame_flip = cv2.flip(image,1)
        ret, jpeg = cv2.imencode('.jpg', frame_flip)
        return jpeg.tobytes()

class CompressedRosCamera(object):
    def __init__(self, topic):
        self.sub = rospy.Subscriber(topic, CompressedImage, self.cam_callback)
        rospy.wait_for_message(topic, CompressedImage, timeout=15)

    # def __del__(self):
    #     self.video.release()

    def cam_callback(self, ros_image):
        # Convert ROS Image message to OpenCV image
        self.frame = ros_image.data


    def get_frame(self):
        # We are using Motion JPEG, but OpenCV defaults to capture raw images,
        # so we must encode it into JPEG in order to correctly display the
        # video stream.
        # image = cv2.resize(self.frame,(256,144))
        # frame_flip = cv2.flip(image,1)
        # ret, jpeg = cv2.imencode('.jpg', frame_flip)
        # return jpeg.tobytes()
        return self.frame

def gen(camera):
    rate = rospy.Rate(10)
    while True:
        rate.sleep()
        frame = camera.get_frame()
        yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')


def video_feed(request):
    return StreamingHttpResponse(gen(VideoCamera()),
                    content_type='multipart/x-mixed-replace; boundary=frame')

def ros_arm_feed(request):
    global arm_ros_topic
    return StreamingHttpResponse(gen(RosCamera(arm_ros_topic)),
                    content_type='multipart/x-mixed-replace; boundary=frame')
def ros_nav_feed(request):
    global nav_ros_topic
    return StreamingHttpResponse(gen(RosCamera(nav_ros_topic)),
                    content_type='multipart/x-mixed-replace; boundary=frame')
def ros_side1_feed(request):
    global ros_side_topic1
    return StreamingHttpResponse(gen(RosCamera(ros_side_topic1)),
                    content_type='multipart/x-mixed-replace; boundary=frame')
def ros_side2_feed(request):
    global ros_side_topic2
    return StreamingHttpResponse(gen(RosCamera(ros_side_topic2)),
                    content_type='multipart/x-mixed-replace; boundary=frame')

def webcam_feed(request):
    return StreamingHttpResponse(gen(IPWebCam()),
                    content_type='multipart/x-mixed-replace; boundary=frame')

def front_feed(request):
    global front_topic
    return StreamingHttpResponse(gen(RosCamera(front_topic)),
                    content_type='multipart/x-mixed-replace; boundary=frame')
def front_down_feed(request):
    global front_down_ip
    return StreamingHttpResponse(gen(IPWebCam(front_down_ip)),
                    content_type='multipart/x-mixed-replace; boundary=frame')
def rear_right_feed(request):
    global rear_right_topic
    return StreamingHttpResponse(gen(CompressedRosCamera(rear_right_topic)),
                    content_type='multipart/x-mixed-replace; boundary=frame')
def rear_left_feed(request):
    global rear_left_topic
    return StreamingHttpResponse(gen(CompressedRosCamera(rear_left_topic)),
                    content_type='multipart/x-mixed-replace; boundary=frame')

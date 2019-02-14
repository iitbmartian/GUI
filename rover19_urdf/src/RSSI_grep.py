#!/usr/bin/env python
import selenium, time
import signal
import sys
import rospy
from std_msgs.msg import Int32MultiArray, String
from selenium import webdriver
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.common.by import By
import numpy as np

from pandas import read_csv
from matplotlib import pyplot
from statsmodels.tsa.arima_model import ARIMA
from sklearn.metrics import mean_squared_error
from pandas import *

import matplotlib.pyplot as plt
import time

def sigint_handler(signal,frame):
    global RSSI_vals
    np.save('RSSI_inc_3.npy',np.array(RSSI_vals))
    sys.exit(0)

signal.signal(signal.SIGINT,sigint_handler)

webbrowser=webdriver.Firefox(executable_path="/usr/local/bin/geckodriver")
webbrowser.maximize_window()

#try catch if not connected
webbrowser.get("https://192.168.1.1/login.asp")

email_field=webbrowser.find_element_by_id("login-username")
email_field.clear()
email_field.send_keys("martian_comm")
time.sleep(1)
 
password_field=webbrowser.find_element_by_id("password")
password_field.clear()
password_field.send_keys("PoE9316"+Keys.ENTER)
time.sleep(1)

data_pub=rospy.Publisher("RSSI_data",Int32MultiArray,queue_size=10)
msg_pub=rospy.Publisher("RSSI_msg",String,queue_size=10)
rospy.init_node("RSSI", anonymous=True)

#*****Forecasting Code*****

def check_RSSI(history):
	data = history
	for i in range(0,3):
		model = ARIMA(data, order=(5,2,0))
		model_fit = model.fit(disp=0)
		output = model_fit.forecast()
		yhat = output[0]
		data.append(yhat)
		RSSI_data[1] = yhat
		if(yhat < 12):
			print("Stop!")
			RSSI_msg.publish("Stop")



RSSI_vals=np.array([])
smooth_RSSI = np.array([])
history = np.array([])
predictions = np.array([])
RSSI_data = np.zeros(2)

##PLotting code
xdata = []
ydata = []
plt.show()
axes = plt.gca()
axes.set_xlim(0, 200)
axes.set_ylim(-10, 100)
line, = axes.plot(xdata, ydata, 'r-')

while not rospy.is_shutdown():
	iframe = webbrowser.find_element_by_id("mainframe")
	webbrowser.switch_to.frame(iframe)
	elem = webbrowser.find_element_by_xpath('//tbody[@class="striped"]/tr/td[5]')
	value=int(elem.text)
	np.append(RSSI_vals,value)
	RSSI_data[0] = value
	RSSI_data[1] = 0
	
	if(len(RSSI_vals)<=5):
		np.append(smooth_RSSI, np.mean(RSSI_vals))
	else:
		new_val = (smooth_RSSI[-1]*5 - RSSI_vals[-6] + RSSI_vals[-1])/5
		np.append(smooth_RSSI, new_val)

	# print(history)
	# for t in range(len(test)):
	# model = ARIMA(history, order=(5,2,0))
	# model_fit = model.fit(disp=0)
	# output = model_fit.forecast()
	# yhat = output[0]
	# predictions.append(yhat)
	# obs = smooth_RSSI[-1]

	xdata.append(len(RSSI_vals))
	ydata.append(RSSI_vals[-1])
	line.set_xdata(xdata)
	line.set_ydata(ydata)
	plt.draw()
	plt.pause(1e-17)
	
	history.append(smooth_RSSI[-1])
	if (value < 20):
		check_RSSI(history)

	RSSI_pub.publish(data=RSSI_data)
	webbrowser.refresh()
	time.sleep(1)
	



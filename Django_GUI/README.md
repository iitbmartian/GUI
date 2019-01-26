
# GUI
## Getting Started
### 1.Basic settings
<<<<<<< HEAD
First, download or clone the GUI repository into your pc directory. Then,open the file `settings.py` at `your-dir/GUI/mysite/settings.py`. Here, Change the directory in `Templates[{..,'DIRS':['your-direct/GUI/mysite/templates/'],...}]`. 
=======
First, download or clone the GUI repository into your pc directory. Then,open the file `settings.py` at `your-dir/GUI/mysite/settings.py`. Here, Change the directory in `Templates[{..,'DIRS':['your-direct/GUI/mysite/templates/'],...}]` to the directory in which GUI is cloned. 
>>>>>>> 2eaea023202bb669c51d7d26bcfe9c73b43affad

### 2.ROS Setup
Now, start the terminal and turn on the ros master.

`$ roscore`

> **Note: Don't close this terminal for the entire project.** 

### 3.Running the Server
Now start new terminal and enter into GUI directory.

`$ cd your-dir/GUI`

**_To run project locally on a pc server_:**

`$ python manage.py runserver`

Now, you can run the project on the chrome at http://127.0.0.1:8000

**_To run the project on all the pc's connected through the same wifi_:**

First we need to get the inet addresss of the wifi network. Start the terminal, run

`$ ifconfig`

Here look for the inet address of your wifi network. It is of the form 192.168.0.X 

Now start the terminal and run

`$ python manage.py runserver 192.168.0.X:8000` 

Now you will be able to access the project on all the devices connected to the same network at http://192.168.0.X:8000

## Description
### 1.ROS Code
All the publishers and subscribers must be made inside `../GUI/manage.py` file.
Also all the data must be published or subscribed from the `manage.py` only. 

### 2.URL 
All the URLs must be inserted into `../GUI/mysite/urls.py` file. In our project, the template page run runs at `server-url/index`.

### 3.Template Page
 The `index.html` and `styles.css` placed at `../GUI/mysite/templates` directory forms the major part for client side of the webpage `server-url/index`.
 
 **Understanding index.html**

_(i)form attribute_: The `form` attribute is used as its data can be sent as URL variables (with `method="GET"`) or as HTTP post transaction (with `method="POST"`). The main difference between GET and POST method is that GET carries request parameter appended in URL string while POST carries request parameter in message body without changing URL which makes it more secure way of transferring data from client to server in http protocol.

 We do not want to change URL as we proceed in our operations so we used the POST method.

 _(ii)\<input\> form attribute_:The input tags are given `type="submit"` since it defines a button which submit the value of it to `views.py` where it is further processed.

All the input tags are given the same `name="action"` since it is being used in `views.py` to capture the value of the button pressed.

_(iii)RTSP Server Stream_: The `div` tag with class server-stream contains the code for the live stream from the IP Camera to the webpage. For the live stream, both the server and camera must be on the same network. Also, the server page must be given access to flash player for the live stream. 

You can notice some URL patterns of format `http://192.168.0.109:8080/...`.You may need to change these URL formats to those provided by your IP Cameras.

### _4.Views.py_:
The `views.py` is in `../GUI/mysite directory`. 

When the function `index(request)` receives an `GET` type request like Webpage Loading, Refreshing etc. it loads the `index.html` page.

When further the buttons are pressed on the webpage the `POST` type requests are made to the function `index(request)`. Here `request.POST['action']` identifies the `value` attribute of the button pressed and assigns to `direction` variable. This `direction` is then sent to `manage.py` using `send_msg` function where the publisher publishes it.









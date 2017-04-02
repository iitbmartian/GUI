# GUI_MSI_2016
GUI for controlling various subsystems of the Mars Rover Project, IIT-Bombay.

![The evolution of the rover design!](evolution.jpg)

## Visualisation of Rover on RViz and Gazebo

### Getting Started with Visualising
Clone the repo, copy the msi_2k16_17_urdf folder to src in your catkin workspace, and after that, go to *[..]/msi_2k16_17_urdf/launch* and run `roslaunch msi_2k16_17_urdf display.launch` to get rviz loaded with the xacro file. After that, change the fixed_frame parameter to **\base_link** and load *robot model* from the options to get the model on the panel. </br>
You will need to clone [Jaco Arm Repo](https://github.com/ksatyaki/JacoROS/), and copy  **jaco_description** directory to */opt/ros/<version>/share* for the xacro to run. In the jaco.urdf file in opt/share, change base_link to base_link1 to avoid conflict.

---
### ROS-URDF

Follow [this link](http://wiki.ros.org/urdf/XML/joint) for the complete documentation of the XML tags under _joint_, and more.  
_Note that all angle measurements are in **radians**._  

Defining inertia can get crucial for Gazebo modelling. The values are to be given as 3D inertia tensors, and a simple reference can be found [here](https://en.wikipedia.org/wiki/List_of_moments_of_inertia). Accurate data can be obtained from [SolidWorks](https://forum.solidworks.com/thread/59325) or [Inventor](http://forums.autodesk.com/t5/inventor-forum/calculate-moment-of-inertia/td-p/3027000).

* For changing the dimensions in the included mesh file, use the following `<mesh : "Filepath://filename" scale = "multplier_x multiplier_y multiplier_z" >`

---  

### Gazebo
ROS Indigo has support for Gazebo2 only, (which is not the case with Kinetic). If you have any other version, you need to purge and do :
```
sudo apt-get install gazebo2
sudo apt-get install ros-indigo-gazebo-ros
```

 [_Important!_ ] The [gazebo_ros_control](http://gazebosim.org/tutorials?tut=ros_control) tutorial will explain how to use Rviz to monitor the state of your simulated robot by publishing `/joint_states` directly from Gazebo. **Would be requied!**

``` bash
cd ~/catkin_ws/src
git clone https://github.com/iitbmartian/GUI_MSI_2016
cd ../
catkin_make
source devel/setup.bash

roslaunch msi_2k16_17_gazebo msi.launch
```

To add a custom world in Gazebo, add a parameter while running `world_name:=worlds/willowgarage.world` for the default, or modify the file accordingly as in `msi.launch`.

---

---

## GPS lat long tracking

Work derived from the repository by [Gareth Cross and team](https://github.com/gareth-cross/rviz_satellite/). Changes done to the original code to make the tiles preloaded, as internet access may not be availaible in MDRS, Utah.


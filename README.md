# GUI_MSI_2016
GUI for controlling various subsystems of the Mars Rover Project, IIT-Bombay.

![The evolution of the rover design!](evolution.jpg)

## Visualisation of Rover on RViz and Gazebo

### Getting Started with Visualising
Clone the repo, copy the msi_2k16_17_urdf folder to src in your catkin workspace, and after that, go to *[..]/msi_2k16_17_urdf/launch* and run `roslaunch msi_2k16_17_urdf display.launch` to get rviz loaded with the xacro file. After that, change the fixed_frame parameter to **\base_link** and load *robot model* from the options to get the model on the panel. </br>
~~You will need to clone [Husky repository](https://github.com/husky/husky) and [Jaco Arm Repo](https://github.com/ksatyaki/JacoROS/), and copy the **husky_description**, and **jaco_description** directory to */opt/ros/<version>/share* for the xacro to run and in the jaco.urdf file in opt/share, change base_link to base_link1 to avoid conflict.~~

Just clone the repo, and you are good to go!

</br>
For more info on the same, refer to :- </br>
[Tutorials on URDF and Xacro: 1, 2, 4](http://wiki.ros.org/urdf/Tutorials) </br>

_After any changes in directory structure or packages, run the following **without fail**:_  [current_dir: catkin_ws]
```bash
catkin_make
source devel/setup.bash
```

---
### ROS-URDF
Joints can be of the following kinds:
* __revolute__ - a hinge joint that rotates along the axis and has a limited range specified by the upper and lower limits. _[gripper claw]_
* __continuous__ - a continuous hinge joint that rotates around the axis and has no upper and lower limits _[wheels, gripper hinge]_
* __prismatic__ - a sliding joint that slides along the axis, and has a limited range specified by the upper and lower limits.
* __fixed__ - This is not really a joint because it cannot move. All degrees of freedom are locked. This type of joint does not require the axis, calibration, dynamics, limits or safety_controller.
* __floating__ - This joint allows motion for all _6_ degrees of freedom.
* __planar__ - This joint allows motion in a plane perpendicular to the axis.

Follow [this link](http://wiki.ros.org/urdf/XML/joint) for the complete documentation of the XML tags under _joint_, and more.  
_Note that all angle measurements are in **radians**._  
**rpy** refers to Roll-Pitch-Yaw. Not very clear how it works out in the R2D2 example! :/


Defining inertia can get crucial for Gazebo modelling. The values are to be given as 3D inertia tensors, and a simple reference can be found [here](https://en.wikipedia.org/wiki/List_of_moments_of_inertia). Accurate data can be obtained from [SolidWorks](https://forum.solidworks.com/thread/59325) or [Inventor](http://forums.autodesk.com/t5/inventor-forum/calculate-moment-of-inertia/td-p/3027000).

While running the tutorials **do not** download the `urdf_tutorials` package from GH. It is an in-built ROS package. Just download the `urdf` folder.

---
* Changed the symbolic links in the main xacro file `msi.xacro`. The changes in dimensions finally work!
* For changing the dimensions in the included mesh file, use the following `<mesh : "Filepath://filename" scale = "multplier_x multiplier_y multiplier_z" >`
* In the jaco.urdf file in opt/share, change base_link to base_link1 to avoid conflict.
* For changing the dimensions in the included mesh file, use the following `<mesh : "Filepath://filename" scale = "multplier_x multiplier_y multiplier_z" >`
* There's something not right by using the copied wheel.urdf.xacro and hence using the original husky file. Also, some dimensions like `wheelbase` and `wheel_vertical_offset` seem to be creating discontinuities but chuck! Sorted for now! ^\_^
* Best part: Colors are editable as no mesh file here!

---  

### Gazebo
Note that although the latest version `7.x` is supposedly the best(and GUI \_/\\\_), and runs on _ALL_ platforms, for linking with ROS, there are compatiblity and ROS Indigo needs to `ros_pkgs` from `gazebo2`. (ROS Kinetic is fine with this version). So if the latest version was downloaded from deb/tarball, purge it (might have to use the Software Centre for that) and run a simple, modest:
```
sudo apt-get install gazebo2
sudo apt-get install ros-indigo-gazebo-ros
```

 [_Important!_ ] The [gazebo_ros_control](http://gazebosim.org/tutorials?tut=ros_control) tutorial will explain how to use Rviz to monitor the state of your simulated robot by publishing `/joint_states` directly from Gazebo. **Would be requied!**

 [_TBD Later_] We can actually add a desert-like terrain in the GUI at URC. Check out [DEM](http://gazebosim.org/tutorials/?tut=dem).

Okay, so Gazebo seems nice and has been set up as of date. The model has been loaded in an empty world with nothing else in particular (tried, but then no enthu!). The repo has taken major changes and hence, I'd say it would be nice to clone this directly in your workspace (or add a remote etc.)
``` bash
cd ~/catkin_ws/src
git clone https://github.com/Agrim9/GUI_MSI_2016
cd ../
catkin_make
source devel/setup.bash

roslaunch msi_2k16_17_gazebo msi.launch
```

To add a custom world in Gazebo, add a parameter while running `world_name:=worlds/willowgarage.world` for the default, or modify the file accordingly as in `msi.launch`.

---

---

## GPS lat long tracking

Work derived from the repository by [Gareth Cross and team](https://github.com/gareth-cross/rviz_satellite/blob/master/src/tileloader.h). Changes done to the original code to make the tiles preloaded.

## Deadlines
- [x] Familiarisation with URDF and Gazebo
- [x] URDF tutorials and Husky integration
- [x] Final URDF model(with arm): Deadline : 14/12/16
- [x] Gazebo intro and tuts: Deadline : 14/12/16
- [x] Gazebo integration and basic movement: Deadline : 17/12/16
- [ ] Full intergation of movement in GUI: Deadline : 13/1/17

# GUI_MSI_2016
GUI for controlling various subsystems of the Mars Rover Project, IIT-Bombay.

### Getting Started with Visualising
Clone the repo, copy the msi_2k16_17_urdf folder to src in your catkin workspace, and after that, go to *[..]/msi_2k16_17_urdf/launch* and run `roslaunch msi_2k16_17_urdf display.launch` to get rviz loaded with the xacro file. After that, change the fixed_frame parameter to **\base_link** and load *robot model* from the options to get the model on the panel. </br>
Also, you will need to clone [Husky repository](https://github.com/husky/husky), and copy the **husky_description** directory to */opt/ros/<version>/share* for the xacro to run. </br>
</br>
For more info on the same, refer to :- </br>
[Tutorials on URDF and Xacro: 1, 2, 4](http://wiki.ros.org/urdf/Tutorials) </br>

_After any changes in directory structure or packages, run the following **without fail**:_  [current_dir: catkin_ws]
```(bash)
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
* For changing the dimensions in the xacro file, you can change the relative positions etc of the `base_link` and `husky_wheel`, but not the actual dimensions of the components because they are apparently conveyed in the `.dae` file, and it's root URDF is not available. What's worse? XML parse failed and hence no retro-conversion.
* Solution? Ditched the husky chassis and made a simple box instead. Could've done that for the wheels too, but they look cool and are pretty close to our real dimensions. _Box made to scale._
* Using the copied wheel xacro somehow ruined everything and the wheels went crazy AF! (Just when you thought it was done.! Check commit [Final URDF(w/o arm) Commit](https://github.com/Agrim9/GUI_MSI_2016/tree/1d3b61bcccbe5419caa51e832377b4f45a5ad261) )

Finally winding up the URDF business after a ton of issues. One thing: there's something not right by using the copied wheel.urdf.xacro and hence using the original husky file. Also, some dimensions like `wheelbase` and `wheel_vertical_offset` seem to be creating discontinuities but chuck! Sorted for now! ^_^
---

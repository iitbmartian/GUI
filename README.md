# GUI_MSI_2016
GUI for controlling various subsystems of the Mars Rover Project, IIT-Bombay.

## Visualisation of the Xacro file on RViz
Clone the repo, copy the msi_2k16_17_urdf folder to src in your catkin workspace, and after that, go to home of your workspace and run `roslaunch msi_2k16_17_urdf display.launch model:=urdf/msi.xacro` to get rviz loaded with the xacro file. After that, change the fixed_frame parameter to **\base_link"** and load robot model from the options to get the model on the panel </br>
</br>

For more info on the same, refer to :- </br> 
[Tutorialson URDF and Xacro, tuts 1,2,4](http://wiki.ros.org/urdf/Tutorials) </br>

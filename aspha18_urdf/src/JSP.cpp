#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>

sensor_msgs::JointState joint_state;
bool init_flag=true;
ros::Publisher joint_pub;

void callbackFn(const std_msgs::Float64MultiArray::ConstPtr& inp)
{
  
  //ROS_INFO("JSP node Message Received");
  std::vector<double> thet_phi = inp -> data;

  joint_state.position[5]=thet_phi[1];
  joint_state.position[6]=thet_phi[0];
  joint_state.position[7] = -0.89;

  //update joint_state
  //ROS_INFO("Message published to JSP");
  joint_pub.publish(joint_state);
}

int main(int argc, char** argv) {

ros::init(argc, argv, "JSP_node");
ros::NodeHandle n;
joint_pub = n.advertise<sensor_msgs::JointState>("URDF_JSP_pub", 10);
joint_state.name.resize(23);
joint_state.position.resize(23);

joint_state.name[0] ="rotate1";
joint_state.name[1] ="rotate2";
joint_state.name[2] ="rotate3";
joint_state.name[3] ="rotate4";
joint_state.name[4] ="base_rot1";
joint_state.name[5] ="shoulder_joint";
joint_state.name[6] ="elbow_joint";
joint_state.name[7] ="gripper_joint";

joint_state.position[0] = 0;
joint_state.position[1] = 0;
joint_state.position[2] = 0;
joint_state.position[3] = 0;
joint_state.position[4] = 0;
joint_state.position[5] = 0;
joint_state.position[6] = 0;
joint_state.position[7] = -0.84;

ros::Subscriber sub = n.subscribe("/Theta_Phi", 1000, callbackFn);
ros::spin();

return 0;
}

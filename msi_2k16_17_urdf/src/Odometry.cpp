#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_broadcaster.h>

sensor_msgs::JointState joint_state;
bool init_flag=true;
ros::Publisher joint_pub;
void joy_JSP(const sensor_msgs::Joy::ConstPtr& joy)
{
  
  ROS_INFO("Message Recieved");
  std::vector<float> axe = joy -> axes;
  std::vector<int> but = joy -> buttons;

  joint_state.position[13]=0.52*axe[3];
  joint_state.position[15]=0.52*axe[3];
  joint_state.position[17]=-0.52*axe[3];
  joint_state.position[19]=-0.52*axe[3];

  //update joint_state
  ROS_INFO("Message published to JSP");
  std::cout<<joint_state.position[13];
  joint_pub.publish(joint_state);
}

int main(int argc, char** argv) {

ros::init(argc, argv, "JSP_joy_node");
ros::NodeHandle n;
joint_pub = n.advertise<sensor_msgs::JointState>("joy_joint_states", 10);
joint_state.name.resize(23);
joint_state.position.resize(23);

joint_state.name[0] ="jaco_joint_1";
joint_state.name[1] ="jaco_joint_2";
joint_state.name[2] ="jaco_joint_3";
joint_state.name[3] ="jaco_joint_4";
joint_state.name[4] ="jaco_joint_5";
joint_state.name[5] ="jaco_joint_6";
joint_state.name[6] ="jaco_finger_joint_1";
joint_state.name[7] ="jaco_finger_joint_2";
joint_state.name[8] ="jaco_finger_joint_3";
joint_state.name[9] ="rocker_R_to_chassis";
joint_state.name[10] ="rocker_L_to_chassis";
joint_state.name[11] ="bogie_R_to_rocker";
joint_state.name[12] ="bogie_L_to_rocker";
joint_state.name[13] ="front_L_steer";
joint_state.name[14] ="front_L_drive";
joint_state.name[15] ="front_R_steer";
joint_state.name[16] ="front_R_drive";
joint_state.name[17] ="rear_L_steer";
joint_state.name[18] ="rear_L_drive";
joint_state.name[19] ="rear_R_steer";
joint_state.name[20] ="rear_R_drive";
joint_state.name[21] ="center_wheel_drive_L";
joint_state.name[22] ="center_wheel_drive_R";

joint_state.position[0] = 0;
joint_state.position[1] = 0;
joint_state.position[2] = 0;
joint_state.position[3] = 2.45;
joint_state.position[4] = 2.49;
joint_state.position[5] = 1.87;
joint_state.position[6] = 0.3;
joint_state.position[7] = 0.25;
joint_state.position[8] = 0.25;
joint_state.position[9] = 0;
joint_state.position[10] = 0;
joint_state.position[11] = 0;
joint_state.position[12] = 0;
joint_state.position[14] = 0;
joint_state.position[16] = 0;
joint_state.position[18] = 0;
joint_state.position[20] = 0;
joint_state.position[21] = 0;
joint_state.position[22] = 0;
joint_state.position[13]=0.51;
joint_state.position[15]=0.51;
joint_state.position[17]=0.51;
joint_state.position[19]=0.51;

ros::Subscriber sub = n.subscribe("/joy", 1000, joy_JSP);
ros::spin();

return 0;
}
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
  std::vector<double> arm_angs = inp -> data;
  joint_state.position[4] = 0;
  joint_state.position[1]=-arm_angs[0]+1.05;
  joint_state.position[2]=-arm_angs[1]+2.3339;
  joint_state.position[3]=-arm_angs[2]+2.645;
  joint_state.position[5]=arm_angs[3];
  joint_state.position[6]=arm_angs[4];
  joint_state.position[7]=arm_angs[5];

  //update joint_state
  //ROS_INFO("Message published to JSP");
  joint_pub.publish(joint_state);
}

int main(int argc, char** argv) {

ros::init(argc, argv, "JSP_node");
ros::NodeHandle n;
joint_pub = n.advertise<sensor_msgs::JointState>("URDF_JSP_pub", 8);
joint_state.name.resize(8);
joint_state.position.resize(8);

joint_state.name[0] ="base_rot_joint";
joint_state.name[1] ="shoulder_joint";
joint_state.name[2] ="elbow_joint";
joint_state.name[3] ="gripper_link1_joint";
joint_state.name[4] ="gripper_link2_joint";
joint_state.name[5] ="finger0_joint";
joint_state.name[6] ="finger1_joint";
joint_state.name[7] ="finger2_joint";

joint_state.position[0] = 0;
joint_state.position[1] = 0;
joint_state.position[2] = 0;
joint_state.position[3] = 0;
joint_state.position[4] = 0;
joint_state.position[5] = 0;
joint_state.position[6] = 0.05;
joint_state.position[7] = 0.05;

joint_pub.publish(joint_state);
ros::Subscriber sub = n.subscribe("/arm_angs", 1000, callbackFn);
ros::spin();

return 0;
}

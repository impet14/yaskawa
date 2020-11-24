#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <vector>
#include <cmath>
using namespace std;

class DHandJointStateRePublisher
{
public:
  DHandJointStateRePublisher(ros::NodeHandle &nh);
  sensor_msgs::JointState js2;

private:
  void jointstateCallback(const sensor_msgs::JointState::ConstPtr &js);
  void jointstateCallback2(const sensor_msgs::JointState::ConstPtr &js);

  ros::Subscriber js_sub;
  ros::Subscriber js_sub2;
  ros::Publisher js_pub;

  std::vector<std::string> dhand_joint_name;
};

DHandJointStateRePublisher::DHandJointStateRePublisher(ros::NodeHandle &nh)
{
  // dhand_joint_name.push_back("dhand_finger_base_left_joint");
  // dhand_joint_name.push_back("dhand_finger_base_right_joint");
  // dhand_joint_name.push_back("dhand_finger_middle_left_joint");
  // dhand_joint_name.push_back("dhand_finger_middle_middle_joint");
  // dhand_joint_name.push_back("dhand_finger_middle_right_joint");
  // dhand_joint_name.push_back("dhand_finger_top_left_joint");
  // dhand_joint_name.push_back("dhand_finger_top_middle_joint");
  // dhand_joint_name.push_back("dhand_finger_top_right_joint");
  dhand_joint_name.push_back("twintool_joint1");
  dhand_joint_name.push_back("twintool_joint2_1");

  ros::NodeHandle n("~");
  js_pub = nh.advertise<sensor_msgs::JointState>(n.param<std::string>("joint_state_republish_topic_name", "/joint_states_republish"), 1);
  js_sub = nh.subscribe<sensor_msgs::JointState>(n.param<std::string>("joint_state_subscribe_topic_name", "/joint_states"), 10, &DHandJointStateRePublisher::jointstateCallback, this);
  js_sub2 = nh.subscribe<sensor_msgs::JointState>(n.param<std::string>("joint_state_subscribe_topic_name2", "/joint_states"), 10, &DHandJointStateRePublisher::jointstateCallback2, this);
}
void DHandJointStateRePublisher::jointstateCallback2(const sensor_msgs::JointState::ConstPtr &js)
{
  js2.header = js->header;
  js2.name = js->name;
  js2.position = js->position;
  js2.velocity = js->velocity;
  js2.effort = js->effort;
}
void DHandJointStateRePublisher::jointstateCallback(const sensor_msgs::JointState::ConstPtr &js)
{
  sensor_msgs::JointState joint_state;

  joint_state.header = js->header;
  joint_state.name = js->name;
  joint_state.position = js->position;
  joint_state.velocity = js->velocity;
  joint_state.effort = js->effort;

  if (js2.name.size() > 0)
  {
    for (int i = 0; i < js2.name.size(); i++)
    {
      joint_state.name.push_back(js2.name.at(i));
      joint_state.position.push_back(js2.position[i]);
      joint_state.velocity.push_back(js2.velocity[i]);
      joint_state.effort.push_back(js2.effort[i]);
    }
  }

  js_pub.publish(joint_state);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "dhand_joint_state_republisher");
  ros::NodeHandle n;
  DHandJointStateRePublisher dhand_joint_state_republisher(n);
  ros::spin();
  return 0;
}
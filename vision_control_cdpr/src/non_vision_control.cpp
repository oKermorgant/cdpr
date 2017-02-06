#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/LinkStates.h"

void testCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
  ROS_INFO("Executed!");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "non_vision_control");
  ros::NodeHandle n("~");

  int queue_size = 1;
  ros::Subscriber test_sub = n.subscribe<gazebo_msgs::LinkStates>("link_states", queue_size,
                                                                  testCallback);
  ros::spin();

  return 0;
}

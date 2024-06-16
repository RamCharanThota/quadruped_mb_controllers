#include <ros/ros.h>
#include <urdf/model.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "anymal_c_kin_dyn");
  ros::NodeHandle nh;

  // Load the URDF model from the parameter server
  urdf::Model model;
  if (!model.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse URDF file");
    return -1;
  }

  // Print the robot name
  ROS_INFO("Successfully parsed URDF model: %s", model.getName().c_str());

  // Iterate through all the links and print their names
  for (const auto& link_pair : model.links_)
  {
    ROS_INFO("Link name: %s", link_pair.second->name.c_str());
  }

  ros::spin();
  return 0;
}


#include<pinocchio/fwd.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <pinocchio/parsers/sample-models.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>

int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "pinocchio_example_node");
  ros::NodeHandle nh;

  // Create a publisher to publish tau (joint torques)
  ros::Publisher tau_pub = nh.advertise<std_msgs::Float64MultiArray>("joint_torques", 10);

  // Create Pinocchio model and data structures
  pinocchio::Model model;
  pinocchio::buildModels::manipulator(model);
  pinocchio::Data data(model);

  // Compute initial state (assuming neutral position)
  Eigen::VectorXd q = pinocchio::neutral(model);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

  // Compute joint torques (RNEA - Recursive Newton-Euler Algorithm)
  const Eigen::VectorXd & tau = pinocchio::rnea(model, data, q, v, a);

  // Publish tau as a ROS message
  std_msgs::Float64MultiArray tau_msg;
  tau_msg.data.resize(model.nv);
  for (int i = 0; i < model.nv; ++i) {
    tau_msg.data[i] = tau[i];
  }

  // Publish tau
  tau_pub.publish(tau_msg);

  // Print tau to console
  ROS_INFO_STREAM("tau = " << tau.transpose());

  // Spin ROS node
  ros::spin();

  return 0;
}

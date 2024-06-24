
#include<pinocchio/fwd.hpp>
#include <ros/ros.h>
#include <ros/package.h>


#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"

// directory here.
#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "path_to_the_model_dir"
#endif



int main(int argc, char ** argv)
{

    ros::init(argc, argv, "urdf_to_pinocchio_node");
    ros::NodeHandle nh;

  // Change to your own URDF file here, or give a path as command-line argument
  std::string package_path=ros::package::getPath("anymal_c_simple_description");
  // Construct the URDF file path
  std::string urdf_filename = package_path + "/urdf/anymal.urdf";
  

  // Load the URDF model
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  // Build a data frame associated with the model
  pinocchio::Data data(model);

  // Sample a random joint configuration, joint velocities and accelerations
  Eigen::VectorXd q = pinocchio::randomConfiguration(model);      // in rad for the UR5
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv); // in rad/s for the UR5
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv); // in rad/s² for the UR5

  // Computes the inverse dynamics (RNEA) for all the joints of the robot
  Eigen::VectorXd tau = pinocchio::rnea(model, data, q, v, a);

  // Print out to the vector of joint torques (in N.m)
  std::cout << "Joint torques: " << data.tau.transpose() << std::endl;

   ros::spin();

    return 0;
}

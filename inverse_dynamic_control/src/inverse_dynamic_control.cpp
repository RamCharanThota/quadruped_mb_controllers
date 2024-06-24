
#include<pinocchio/fwd.hpp>
#include <ros/ros.h>
#include <ros/package.h>



#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/rnea.hxx>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/crba.hpp>




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

  // compute the mass matrix 
  // Compute the mass matrix
    pinocchio::crba(model, data, q);
    Eigen::MatrixXd M = data.M;
    M.triangularView<Eigen::StrictlyLower>() = M.transpose().triangularView<Eigen::StrictlyLower>();

    // Compute the Coriolis matrix
    Eigen::MatrixXd C = pinocchio::computeCoriolisMatrix(model, data, q, v);

    // Compute the gravity vector
    Eigen::VectorXd g = pinocchio::computeGeneralizedGravity(model, data, q);

    // Print the results
    std::cout << "Mass Matrix (M):\n" << M << std::endl;
    std::cout << "Coriolis Matrix (C):\n" << C << std::endl;
    std::cout << "Gravity Vector (g):\n" << g.transpose() << std::endl;


  // Print out to the vector of joint torques (in N.m)
  std::cout << "Joint torques: " << data.tau.transpose() << std::endl;

   ros::spin();

    return 0;
}

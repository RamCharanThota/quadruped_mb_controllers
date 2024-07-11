#include <tsid/robots/robot-wrapper.hpp>
#include <tsid/solvers/solver-HQP-factory.hxx>
#include <tsid/tasks/task-se3-equality.hpp>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/utils/timer.hpp>
#include <Eigen/Core>


#include <iostream>
#include <fstream>
#include <string>
#include <vector>

using namespace tsid;
using namespace tsid::robots;
using namespace tsid::solvers;
using namespace tsid::tasks;
using namespace tsid::trajectories;
using namespace tsid::contacts;

int main(int argc, char *argv[])
{
    if(argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " path_to_urdf" << std::endl;
        return -1;
    }

    std::string urdf_file = "robot.urdf";
    std::vector<std::string> urdf_path = {"/path/to/project/my_robot_description"};

    const std::string robot_name = "robot";
    const double dt = 0.001; // Time step

    RobotWrapper robot(urdf_file, urdf_path, false);
    const pinocchio::Model &model = robot.model();

    std::cout << "Robot model loaded with " << model.nq << " DoF" << std::endl;

    InverseDynamicsFormulationAccForce tsid(robot_name, robot, false);
    Vector q(model.nq);
    Vector v(model.nv);
    q.setZero();
    v.setZero();

    tsid.computeProblemData(0.0, q, v);
    const double w = 1.0;
    const std::string task_name = "task-se3";
    const std::string frame_name = "end_effector";
    
    TaskSE3Equality task(task_name, robot, frame_name);
    const Vector3d pos_desired(0.3, 0.3, 0.3);
    SE3 desired_pose(SE3::Identity());
    desired_pose.translation() = pos_desired;

    TrajectorySample sample(12);
    sample.pos.head<3>() = desired_pose.translation();
    task.setReference(sample);

    tsid.addMotionTask(task, w, 1);

    SolverHQPBase *solver = SolverHQPFactory::createNewSolver(SOLVER_HQP_QPOASES, "solver");

    HQPData hqpData = tsid.computeProblemData(0.0, q, v);
    const HQPOutput &sol = solver->solve(hqpData);

    if(sol.status == HQP_STATUS_OPTIMAL)
    {
        std::cout << "QP solved!" << std::endl;
        std::cout << "qddot: " << sol.x << std::endl;
    }
    else
    {
        std::cerr << "QP problem could not be solved." << std::endl;
    }

    delete solver;

    return 0;
}

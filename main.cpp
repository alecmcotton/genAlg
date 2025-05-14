#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "kinematics.h"

int main() {

    std::vector<DHParam> dh_params = {
        {0.0, 135.0, 0.0, -M_PI / 2},
        {-M_PI / 2, 0.0, 135.0, 0.0},
        {0.0, 0.0, 38.0, -M_PI / 2},
        {0.0, 120.0, 0.0, M_PI / 2},
        {0.0, 0.0, 0.0, -M_PI / 2},
        {M_PI, 70.0, 200.0, 0.0}
    };


    Eigen::VectorXd joint_angles(dh_params.size());
    joint_angles << M_PI/8, M_PI/8, M_PI/8, M_PI/8, M_PI/8, M_PI/8;

    std::vector<DHParam> robot_params = substituteJoints(dh_params,joint_angles);
    //for (size_t i = 0; i < dh_params.size(); ++i) {
    //    dh_params[i].theta += joint_angles[i];
    //}

    Eigen::VectorXd pose = forwardKinematics(robot_params);
    std::cout << "End-Effector Position (x, y, z):\n" << pose.head<3>().transpose() << std::endl;
    std::cout << "End-Effector Orientation (Euler angles α, β, γ in radians):\n" << pose.tail<3>().transpose() << std::endl;
    Eigen::MatrixXd J = computeNumericalJacobian(robot_params);
    std::cout << "Jacobian:\n" << J << std::endl;
    double mu = computeManipulability(J);
    std::cout << "Manipulability:\n" << mu << std::endl;
    Eigen::VectorXd q = numericalInverseKinematics(pose,dh_params,200,1e-3,1e-4);
    std::cout << "Inverse kinematic solution:\n" << q << std::endl;
    std::vector<DHParam> next_params = substituteJoints(dh_params,q);
    Eigen::VectorXd pose_check = forwardKinematics(next_params);
    std::cout << "End-Effector Position (x, y, z):\n" << pose_check.head<3>().transpose() << std::endl;
    std::cout << "End-Effector Orientation (Euler angles α, β, γ in radians):\n" << pose_check.tail<3>().transpose() << std::endl;
    double error = (pose_check-pose).norm();
    std::cout << "Error:\n" << error << std::endl;
    return 0;
}


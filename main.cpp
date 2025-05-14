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


    std::vector<double> joint_angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    for (size_t i = 0; i < dh_params.size(); ++i) {
        dh_params[i].theta += joint_angles[i];
    }

    Eigen::VectorXd pose = forwardKinematics(dh_params);

    std::cout << "End-Effector Position (x, y, z):\n" << pose.head<3>().transpose() << std::endl;
    std::cout << "End-Effector Orientation (Euler angles α, β, γ in radians):\n" << pose.tail<3>().transpose() << std::endl;

    return 0;
}


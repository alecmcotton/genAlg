#pragma once

#include <vector>
#include <Eigen/Dense>

struct DHParam {
    double theta;
    double d;
    double a;
    double alpha;
};

Eigen::Matrix4d computeDHMatrix(const DHParam& param);

Eigen::Matrix4d computeForwardTransform(const std::vector<DHParam>& params);

Eigen::VectorXd forwardKinematics(const std::vector<DHParam>& dhParams);

Eigen::MatrixXd computeJacobian(const std::vector<DHParam>& params);

double computeManipulability(const Eigen::MatrixXd& jacobian);

Eigen::Vector3d computeEulerAngles(const Eigen::Matrix3d& R);


#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <vector>
#include <Eigen/Dense>


struct DHParam {
    double theta;
    double d;
    double a;
    double alpha;
};


Eigen::Vector3d forwardKinematics(const std::vector<double>& jointAngles);

std::vector<double> inverseKinematics(const Eigen::Vector3d& endEffectorPos);

Eigen::Matrix4d computeDHMatrix(const DHParam& param);

Eigen::Matrix4d computeForwardTransform(const std::vector<DHParam>& dhParams);

Eigen::MatrixXd computeJacobian(const std::vector<double>& jointAngles);

double computeManipulability(const Eigen::MatrixXd& jacobian);

#endif


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
Eigen::MatrixXd computeNumericalJacobian(const std::vector<DHParam>& dhParams);
double computeManipulability(const Eigen::MatrixXd& jacobian);
Eigen::Vector3d computeEulerAngles(const Eigen::Matrix3d& R);
Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& mat, double epsilon = 1e-6);
std::vector<DHParam> substituteJoints(const std::vector<DHParam>& dh_params, const Eigen::VectorXd& joint_angles);
Eigen::VectorXd numericalInverseKinematics(const Eigen::VectorXd& pose,
                                           std::vector<DHParam>& dh_params,
                                           int max_iter,
                                           double tol,
                                           double lambda);

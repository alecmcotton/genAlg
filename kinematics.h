#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Eigen/Dense>
#include <vector>

Eigen::Matrix4d dh_transform(double a, double alpha, double d, double theta);
Eigen::Matrix4d forward_kinematics(const Eigen::VectorXd &theta, double L);
Eigen::Vector3d rotation_error(const Eigen::Matrix3d &R1, const Eigen::Matrix3d &R2);
Eigen::MatrixXd numerical_jacobian(const Eigen::VectorXd &theta, double L);

std::pair<Eigen::VectorXd, std::vector<double>> inverse_kinematics_NR(
    const Eigen::Matrix4d &T_target,
    Eigen::VectorXd theta0,
    double L,
    int max_iters = 100,
    double tol = 1e-4
);

#endif


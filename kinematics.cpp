#include "kinematics.h"
#include <cmath>

using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Matrix3d;

Matrix4d computeDHMatrix(const DHParam& param) {
    double ct = cos(param.theta);
    double st = sin(param.theta);
    double ca = cos(param.alpha);
    double sa = sin(param.alpha);

    Matrix4d T;
    T << ct, -st * ca,  st * sa, param.a * ct,
         st,  ct * ca, -ct * sa, param.a * st,
          0,       sa,       ca,       param.d,
          0,        0,        0,           1;
    return T;
}

Matrix4d computeForwardTransform(const std::vector<DHParam>& dhParams) {
    Matrix4d T = Matrix4d::Identity();
    for (const auto& param : dhParams) {
        T *= computeDHMatrix(param);
    }
    return T;
}

Eigen::VectorXd forwardKinematics(const std::vector<DHParam>& dhParams) {
    Eigen::Matrix4d T = computeForwardTransform(dhParams);
    Eigen::Vector3d position = T.block<3,1>(0, 3);
    Eigen::Matrix3d R = T.block<3,3>(0, 0);
    Eigen::Vector3d eulers = computeEulerAngles(R);

    Eigen::VectorXd result(6);
    result << position, eulers;
    return result;
}


Eigen::Vector3d computeEulerAngles(const Eigen::Matrix3d& R) {
    double alpha, beta, gamma;

    if (std::abs(R(2, 0)) < 1.0) {
        beta = std::asin(-R(2, 0));
        alpha = std::atan2(R(2, 1) / std::cos(beta), R(2, 2) / std::cos(beta));
        gamma = std::atan2(R(1, 0) / std::cos(beta), R(0, 0) / std::cos(beta));
    } else {
        beta = M_PI_2 * std::copysign(1.0, -R(2, 0));
        alpha = 0.0;
        gamma = std::atan2(-R(0, 1), R(1, 1));
    }

    return Eigen::Vector3d(alpha, beta, gamma);
}

double computeManipulability(const MatrixXd& jacobian) {
    MatrixXd JJt = jacobian * jacobian.transpose();
    return std::sqrt(JJt.determinant());
}

Eigen::MatrixXd computeNumericalJacobian(const std::vector<DHParam>& dhParams) {
    constexpr double epsilon = 1e-12;
    int n = dhParams.size();
    Eigen::MatrixXd J(6, n);

    for (int i = 0; i < n; ++i) {
        std::vector<DHParam> dh_lower = dhParams;
        std::vector<DHParam> dh_upper = dhParams;
        dh_lower[i].theta -= epsilon;
        dh_upper[i].theta += epsilon;
        Eigen::VectorXd pose_upper = forwardKinematics(dh_upper);
        Eigen::VectorXd pose_lower = forwardKinematics(dh_lower);
        Eigen::VectorXd diff = (pose_upper - pose_lower) / (2*epsilon);
        J.col(i) = diff;
    }

    return J;
}

std::vector<DHParam> substituteJoints(const std::vector<DHParam>& dh_params, const Eigen::VectorXd& joint_angles) {
    std::vector<DHParam> result = dh_params;
    for (size_t i = 0; i < result.size(); ++i) {
        result[i].theta += joint_angles[i];
    }
    return result;
}


Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& mat, double tolerance) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto& singularValues = svd.singularValues();
    Eigen::VectorXd invSingularValues(singularValues.size());

    for (int i = 0; i < singularValues.size(); ++i) {
        invSingularValues(i) = (singularValues(i) > tolerance) ? 1.0 / singularValues(i) : 0.0;
    }

    return svd.matrixV() * invSingularValues.asDiagonal() * svd.matrixU().transpose();
}

Eigen::VectorXd numericalInverseKinematics(const Eigen::VectorXd& pose,
                                            std::vector<DHParam>& dh_params,
                                            int max_iter, double tol, double lambda) {
    int iter = 0;
    Eigen::VectorXd theta_prev = Eigen::VectorXd::Zero(dh_params.size()); // are you starting at a singularilty??
    std::vector<DHParam> temp = substituteJoints(dh_params, theta_prev);
    Eigen::VectorXd curr = forwardKinematics(temp);
    Eigen::VectorXd error = (pose-curr);
    double e = error.norm();
    Eigen::VectorXd theta(dh_params.size());
    while (e > tol && iter < max_iter) {
        Eigen::MatrixXd J = computeNumericalJacobian(temp);
        Eigen::MatrixXd Jt = pseudoInverse(J, 1e-6);
        theta = Jt * error * lambda + theta_prev;
        theta_prev = theta;
        temp = substituteJoints(dh_params, theta);
        curr = forwardKinematics(temp);
        error = (pose-curr);
        e = error.norm();
        iter++;
    }

    return theta;
}


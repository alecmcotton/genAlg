#include <iostream>
#include <Eigen/Dense>
#include <cstdlib>
#include <ctime>
#include "kinematics.h"

using namespace Eigen;
using namespace std;

int main() {
    srand((unsigned int)time(0));
    double L = 200;
    VectorXd theta_true(6);
    for (int i = 0; i < 6; ++i)
        theta_true(i) = ((double)rand() / RAND_MAX) * 2 * M_PI - M_PI;

    Matrix4d T_desired = forward_kinematics(theta_true, L);
    VectorXd theta_guess = VectorXd::Zero(6);

    auto [theta_sol, errors] = inverse_kinematics_NR(T_desired, theta_guess, L);

    cout << "True joint angles: \n" << theta_true.transpose() << "\n\n";
    cout << "Recovered joint angles: \n" << theta_sol.transpose() << "\n\n";

    cout << "Error convergence:\n";
    for (size_t i = 0; i < errors.size(); ++i)
        cout << "Iter " << i << ": Error = " << errors[i] << endl;

    return 0;
}

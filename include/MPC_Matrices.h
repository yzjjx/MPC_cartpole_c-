#ifndef MPC_MATRICES
#define MPC_MATRICES

#include <Eigen/Dense>
#include <unsupported/Eigen/KroneckerProduct>// 使用kron，克罗内科积

using namespace std;

struct MPC_Matrices
{
    Eigen::MatrixXd E;
    Eigen::MatrixXd H;
};

MPC_Matrices MPC_cal(
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& F,
    const int& N);

#endif
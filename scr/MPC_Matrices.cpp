#include <Eigen/Dense>

using namespace std;

// 使用结构体，用来满足多输入多输出的函数需求
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
    const int& N)
{
    int n = A.rows();
    int p = B.cols();

    // 创建M矩阵
    Eigen::MatrixXd M_2(N*n,n);
    Eigen::MatrixXd M((N+1)*n,n);
    M << Eigen::MatrixXd::Identity(n,n),
         M_2;

    // 创建C矩阵
    Eigen::MatrixXd C((N+1)*n,N*p);

    // 创建临时矩阵
    Eigen::MatrixXd tmp = Eigen::MatrixXd::Identity(n,n);

    // 更新矩阵M和矩阵C,利用块操作来进行更新
    for(int i = 0;i<N;i++)
    {
        rows = i*n+
    }


}

#include <Eigen/Dense>
#include <iostream>

using namespace std;
// using namespace Eigen;

void main()
{
    // 第一步:定义状态空间矩阵
    Eigen::Matrix2d A;
    A << 1,  0.1,
         -1, 2;
    int n = A.rows();

    // 定义输入矩阵B，矩阵维度为n*p
    Eigen::Matrix2d B;
    B << 0.2,1,
         0.5,2;
    int p = B.cols();

    // 定义状态变量矩阵Q，矩阵维度为n*n
    Eigen::MatrixXd Q;
    Q << 1,0;
         0,1;
    // 定义终端误差矩阵F，矩阵维度为n*n
    Eigen::MatrixXd F;
    F << 1,0;
         0,1;
    // 定义控制变量矩阵R，矩阵维度为p*p
    Eigen::MatrixXd R;
    R << 1,0;
         0,1;

    // 定义仿真步
    int k_steps = 100;

    // 定义状态矩阵X_K
    Eigen::MatrixXd X_K(n,k_steps);

    // 定义初始状态变量的值，变量为n*1的向量1
    X_K(1,1) = 20;
    X_K(2,1) = -20;

    // 定义输入矩阵U_K,矩阵维度为p*k
    Eigen::MatrixXd U_K(p,k_steps);

    // 定义预测区间
    int N = 5;

    // 调用矩阵计算的函数
    
}
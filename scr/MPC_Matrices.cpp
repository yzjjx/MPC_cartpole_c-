// #include <Eigen/Dense>
// #include <unsupported/Eigen/KroneckerProduct>// 使用kron，克罗内科积

// using namespace std;

// // 使用结构体，用来满足多输入多输出的函数需求
// struct MPC_Matrices
// {
//     Eigen::MatrixXd E;
//     Eigen::MatrixXd H;
// };

#include "MPC_Matrices.h"

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
    Eigen::MatrixXd M_2 = Eigen::MatrixXd::Zero(N*n, n);
    Eigen::MatrixXd M((N+1)*n,n);
    M << Eigen::MatrixXd::Identity(n,n),
         M_2;

    // 创建C矩阵
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero((N+1)*n, N*p);

    // 创建临时矩阵
    Eigen::MatrixXd tmp = Eigen::MatrixXd::Identity(n,n);

    // 循环从0到 N-1，完全对应MATLAB的i = 1到N
    for(int i = 0; i < N; i++) 
    {
        // 计算当前要操作的矩阵块的起始行索引
        // MATLAB中i=1时是第n+1 行(索引 n),C++ 中i=0时也是索引 n。
        int row_start = (i + 1) * n; 

        // 更新矩阵C 
        // 第一部分：前 p 列写入 tmp * B
        // block(起始行, 起始列, 行数, 列数)
        C.block(row_start, 0, n, p) = tmp * B;
        
        // 第二部分：剩下的列，从上一块(row_start - n)拷贝过来
        int remaining_cols = C.cols() - p;
        if (remaining_cols > 0) {
            C.block(row_start, p, n, remaining_cols) = C.block(row_start - n, 0, n, remaining_cols);
        }
        
        // 更新矩阵 M
        tmp = A * tmp;
        M.middleRows(row_start, n) = tmp;
    }

    Eigen::MatrixXd Q_bar_1 = Eigen::MatrixXd::Identity(N,N);
    Eigen::MatrixXd kron_part = kroneckerProduct(Q_bar_1, Q);  
    int rows1 = kron_part.rows();
    int cols1 = kron_part.cols();
    int rows2 = F.rows();
    int cols2 = F.cols();
    // 创建全零矩阵
    Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(rows1 + rows2, cols1 + cols2);
    // 填入 kron 结果
    Q_bar.topLeftCorner(rows1, cols1) = kron_part;
    // 填入 F 块（右下角）
    Q_bar.bottomRightCorner(rows2, cols2) = F;
    Eigen::MatrixXd R_bar = kroneckerProduct(Q_bar_1,R);

    Eigen::MatrixXd G = M.transpose()*Q_bar*M;
    Eigen::MatrixXd E = M.transpose()*Q_bar*C;
    Eigen::MatrixXd H = C.transpose()*Q_bar*C + R_bar;

    // 返回结构体
    return {E, H};

}

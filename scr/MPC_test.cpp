#include <Eigen/Dense>
#include <iostream>
#include "MPC_Matrices.h"
#include "Prediction.h"
#include <fstream>

using namespace std;
// using namespace Eigen;

/**
 * 将 Eigen 矩阵保存为 CSV 文件
 * @param matrix 要保存的 Eigen 矩阵
 * @param filename 文件名 (例如 "output.csv")
 */
void saveEigenToCSV(const Eigen::MatrixXd& matrix, const std::string& filename) {
    const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", "\n");

    std::ofstream file(filename);
    
    if (file.is_open()) {
        file << matrix.format(CSVFormat);
        file.close();
        std::cout << "成功将数据保存至: " << filename << std::endl;
    } else {
        std::cerr << "无法打开文件 " << filename << std::endl;
    }
}

int main()
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
    Eigen::Matrix2d Q;
    Q << 1,0,
         0,1;
    // 定义终端误差矩阵F，矩阵维度为n*n
    Eigen::Matrix2d F;
    F << 1,0,
         0,1;
    // 定义控制变量矩阵R，矩阵维度为p*p
    Eigen::Matrix2d R;
    R << 0.1,0,
         0,0.1;

    // 定义仿真步
    int k_steps = 100;

    // 定义状态矩阵X_K
    Eigen::MatrixXd X_K = Eigen::MatrixXd::Zero(n,k_steps+1);

    // 定义初始状态变量的值，变量为n*1的向量1
    X_K(0,0) = 20;
    X_K(1,0) = -20;

    // 定义输入矩阵U_K,矩阵维度为p*k
    Eigen::MatrixXd U_K = Eigen::MatrixXd::Zero(p,k_steps);

    // 定义预测区间
    int N = 5;

    // 调用矩阵计算的函数
    MPC_Matrices result = MPC_cal(A,B,Q,R,F,N);
    Eigen::MatrixXd E = result.E;
    Eigen::MatrixXd H = result.H;

    // 计算每一步状态变量的值
    for(int i = 0;i < k_steps;i++)
    {
        U_K.col(i) = Prediction(X_K.col(i),E,H,N,p);
        X_K.col(i+1) = (A*X_K.col(i)+B*U_K.col(i));
    }

    saveEigenToCSV(X_K, "../output/X_K_result.csv");
    saveEigenToCSV(U_K, "../output/U_K_result.csv");

    cout<<"计算完成"<<endl;

    return 0;
}
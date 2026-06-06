#include <Eigen/Dense>
#include <iostream>
#include "MPC_Matrices.h"
#include "Prediction.h"
#include <fstream>
#include <cmath>

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
{   // 定义系统初始参数
    double Ts = 0.01;
    int n = 4;
    int p = 1;
    // 定义仿真步
    int k_steps = 3000;

    // 定义状态变量矩阵Q，矩阵维度为n*n
    Eigen::Matrix4d Q;
    Q << 10, 0, 0, 0,
         0,  1, 0, 0,
         0,  0, 200,0,
         0,  0, 0,  1;
    // 定义终端误差矩阵F，矩阵维度为n*n
    Eigen::Matrix4d F = Q;
    // 定义控制变量矩阵R，矩阵维度为p*p
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(p, p) * 0.1;

    // 定义状态矩阵X_K
    Eigen::MatrixXd X_K = Eigen::MatrixXd::Zero(n,k_steps+1);

    // 定义初始状态变量的值，变量为n*1的向量1
    X_K(0, 0) = 0.0;
    X_K(1, 0) = 0.0;
    X_K(2, 0) = 0.52;
    X_K(3, 0) = 0.0;

    // 定义输入矩阵U_K,矩阵维度为p*k
    Eigen::MatrixXd U_K = Eigen::MatrixXd::Zero(p,k_steps);

    // 定义预测区间
    int N = 30;

    // 创建连续矩阵和离散矩阵的容器
    Eigen::Matrix4d Ac = Eigen::Matrix4d::Zero();
    Eigen::MatrixXd Bc = Eigen::MatrixXd::Zero(n, p);
    Eigen::Matrix4d A_k;
    Eigen::MatrixXd B_k;

    // 计算每一步状态变量的值
    for(int i = 0;i < k_steps;i++)
    {
        // 定义MATLAB的矩阵公式计算结果
        double x_1 = X_K(0,i);
        double x_2 = X_K(1,i);
        double x_3 = X_K(2,i);
        double x_4 = X_K(3,i);

        // 围绕 u = 0 展开雅可比 (如你 MATLAB 中 u_linearize = 0)
        double u_linearize = 0.0;
        double cos_x3 = cos(x_3);
        double a11 = 0.0;
        double a12 = 1.0;
        double a13 = 0.0;
        double a14 = 0.0;
        double a21 = 0.0;
        double a22 = 533.0/(10000.0*((3969.0*cos_x3*cos_x3)/2500.0 - 32513.0/5000.0));
        double a23 = - ((33579.0*x_4*x_4*cos(x_3))/50000.0 + (3893589.0*cos(x_3)*cos(x_3))/250000.0 
                     - (3893589.0*sin(x_3)*sin(x_3))/250000.0)/((3969.0*cos(x_3)*cos(x_3))/2500.0 - 
                     32513.0/5000.0) - (3969.0*cos(x_3)*sin(x_3)*((33579.0*sin(x_3)*x_4*x_4)/50000.0 
                     + (533.0*u_linearize)/1000.0 - (533.0*x_2)/10000.0 + 
                     (3893589.0*cos(x_3)*sin(x_3))/250000.0))/(1250.0*pow(((3969.0*cos(x_3)*cos(x_3))/2500.0 
                     - 32513.0/5000.0),2));
        double a24 = -(33579.0*x_4*sin(x_3))/(25000.0*((3969.0*cos(x_3)*cos(x_3))/2500.0 - 32513.0/5000.0));
        double a31 = 0.0;
        double a32 = 0.0;
        double a33 = 0.0;
        double a34 = 1.0;
        double a41 = 0.0;
        double a42 = -(63.0*cos(x_3))/(500.0*((3969.0*cos(x_3)*cos(x_3))/2500.0 - 32513.0/5000.0));
        double a43 = (63.0*((59841.0*cos(x_3))/500.0 + (63.0*x_4*x_4*cos(x_3)*cos(x_3))/50.0 - 
                     (63.0*x_4*x_4*sin(x_3)*sin(x_3))/50.0 - u_linearize*sin(x_3) + 
                     (x_2*sin(x_3))/10.0))/(50.0*((3969.0*cos(x_3)*cos(x_3))/2500.0 - 32513.0/5000.0)) + 
                     (250047.0*cos(x_3)*sin(x_3)*((63.0*cos(x_3)*sin(x_3)*x_4*x_4)/50.0 + 
                     (59841.0*sin(x_3))/500.0 + u_linearize*cos(x_3) - 
                     (x_2*cos(x_3))/10.0))/(62500.0*pow(((3969.0*cos(x_3)*cos(x_3))/2500.0 - 32513.0/5000.0),2));
        double a44 = (3969.0*x_4*cos(x_3)*sin(x_3))/(1250.0*((3969.0*cos(x_3)*cos(x_3))/2500.0 - 32513.0/5000.0));

        double b11 = 0.0;
        double b21 = -533.0/(1000.0*((3969.0*cos(x_3)*cos(x_3))/2500.0 - 32513.0/5000.0));
        double b31 = 0.0;
        double b41 = (63.0*cos(x_3))/(50.0*((3969.0*cos(x_3)*cos(x_3))/2500.0 - 32513.0/5000.0));

        Ac(0,0) = a11;  Ac(0,1) = a12;    Ac(0,2) = a13;  Ac(0,3) = a14;
        Ac(1,0) = a21;  Ac(1,1) = a22;    Ac(1,2) = a23;  Ac(1,3) = a24;
        Ac(2,0) = a31;  Ac(2,1) = a32;    Ac(2,2) = a33;  Ac(2,3) = a34;
        Ac(3,0) = a41;  Ac(3,1) = a42;    Ac(3,2) = a43;  Ac(3,3) = a44;

        Bc(0,0) = b11;
        Bc(1,0) = b21;
        Bc(2,0) = b31;
        Bc(3,0) = b41;

        // 实时离散化
        A_k = Eigen::Matrix4d::Identity() + Ac * Ts;
        B_k = Bc * Ts;

        // 在循环内部更新MPC预测矩阵
        MPC_Matrices result = MPC_cal(A_k, B_k, Q, R, F, N);

        // 求解二次规划获得控制力u 
        U_K.col(i) = Prediction(X_K.col(i), result.E, result.H, N, p);
        double actual_u = U_K(0, i);

        // 更新真实物理状态(非线性前向欧拉)
        Eigen::Vector4d f_current;
        f_current(0) = x_2;
        f_current(1) = -((33579*sin(x_3)*x_4*x_4)/50000 + 
                       (533*actual_u)/1000 - (533*x_2)/10000 + 
                       (3893589*cos(x_3)*sin(x_3))/250000)/((3969*cos(x_3)*cos(x_3))/2500 -
                       32513/5000);
        f_current(2) = x_4;
        f_current(3) = (63*((63*cos(x_3)*sin(x_3)*x_4*x_4)/50 + 
                       (59841*sin(x_3))/500 + actual_u*cos(x_3) - 
                       (x_2*cos(x_3))/10))/(50*((3969*cos(x_3)*cos(x_3))/2500 - 32513/5000));

        X_K.col(i + 1) = X_K.col(i) + f_current * Ts;
    }

    saveEigenToCSV(X_K, "../output/X_K_result.csv");
    saveEigenToCSV(U_K, "../output/U_K_result.csv");

    cout<<"计算完成"<<endl;

    return 0;
}
#include "CartpoleMPC.h"
#include <cmath>
#include <algorithm>
#include "MPC_Matrices.h"
#include "Prediction.h"

// 实现构造函数，注意前缀 CartpoleMPC::
CartpoleMPC::CartpoleMPC() {
    Ts = 0.01;
    n = 4; p = 1; N = 30;
    Q << 10, 0, 0, 0,
         0,  1, 0, 0,
         0,  0, 200, 0,
         0,  0, 0,  1;
    F = Q;
    R = Eigen::MatrixXd::Identity(p, p) * 0.1;
}

// 输入当前状态，输出推力u
double CartpoleMPC::compute_control(Eigen::Vector4d current_state) {
    double x_1 = current_state(0);
    double x_2 = current_state(1);
    double x_3 = current_state(2);
    double x_4 = current_state(3);

    // 围绕u = 0展开雅可比
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

    Eigen::Matrix4d Ac;
    Eigen::MatrixXd Bc(n, p);

    Ac(0,0) = a11;  Ac(0,1) = a12;    Ac(0,2) = a13;  Ac(0,3) = a14;
    Ac(1,0) = a21;  Ac(1,1) = a22;    Ac(1,2) = a23;  Ac(1,3) = a24;
    Ac(2,0) = a31;  Ac(2,1) = a32;    Ac(2,2) = a33;  Ac(2,3) = a34;
    Ac(3,0) = a41;  Ac(3,1) = a42;    Ac(3,2) = a43;  Ac(3,3) = a44;

    Bc(0,0) = b11;
    Bc(1,0) = b21;
    Bc(2,0) = b31;
    Bc(3,0) = b41;

    // 离散化
    Eigen::Matrix4d A_k = Eigen::Matrix4d::Identity() + Ac * Ts;
    Eigen::MatrixXd B_k = Bc * Ts;

    // 计算预测矩阵 E 和 H
    MPC_Matrices result = MPC_cal(A_k, B_k, Q, R, F, N);

    // 求解 QP 得到输入序列 U
    Eigen::VectorXd U_K = Prediction(current_state, result.E, result.H, N, p);
    
    // 提取第一步控制量并限幅
    double actual_u = U_K(0);
    // actual_u = std::max(-50.0, std::min(50.0, actual_u));

    return actual_u;
}
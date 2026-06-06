#pragma once

#include <Eigen/Dense>

class CartpoleMPC {
private:
    // 内部变量
    double Ts;
    int n;
    int p;
    int N;
    
    Eigen::Matrix4d Q;
    Eigen::Matrix4d F;
    Eigen::MatrixXd R;

public:
    // 构造函数，用于初始化参数
    CartpoleMPC();

    // 核心控制函数，输入当前状态，输出控制推力
    double compute_control(Eigen::Vector4d current_state);
};
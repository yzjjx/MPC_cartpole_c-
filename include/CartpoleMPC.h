#ifndef CARTPOLE_MPC_H
#define CARTPOLE_MPC_H

#include <Eigen/Dense>

// private代表私有属性，即只能在对应的cpp文件被使用和修改，外部不能被使用和修改
// public为外部公开结构，外部代码只能通过调用这些接口让这个class工作
// CartpoleMPC();：为“构造函数”，名字必须和类名一模一样。它的作用是：当别人新建这个类的对象
// 时，负责把 Q, R 这些矩阵初始化好
class CartpoleMPC {
private:
    double Ts;
    int n, p, N;
    Eigen::Matrix4d Q, F;
    Eigen::MatrixXd R;
    double last_u;

public:
    CartpoleMPC(); // 构造函数声明
    double compute_control(Eigen::Vector4d current_state); // 控制函数声明
};

#endif
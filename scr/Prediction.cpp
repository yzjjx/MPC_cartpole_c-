// #include <qpOASES.hpp>
// #include <Eigen/Dense>
// #include <iostream>
#include "Prediction.h"

// 格式转换函数，将矩阵格式转换为QPOASES可以使用的格式
template<typename Derived>
std::vector<qpOASES::real_t> eigen2QpArray(const Eigen::MatrixBase<Derived>& mat)
{
    std::vector<qpOASES::real_t> vec;
    vec.reserve(mat.size());

    // qpOASES使用连续1的行优先的数据
    for(int j = 0; j < mat.rows(); j++)
    {
        for(int i = 0; i < mat.cols(); i++)
        {
            vec.push_back(static_cast<qpOASES::real_t>(mat(j,i)));
        }
    }
    return vec;
}


Eigen::VectorXd Prediction(
    const Eigen::VectorXd& x_k,
    const Eigen::MatrixXd& E,
    const Eigen::MatrixXd& H,
    int N,
    int p
)
{
    // U_k的数量为N*p
    int nV = N*p;
    // 构造二次规划数据
    Eigen::MatrixXd H_qp = H + H.transpose();
    H_qp += 1e-4 * Eigen::MatrixXd::Identity(nV, nV);
    Eigen::VectorXd g_qp = 2.0 * (E.transpose() * x_k);

    // 无约束求解
    Eigen::VectorXd lb = Eigen::VectorXd::Constant(nV,-50);
    Eigen::VectorXd ub = Eigen::VectorXd::Constant(nV, 50);

    // 转换为qpOASES使用的格式
    std::vector<qpOASES::real_t> H_arr = eigen2QpArray(H_qp);
    std::vector<qpOASES::real_t> g_arr = eigen2QpArray(g_qp);
    std::vector<qpOASES::real_t> lb_arr = eigen2QpArray(lb);
    std::vector<qpOASES::real_t> ub_arr = eigen2QpArray(ub);

    // 实例化求解问题
    qpOASES::QProblemB problem(nV);

    // 设置问题属性1
    qpOASES::Options options;
    options.printLevel=qpOASES::PL_NONE;
    problem.setOptions(options);

    // nWSR 是最大允许的划分树重构次数
    // 该变量传入后会被求解器内部修改为实际迭代次数
    int nWSR = 1000;

    // 初始化并求解二次规划问题
    // 注意：由于没有通用的线性矩阵约束 A*x，这里直接调用 5 个参数的 init 即可
    qpOASES::returnValue status = problem.init(
        H_arr.data(), 
        g_arr.data(), 
        lb_arr.data(), 
        ub_arr.data(), 
        nWSR
    );

    // 初始化返回值 u_k
    Eigen::VectorXd u_k = Eigen::VectorXd::Zero(p);

    // 检查求解器是否成功收敛
    if (status == qpOASES::SUCCESSFUL_RETURN) {
        // 定义存储优化结果的数组
        std::vector<qpOASES::real_t> U_opt(nV);
        // 获取最优原始解 (Primal Solution)
        problem.getPrimalSolution(U_opt.data());

        // 使用 Eigen::Map 将一维数组映射回 Eigen::VectorXd
        Eigen::VectorXd U_k = Eigen::Map<Eigen::VectorXd>(U_opt.data(), nV);

        // 提取整个预测时域序列 U_k 中的第一个控制步长动作
        u_k = U_k.head(p);
    } 
    else {
        std::cerr << "qpOASES 求解器未成功收敛！错误码: " << (int)status << std::endl;
    }

    return u_k;
    
}


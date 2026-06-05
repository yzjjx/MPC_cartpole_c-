#ifndef PREDICTION
#define PREDICTION

#include <qpOASES.hpp>
#include <Eigen/Dense>
#include <iostream>

template<typename Derived>
std::vector<qpOASES::real_t> eigen2QpArray(const Eigen::MatrixBase<Derived>& mat);

Eigen::VectorXd Prediction(
    const Eigen::VectorXd& x_k,
    const Eigen::MatrixXd& E,
    const Eigen::MatrixXd& H,
    int N,
    int p
);

#endif
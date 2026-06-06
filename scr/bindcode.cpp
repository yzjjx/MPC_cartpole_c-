#include <pybind11/pybind11.h>
#include <pybind11/eigen.h> 

// 包含.h头文件即可
#include "CartpoleMPC.h"  

namespace py = pybind11;
// mpc_controller为打包出来的动态库的名字，m代表模块本身，doc表示模块说明书，在python中
// 使用help(mpc_controller),就会显示出来这个doc
PYBIND11_MODULE(mpc_controller, m) {
    m.doc() = "C++ LTV-MPC Controller for Cartpole";

    py::class_<CartpoleMPC>(m, "CartpoleMPC")
        .def(py::init<>()) 
        .def("compute_control", &CartpoleMPC::compute_control);
}
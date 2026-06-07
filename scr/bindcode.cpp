#include <pybind11/pybind11.h>
#include <pybind11/eigen.h> 

// 包含.h头文件即可
#include "CartpoleMPC.h"  

namespace py = pybind11;
// MPC_test1为打包出来的动态库的名字，m代表模块本身，doc表示模块说明书，在python中
// 使用help(MPC_test1),就会显示出来这个doc
// 注意:这里的文件名字(MPC_test1)必须和CMakeLists.txt的文件名字是一样的，否则就会报错
PYBIND11_MODULE(MPC_test1, m) {
    m.doc() = "C++ LTV-MPC Controller for Cartpole";

    py::class_<CartpoleMPC>(m, "CartpoleMPC")
        .def(py::init<>()) 
        .def("compute_control", &CartpoleMPC::compute_control);
}
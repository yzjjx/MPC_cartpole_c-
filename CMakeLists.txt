cmake_minimum_required(VERSION 3.14)
project(MyCartpoleSim)

set(CMAKE_CXX_STANDARD 17)

# 引入克隆的 MuJoCo 源码目录 
add_subdirectory(/home/yyy/cpp_pac/mujoco build EXCLUDE_FROM_ALL)

# 编译主程序
add_executable(my_simulator test/xml_open.cpp)

# 链接 mujoco 和 glfw3
target_link_libraries(my_simulator PRIVATE mujoco glfw)
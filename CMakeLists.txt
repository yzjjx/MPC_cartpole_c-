cmake_minimum_required(VERSION 3.14)
project(MPCCartpoleSim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 寻找 Eigen3 库 
find_package(Eigen3 REQUIRED)

find_library(QPOASES_LIB 
    NAMES qpOASES 
    PATHS /home/yyy/cpp_pac/qpOASES/build/libs  
)

# 3. 添加头文件搜索路径
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

# 4. 编译主程序 (生成的可执行文件叫 MPC_test1)
add_executable(MPC_test1 
    scr/MPC_test.cpp
    scr/MPC_Matrices.cpp
    scr/Prediction.cpp
)

# 5. 链接第三方库到主程序
# 语法：target_link_libraries(程序名 库1 库2 ...)
target_link_libraries(MPC_test1
    PUBLIC
    # mujoco       
    Eigen3::Eigen 
    ${QPOASES_LIB}
)
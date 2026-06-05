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

# # 2. 引入克隆的 MuJoCo 源码目录 v
# # 注意：第二个参数是 cmake 构建 mujoco 时产生的临时文件存放地
# # 建议改名为 mujoco_build，防止和你当前项目的 build 文件夹产生冲突
# add_subdirectory(/home/yyy/cpp_pac/mujoco mujoco_build EXCLUDE_FROM_ALL)

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
# 语法：target_link_libraries(你的程序名 库1 库2 ...)
target_link_libraries(MPC_test1
    PUBLIC
    # mujoco         # 链接你通过 add_subdirectory 引入的 mujoco 核心物理引擎
    Eigen3::Eigen  # 链接 Eigen3 矩阵库
    ${QPOASES_LIB}
)
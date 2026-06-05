#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>

int main() {
    // 初始化 GLFW
    if (!glfwInit()) {
        std::cerr << "无法初始化 GLFW" << std::endl;
        return 1;
    }

    // 加载 MuJoCo 模型
    const char* filename = "../model/cartpole_no_names.xml";
    char error[1000] = "Could not load the model";
    mjModel* m = mj_loadXML(filename, nullptr, error, 1000);
    if (!m) {
        std::cerr << "加载失败: " << error << std::endl;
        return 1;
    }
    mjData* d = mj_makeData(m);

    // 创建 GLFW 窗口
    GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo_GLFW显示", NULL, NULL);
    glfwMakeContextCurrent(window);
    // 渲染帧率与显示器刷新率同步
    glfwSwapInterval(1); 

    // 初始化 MuJoCo 渲染数据结构
    mjvCamera cam;       mjv_defaultCamera(&cam);
    mjvOption opt;       mjv_defaultOption(&opt);
    mjvScene scn;        mjv_defaultScene(&scn);
    mjrContext con;      mjr_defaultContext(&con);

    // 创建场景和渲染上下文
    mjv_makeScene(m, &scn, 2000); 
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // 主循环
    while (!glfwWindowShouldClose(window)) {
        // --- 物理仿真 ---
        // 在这里加入控制算法
        mj_step(m, d);

        // --- 画面渲染 ---
        // 获取当前窗口大小
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        mjrRect viewport = {0, 0, width, height};

        // 更新场景并渲染
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // 交换缓冲区，处理窗口事件
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // 6. 释放所有资源
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();

    return 0;
}
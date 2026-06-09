import time
import csv
import sys
import os
import mujoco
import mujoco.viewer
import numpy as np

# 导入并初始化控制器
# 动态获取当前脚本的绝对路径，并拼接 build 目录
current_dir = os.path.dirname(os.path.abspath(__file__))
build_dir = os.path.join(current_dir, '../build')
sys.path.append(build_dir)

try:
    import MPC_test1
    print("成功导入 C++ 模块")
except ImportError as e:
    print(f"导入失败")
    sys.exit(1)

# 实例化MPC控制器
mpc = MPC_test1.CartpoleMPC()

# 1.加载模型和数据
xml_path = "../model/cartpole.xml"
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# 获取XML中设置的物理步长
dt = model.opt.timestep

# 系统初始扰动角度 (大约 30 度)
data.qpos[1] = 1.22

# 初始化缓存列表
input_log = []   # 存放输入数据
output_log = []  # 存放输出数据

print("MuJoCo仿真启动")

# 2.启动仿真循环
with mujoco.viewer.launch_passive(model, data) as viewer:
    
    # 只要窗口没手动关闭，就一直循环
    while viewer.is_running():
        step_start = time.time()
        theta_normalized = np.arctan2(np.sin(data.qpos[1]), np.cos(data.qpos[1]))

        current_state = np.array([
            data.qpos[0],  # 小车位置 x
            data.qvel[0],  # 小车速度 v 
            theta_normalized,  # 摆杆角度 theta
            data.qvel[1]   # 摆杆角速度 omega (dot_theta)
        ], dtype=np.float64)

        # 计算最优控制量
        u_opt = mpc.compute_control(current_state)
        
        data.ctrl[0] = u_opt

        # 采集数据
        input_log.append([data.time, data.ctrl[0]])

        output_log.append([
            data.time,
            data.qpos[0],
            theta_normalized,
            data.qvel[0],
            data.qvel[1]
        ])

        # 推进画面渲染
        mujoco.mj_step(model, data)
        viewer.sync()

        # 简单的锁帧操作，保证仿真速度与现实时间 1:1 同步
        time_until_next_step = dt - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

# 仿真结束后保存 CSV
print("\n正在保存数据")

# 确保输出目录存在
output_dir = os.path.abspath(os.path.join(current_dir, '../python_output'))
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# 写入输入文件
with open(os.path.join(output_dir, 'control_inputs.csv'), 'w', newline='', encoding='utf-8') as f:
    writer = csv.writer(f)
    writer.writerow(['time', 'ctrl_force'])
    writer.writerows(input_log)

# 写入输出文件
with open(os.path.join(output_dir, 'sys_outputs.csv'), 'w', newline='', encoding='utf-8') as f:
    writer = csv.writer(f)
    writer.writerow(['time', 'pos', 'angle', 'velocity', 'angular'])
    writer.writerows(output_log)

print("数据保存成功")
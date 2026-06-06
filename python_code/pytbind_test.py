import sys
import numpy as np

sys.path.append('../build')

import MPC_test1

print("成功导入 C++ 模块！")

# 1. 实例化你在 C++ 里写的类
mpc = MPC_test1.CartpoleMPC()

# 2. 伪造一个当前的小车状态 [x, v, theta, omega]
current_state = np.array([0.0, 0.0, 0.52, 0.0], dtype=np.float64)

# 3. 让 C++ 大脑计算推力
u_opt = mpc.compute_control(current_state)

print(f"C++ 计算出的最优推力 u = {u_opt}")
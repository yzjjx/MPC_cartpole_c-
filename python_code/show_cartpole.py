import time
import mujoco
import mujoco.viewer

# 1. 加载模型和数据
xml_path = "model/cartpole_no_names.xml"
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# 获取 XML 中设置的物理步长
dt = model.opt.timestep

# 2. 使用 with 语句启动被动渲染器 
with mujoco.viewer.launch_passive(model, data) as viewer:
    
    # 只要窗口没被手动关闭，就一直循环
    while viewer.is_running():
        step_start = time.time()

        # ==========================================
        # 在这里读取状态 (data.qpos, data.qvel)
        # 在这里写入控制指令 (data.ctrl)
        # ==========================================

        # 3. 物理引擎向前推进一步
        mujoco.mj_step(model, data)

        # 4. 将底层的物理变化同步到渲染画面上
        viewer.sync()

        # 5. 简单的锁帧操作，保证仿真速度与你的现实时间 1:1 同步
        # 如果不加这段 sleep，简单的模型会以几千帧/秒的速度狂奔
        time_until_next_step = dt - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
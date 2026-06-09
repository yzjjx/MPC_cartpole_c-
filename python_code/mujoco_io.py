# 用来实现给定初始输入，获取系统输出的功能，将输入和输出导出为csv文件，在关闭simulate界面的时候，csv
# 自动被保存
import time
import csv
import mujoco
import mujoco.viewer

# 1. 加载模型和数据
xml_path = "../model/cartpole_no_names.xml"
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# 获取 XML 中设置的物理步长
dt = model.opt.timestep

# 初始化缓存列表
# 存放输入数据
input_log = []
# 存放输出数据
output_log = [] 

# 2. 使用 with 语句启动被动渲染器 
with mujoco.viewer.launch_passive(model, data) as viewer:
    
    # 只要窗口没被手动关闭，就一直循环
    while viewer.is_running():
        step_start = time.time()

        # 采集输入数据
        input_log.append([data.time,data.ctrl[0]])

        # 采集输出数据
        output_log.append([
            data.time,
            data.qpos[0],
            data.qpos[1],
            data.qvel[0],
            data.qvel[1]
        ])

        # 3. 物理引擎向前推进一步
        mujoco.mj_step(model, data)

        # 4. 将底层的物理变化同步到渲染画面上
        viewer.sync()

        # 5. 简单的锁帧操作，保证仿真速度与你的现实时间 1:1 同步
        # 如果不加这段 sleep，简单的模型会以几千帧/秒的速度狂奔
        time_until_next_step = dt - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

# 在结束循环后，将数据全部写入
with open('../python_output/control_inputs.csv','w',newline='',encoding='utf-8') as f:
    writer = csv.writer(f)
    # 写入表头
    writer.writerow(['time','ctrl_force'])

    writer.writerows(input_log)

with open('../python_output/sys_outputs.csv','w',newline='',encoding='utf-8') as f:
    writer = csv.writer(f)
    # 写入表头
    writer.writerow(['time','pos','angle','velocity','angular'])

    writer.writerows(output_log)

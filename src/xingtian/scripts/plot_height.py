#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import matplotlib

# 设置中文字体为宋体
matplotlib.rcParams['font.sans-serif']=['SimSun']  # 使用宋体
matplotlib.rcParams['axes.unicode_minus']=False  # 解决负号显示问题

# 初始化数据列表
pitch_data = []
pos2_data = []
time_data = []
adjust_high_data = []
adjust_wheel_data = []
start_time = time.time()

# 回调函数
def angle_callback(data):
    global pitch_data, pos2_data, time_data, adjust_high_data, adjust_wheel_data
    pitch = data.data[2]  # 获取 pitch 角度
    pos2 = data.data[0]   # 获取 pos[2] 值
    adjust_wheel = data.data[3]
    adjust_high = data.data[4]
    current_time = time.time() - start_time
    time_data.append(current_time)
    # 更新数据
    adjust_high_data.append(adjust_high)
    adjust_wheel_data.append(adjust_wheel)
    pitch_data.append(-pitch)
    pos2_data.append(pos2)

# 初始化 ROS 节点
rospy.init_node('plot_node', anonymous=True)

# 订阅 ROS 话题
rospy.Subscriber("/xingtian/angle_ola1", Float64MultiArray, angle_callback)

# 创建图表
fig, ax = plt.subplots()

# 更新图表数据的函数
def update_plot(frame):
    ax.clear()
    # ax.plot(pitch_data, pos2_data, label='body height')
    ax.plot(pitch_data, adjust_wheel_data, label='adjust_wheel_data')
    ax.set_xlabel('slope(degree)', fontproperties='SimSun')
    ax.set_ylabel('adjust_wheel_data(mm)', fontproperties='SimSun')
    ax.set_title('The change of wheel distance with slope angle', fontproperties='SimSun')
    ax.legend(prop={'family': 'SimSun'})

# 设置动画
ani = FuncAnimation(fig, update_plot, blit=False)

# 显示图表
plt.show()

# 保持脚本运行
rospy.spin()
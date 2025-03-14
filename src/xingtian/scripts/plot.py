#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray
from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import WrenchStamped
# 初始化ROS节点
rospy.init_node('ros_plot', anonymous=True)

# 创建存储数据的列表
x_data = []
x1_data = []
x2_data = []
x3_data = []
y1_data = []  # 用于存储 oula2 数据
y2_data = []  # 用于存储 oula3 数据

# 创建存储数据的列表
data_list_topic1 = []  # 存储来自第一个话题的数据
data_list_topic2 = []  # 存储来自第二个话题的数据
time_list_topic1 = []  # 存储第一个话题的时间戳
time_list_topic2 = []  # 存储第二个话题的时间戳
# ROS消息回调函数
zqt2_force = []
zht2_force = []
yht2_force = []
yqt2_force = []
zqt2_data = []
zqt3_data = []
zht2_data = []
zht3_data = []
yht2_data = []
yht3_data = []
yqt2_data = []
yqt3_data = []
all_data_received = False
# def callback2(data):
#     # 将ROS消息中的数据添加到列表中
#     global data_list_topic1, time_list_topic1
#     data_list_topic1.append(data.data)
#     time_list_topic1.append(rospy.get_time())

# def callback3(data):
#     # 将ROS消息中的数据添加到列表中
#     global data_list_topic2, time_list_topic2
#     data_list_topic2.append(data.data)
#     time_list_topic2.append(rospy.get_time())
def callback1(data):
    # 将ROS消息中的数据添加到列表中
    x_data.append(rospy.get_time())   # 使用oula1数据作为横坐标
    y1_data.append(data.data[1]) 
    y2_data.append(data.data[2])
    check_data_received()
    pass
def callback_zqt2(data):
    x1_data.append(data.stamp.secs)
    zqt2_force.append(data.force.z)
    zqt2_data.append(data.torque.y)
    check_data_received()
    pass
def callback_zqt3(data):
    # zqt2_data[0] = data.force.z
    x1_data.append(data.stamp.secs)
    zqt3_data.append(data.torque.y)
    check_data_received()
    pass
def callback_zht2(data):
    x1_data.append(data.stamp.secs)
    zht2_force.append(data.force.z)
    zht2_data.append(data.torque.y)
    check_data_received()
    pass
def callback_zht3(data):
    zht3_data.append(data.torque.y)
    check_data_received()
    pass
def callback_yht2(data):
    x1_data.append(data.stamp.secs)
    yht2_force.append(data.force.z)
    yht2_data.append(data.torque.y)
    check_data_received()
    pass
def callback_yht3(data):
    yht3_data.append(data.torque.y)
    check_data_received()
    # print(data.torque.y)
    pass
def callback_yqt2(data):
    x1_data.append(data.stamp.secs)
    yqt2_force.append(data.force.z)
    yqt2_data.append(data.torque.y)
    check_data_received()
    pass
def callback_yqt3(data):
    yqt3_data.append(data.torque.y)
    check_data_received()
    pass
def check_data_received():
    global all_data_received
    # 检查每个数据列表的长度是否大于0
    if (
        len(x_data) > 0 and
        len(y1_data) > 0 and
        len(y2_data) > 0 and
        len(zqt2_force) > 0 and
        len(zqt2_data) > 0 and
        len(zqt3_data) > 0 and
        len(zht2_force) > 0 and
        len(zht2_data) > 0 and
        len(zht3_data) > 0 and
        len(yht2_force) > 0 and
        len(yht2_data) > 0 and
        len(yht3_data) > 0 and
        len(yqt2_force) > 0 and
        len(yqt2_data) > 0 and
        len(yqt3_data) > 0
    ):
        all_data_received = True
        update_plot()
# 订阅ROS消息
rospy.Subscriber("/xingtian/angle_ola1", Float64MultiArray, callback1)
# rospy.Subscriber("/xingtian/vmc_angle", Float64MultiArray, callback2)
# rospy.Subscriber("/xingtian/vmc_angle_p", Float64MultiArray, callback3)
rospy.Subscriber("/xingtian/torque_zqt2", WrenchStamped, callback_zqt2)
rospy.Subscriber("/xingtian/torque_zqt3", WrenchStamped, callback_zqt3)
rospy.Subscriber("/xingtian/torque_zht2", WrenchStamped, callback_zht2)
rospy.Subscriber("/xingtian/torque_zht3", WrenchStamped, callback_zht3)
rospy.Subscriber("/xingtian/torque_yht2", WrenchStamped, callback_yht2)
rospy.Subscriber("/xingtian/torque_yht3", WrenchStamped, callback_yht3)
rospy.Subscriber("/xingtian/torque_yqt2", WrenchStamped, callback_yqt2)
rospy.Subscriber("/xingtian/torque_yqt3", WrenchStamped, callback_yqt3)
# 创建图表
fig, axs = plt.subplots(2, 2)

# 更新图表数据的函数
def update_plot(frame):
    axs[0, 0].clear()
#     
    # axs[0, 0].plot(x_data, y1_data, label ='roll')
    # axs[0, 0].plot(x_data, y2_data, label='pitch')
    # axs[0, 0].set_xlabel('Time (s)')
    # axs[0, 0].set_ylabel('Angle(。)')
    # axs[0, 0].set_title('Plan 1: The change of Euler angles over time')

    axs[0, 1].clear()
    axs[0, 1].plot(x1_data, zqt2_force,label = 'zqt2_force')
    axs[0, 1].plot(x1_data, zht2_force,label = 'zht2_force')
    axs[0, 1].plot(x1_data, yht2_force,label = 'yht2_force')
    axs[0, 1].plot(x1_data, yqt2_force,label = 'yqt2_force')
    axs[0, 1].set_xlabel('Time (s)')
    axs[0, 1].set_ylabel('Force(N)')
    axs[0, 1].set_title('Plan 1: The variation of force along the z-axis over time.')

    # axs[1, 0].clear()
    # axs[1, 0].plot(x_data, zqt2_data,label = 'zqt2_torque')
    # axs[1, 0].plot(x_data, zht2_data,label = 'zht2_torque')
    # axs[1, 0].plot(x_data, yht2_data,label = 'yht2_torque')
    # axs[1, 0].plot(x_data, yqt2_data,label = 'yqt2_torque')
    # axs[1, 0].set_xlabel('Time (s)')
    # axs[1, 0].set_ylabel('Torque(Nm)')
    # axs[1, 0].set_title('Plan 1: Variation of torque in joint 2 over time')

    # axs[1, 1].clear()
    # axs[1, 1].plot(x_data, zqt3_data,label = 'zqt3_torque')
    # axs[1, 1].plot(x_data, zht3_data,label = 'zht3_torque')
    # axs[1, 1].plot(x_data, yht3_data,label = 'yht3_torque')
    # axs[1, 1].plot(x_data, yqt3_data,label = 'yqt3_torque')
    # axs[1, 1].set_xlabel('Time (s)')
    # axs[1, 1].set_ylabel('Torque(Nm)')
    # axs[1, 1].set_title('Plan 1: Variation of torque in joint 3 over time')
# while not all_data_received:
#     plt.pause(0.1) 
# 设置动画
ani = FuncAnimation(fig, update_plot, frames=None, blit=False)

# 显示图表
plt.show()





























# fig, ax = plt.subplots()

# def update_plot(frame):
#     ax.clear()
#     # 检查是否有数据
#     if time_list_topic1 and time_list_topic2:
#         # 将时间戳对齐，取最小和最大时间戳
#         start_time = min(time_list_topic1[0], time_list_topic2[0])
#         end_time = max(time_list_topic1[-1], time_list_topic2[-1])
#         ax.set_xlim(start_time, end_time)  # 设置横坐标范围为最早到最晚的时间戳
    
#     # 绘制第一个话题的数据
#     if data_list_topic1:
#         data_array_topic1 = np.array(data_list_topic1)
#         time_array_topic1 = np.array(time_list_topic1)
#         for i in range(data_array_topic1.shape[1]):
#             ax.plot(time_array_topic1, data_array_topic1[:, i], label=f"Topic 1 Variable {i+1}")
    
#     # 绘制第二个话题的数据
#     if data_list_topic2:
#         data_array_topic2 = np.array(data_list_topic2)
#         time_array_topic2 = np.array(time_list_topic2)
#         for i in range(data_array_topic2.shape[1]):
#             ax.plot(time_array_topic2, data_array_topic2[:, i], label=f"Topic 2 Variable {i+1}")
    
#     ax.legend()
#     ax.set_xlabel('Time (s)')
#     ax.set_ylabel('angle')
#     ax.set_title('compare vmc_angle and angle')
#     ax.grid(True)


# 更新图表数据的函数
# def update_plot(frame):
#     ax.clear()
#     ax.plot(x_data, y1_data, label='roll')
#     ax.plot(x_data, y2_data, label='pitch')
#     ax.set_xlabel('high(mm)')
#     ax.set_ylabel('angle')
#     ax.set_title('The Relationship between Height and Euler Angles')
#     ax.legend()
#     ax.grid(True)

# 设置动画
# ani = FuncAnimation(fig, update_plot, frames=None, blit=False)

# # 显示图表
# plt.show()

# 循环等待ROS消息
rospy.spin()

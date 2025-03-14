#! /usr/bin/env python3
#coding=utf-8
# 没用
import rospy
import numpy as np
import rpy_convert
import threading
import math
import time

# 全局变量
pi = math.pi
rotation = np.zeros((3,3))
zqt_x, zqt_z, zht_x, zht_z, yht_x, yht_z, yqt_x, yqt_z = 0, 0, 0, 0, 0, 0, 0, 0

def leg_vector_solve(rotation):
    # 定义轮腿车结构参数
    length = 678
    width_yao = 165
    width_foot = 400
    high = 300
    roll =0;pitch= 0;
    k = 0;p = 0;
    # 定义目标位姿 就是质心在地面的投影到质心的向量坐标。竖直方向的
    pos = np.mat([0.0, -30.0, 200]).T  # 示例期望位置
    if pos[1] != 0 :
        roll = math.atan2(pos[2], pos[1])
    if pos[0] != 0 :
        pitch = math.atan2(pos[2], pos[0])
    # print(roll)
    if roll != 0:
        k = width_foot/math.tan(roll);
    if pitch != 0 :
        p = length/math.tan(pitch);
    body_stru = np.mat([[length/2, width_yao/2, 0],      # 左前
                        [-length/2, width_yao/2, 0],     # 左后
                        [-length/2, -width_yao/2, 0],    # 右后
                        [length/2, -width_yao/2, 0]]).T  # 右前

    footpoint_struc = np.mat([[length/2, width_foot/2, 0],      # 左前
                             [-length/2, width_foot/2, 0],     # 左后
                             [-length/2, -width_foot/2, 0],    # 右后
                             [length/2, -width_foot/2, 0]]).T  # 右前

    tf_ab = np.mat(np.zeros((3, 4)))
    for i in range(4):
        tf_ab[:,i] = -pos - np.dot(rotation,body_stru[:,i])+footpoint_struc[:,i]
    # print(rotation)
    # 计算期望坐标
    global zqt_x, zqt_z, zht_x, zht_z, yht_x, yht_z, yqt_x, yqt_z
    zqt_x, zht_x, yht_x, yqt_x = tf_ab[0, 0], tf_ab[0, 1], tf_ab[0, 2], tf_ab[0, 3]
    zqt_z = -math.sqrt((tf_ab[1, 0]-200)**2 + tf_ab[2, 0]**2) + k/2 + p/2
    zht_z = -math.sqrt((tf_ab[1, 1]-200)**2 + tf_ab[2, 1]**2) + k/2 - p/2
    yht_z = -math.sqrt((tf_ab[1, 2]+200)**2 + tf_ab[2, 2]**2) - k/2 - p/2
    yqt_z = -math.sqrt((tf_ab[1, 3]+200)**2 + tf_ab[2, 3]**2) - k/2 + p/2
    print("左前、左后、右后、右前腿期望坐标：")
    print(zqt_x, zqt_z, zht_x, zht_z, yht_x, yht_z, yqt_x, yqt_z)

def imu_listener_thread():
    while not rospy.is_shutdown():
        rpy_convert.imu_listener()
        global rotation
        rotation = rpy_convert.rotationzyx
        print(rotation)
        # rospy.sleep(0.1)
def leg_vector_solve_thread():
    while not rospy.is_shutdown():
        leg_vector_solve(rotation)
        # print(rotation)
        rospy.sleep(0.1)  # 每隔一段时间执行一次

def self_stabilize():
    # rospy.init_node('self_stabilize_node', anonymous=True)  # 初始化ROS节点
    rpy_convert_thread = threading.Thread(target=imu_listener_thread)
    leg_thread = threading.Thread(target=leg_vector_solve_thread)
    rpy_convert_thread.start()
    leg_thread.start()
    # rospy.spin()  # 防止主线程退出
    
if __name__ == "__main__":
    try:
        self_stabilize()
    except rospy.ROSInterruptException:
        pass   
   
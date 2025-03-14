#!/usr/bin/env python3
# -*- coding: utf-8 -*
#调整机身重心姿态,以及机身高度！
import numpy as np
import math
# 机身参数
length = 418
width_yao = 165
width_foot = 400
def attitude_adjust(pitch,roll,high):    #pitch，roll，机身高度
    k = 0;p = 0;
    pos = np.mat([0,0,0]).T
    pos[0] = pitch
    pos[1] = roll
    pos[2] = high
    print(pitch,roll)
    if pos[1] != 0 :
      k = width_foot/math.tan(pos[1]);
    if pos[0] != 0 :
      p = length/math.tan(pos[0]);
    body_stru = np.mat([[length/2, width_yao/2, -36],      # 左前
                        [-length/2, width_yao/2, -36],     # 左后
                        [-length/2, -width_yao/2, -36],    # 右后
                        [length/2, -width_yao/2, -36]]).T  # 右前

    footpoint_struc = np.mat([[length/2, width_foot/2, 0],      # 左前
                             [-length/2, width_foot/2, 0],     # 左后
                             [-length/2, -width_foot/2, 0],    # 右后
                             [length/2, -width_foot/2, 0]]).T  # 右前
    tf_ab = np.mat(np.zeros((3, 4)))
    for i in range(4):
          # 在初始直立的姿态进行控制。相对于直立的姿态，缺点是不知道目前的状态，都是以原点为基础进行运算的。
          # 相当于pos给的是绝对值。
      tf_ab[:,i] = -pos - body_stru[:,i] + footpoint_struc[:,i]
    print(tf_ab)
    print(pos)
    global zqt_x, zqt_z, zht_x, zht_z, yht_x, yht_z, yqt_x, yqt_z
      # 在更新值时更新上一时刻的值
    initialized = False
    # 在初始化时给变量赋值
    if not initialized:
        zqt_x, zht_x, yht_x, yqt_x = tf_ab[0, 0]+209, tf_ab[0, 1]-209, tf_ab[0, 2]-209, tf_ab[0, 3]+209
        zqt_z = -math.sqrt( tf_ab[2, 0]**2) + k/2 + p/2
        zht_z = -math.sqrt( tf_ab[2, 1]**2) + k/2 - p/2
        yht_z = -math.sqrt( tf_ab[2, 2]**2) - k/2 - p/2
        yqt_z = -math.sqrt( tf_ab[2, 3]**2) - k/2 + p/2
        initialized = True
    zqt_x_prev, zht_x_prev, yht_x_prev, yqt_x_prev = zqt_x, zht_x, yht_x, yqt_x
    zqt_z_prev, zht_z_prev, yht_z_prev, yqt_z_prev = zqt_z, zht_z, yht_z, yqt_z
    threshold = 5
    # 更新当前时刻的值
    zqt_x = tf_ab[0, 0] + 209
    zht_x = tf_ab[0, 1] - 209
    yht_x = tf_ab[0, 2] - 209
    yqt_x = tf_ab[0, 3] + 209

    zqt_z = -math.sqrt(tf_ab[2, 0]**2) + k/2 + p/2
    zht_z = -math.sqrt(tf_ab[2, 1]**2) + k/2 - p/2
    yht_z = -math.sqrt(tf_ab[2, 2]**2) - k/2 - p/2
    yqt_z = -math.sqrt(tf_ab[2, 3]**2) - k/2 + p/2

    def smooth_coordinate(cur_val, new_val, threshold):
        if abs(new_val - cur_val) < threshold:
            return (cur_val+new_val)/2  # 如果新值和当前值之间的差距在阈值内，则保持当前值不变
        else:
            return new_val  # 如果差距超过阈值，则使用新值
        # 对每个坐标进行平滑处理
    zqt_z = max(min(zqt_z, -10), -300)
    zht_z = max(min(zht_z, -10), -300)
    yqt_z = max(min(yqt_z, -10), -300)
    yht_z = max(min(yht_z, -10), -300)
    zqt_x = smooth_coordinate(zqt_x_prev, zqt_x, threshold)
    zht_x = smooth_coordinate(zht_x_prev, zht_x, threshold)
    yht_x = smooth_coordinate(yht_x_prev, yht_x, threshold)
    yqt_x = smooth_coordinate(yqt_x_prev, yqt_x, threshold)
    zqt_z = smooth_coordinate(zqt_z_prev, zqt_z, threshold)
    zht_z = smooth_coordinate(zht_z_prev, zht_z, threshold)
    yht_z = smooth_coordinate(yht_z_prev, yht_z, threshold)
    yqt_z = smooth_coordinate(yqt_z_prev, yqt_z, threshold)
    return zqt_x,zqt_z,zht_x,zht_z,yht_x,yht_z,yqt_x,yqt_z

#! /user/bin/env/ python3
#coding=utf-8
import numpy as np
import math
pi = math.pi
# PD参数设置
# 这里是把二维矩阵分开用了。
KP_X=0.5     #X坐标控制器P环（初始，适用于trot）
KP_Z=0.5    #Z坐标控制器P环（初始，适用于trot）
KD_X=0.05     #X坐标控制器D环
KD_Z=0.05     #Z坐标控制器D环
# Vx=0;Vz=0  #暂时不引入速度反馈，但保留接口
# Xe Ze 是期望位置，X，Z是实际位置，
def cal_vmc_single_leg(Xe,Ze,X,Z,theta2,theta3,X_last = 0,Z_last = 0):
    # 定义雅可比矩阵# theta2 是大腿 theta3是小腿
    Jacobi = np.array([[-140*math.sin(theta2+theta3)-140*math.sin(theta2),-140*math.sin(theta2+theta3)],
          [140*math.cos(theta2+theta3)+140*math.cos(theta2),140*math.cos(theta2+theta3)]])
    # 雅可比矩阵转置
    Jacobi_T = np.transpose(Jacobi)
    # PD 控制器
    # 初始化速度，存储上个时刻的位置
    Vx = X-X_last
    Vz = Z-Z_last
    Fx=KP_X*((Xe-X)*0.001)+KD_X*(0-Vx)*0.001
    Fz=KP_Z*(Ze-Z)*0.001+KD_Z*(0-Vz)*0.001
    #保存上一个位置参数，用来在下一次循环求速度
    X_last=X
    Z_last=Z
    # 求力矩
    torque = np.zeros((2,1))
    torque = np.dot(Jacobi_T,np.array([[Fx],[Fz]]))
    return torque
    

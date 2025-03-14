#! /user/bin/env/ python3
#coding=utf-8
import numpy as np
import math
from std_msgs.msg import Float64

# def angle_insolve():
def angle_insolve(zqt_x,zqt_y,zht_x,zht_y,yht_x,yht_y,yqt_x,yqt_y):
    # 质心坐标系下的坐标。
    coordinate1 = np.array([zqt_x,zqt_y, 200.5,1]) 
    coordinate2 = np.array([zht_x,zht_y, 200.5,1])    
    coordinate3 = np.array([yht_x,yht_y,-200.5,1])   
    coordinate4 = np.array([yqt_x,yqt_y,-200.5,1]) 


    #     # 质心坐标系下的髋关节坐标 左前 左后 右后 右前
    #     # 实际坐标对应x z y % 转移到质心坐标系下 y的值是固定的，不用改。
    #     # 初始位置是 x 338.5 z-300
    # coordinate1 = np.array([ 353.5,-195, 200.5,1]) # 
    # coordinate2 = np.array([-326.5,-195, 200.5,1])   # 
    # coordinate3 = np.array([-326.5,-195,-200.5,1])   # 
    # coordinate4 = np.array([ 353.5,-195,-200.5,1])   #

    # 四个平移矩阵 x z y
    # x轴正向为前方
    # 左前腿                    ---------------------------------------------------
    T01 = np.array([[1, 0, 0,  209], [0, 1, 0, -35.91], [0, 0, 1,  82.5], [0, 0, 0, 1]])
    # 左后腿
    T02 = np.array([[1, 0, 0, -209], [0, 1, 0, -35.91], [0, 0, 1,  82.5], [0, 0, 0, 1]])
    # 右后腿
    T03 = np.array([[1, 0, 0, -209], [0, 1, 0, -35.91], [0, 0, 1,  -82.5], [0, 0, 0, 1]])
    # 右前腿
    T04 = np.array([[1, 0, 0,  209], [0, 1, 0, -35.91], [0, 0, 1,  -82.5], [0, 0, 0, 1]])

    # 髋关节下的坐标系
    coor1 = np.dot(np.linalg.inv(T01), coordinate1)
    coor2 = np.dot(np.linalg.inv(T02), coordinate2)
    coor3 = np.dot(np.linalg.inv(T03), coordinate3)
    coor4 = np.dot(np.linalg.inv(T04), coordinate4)

    # 求解过程 theta3 赋值x y
    x1, y1 = coor1[:2]
    x2, y2 = coor2[:2]
    x3, y3 = coor3[:2]
    x4, y4 = coor4[:2]

    # 杆长 l1 l2
    l1 = 140
    l2 = 140

    # 判断是否有解
    a1 = (x1*x1 + y1*y1 - l1*l1 - l2*l2) / (2*l1*l2)
    a2 = (x2*x2 + y2*y2 - l1*l1 - l2*l2) / (2*l1*l2)
    a3 = (x3*x3 + y3*y3 - l1*l1 - l2*l2) / (2*l1*l2)
    a4 = (x4*x4 + y4*y4 - l1*l1 - l2*l2) / (2*l1*l2)

    # 判断是否有解
    if np.abs(a1) > 1 or np.abs(a2) > 1 or np.abs(a3) > 1 or np.abs(a4) > 1:
        print("无法到达该位置")
        exit()

    # 求解 theta3
    theta13 = np.zeros((2, 1))
    theta13[0, 0] = np.arccos(a1)
    theta13[1, 0] = -np.arccos(a1)

    # 左后腿
    theta23 = np.zeros((2, 1))
    theta23[0, 0] = np.arccos(a2)
    theta23[1, 0] = -np.arccos(a2)

    # 右后腿
    theta33 = np.zeros((2, 1))
    theta33[0, 0] = np.arccos(a3)
    theta33[1, 0] = -np.arccos(a3)

    # 右前腿
    theta43 = np.zeros((2, 1))
    theta43[0, 0] = np.arccos(a4)
    theta43[1, 0] = -np.arccos(a4)

    # 求解 theta2

    # 左前腿
    k11 = l1 + l2 * np.cos(theta13[0, 0])
    k12 = l2 * np.sin(theta13[0, 0])
    gama1 = np.arctan2(k12, k11)
    theta12_1 = np.arctan2(y1, x1) - gama1

    k11 = l1 + l2 * np.cos(theta13[1, 0])
    k12 = l2 * np.sin(theta13[1, 0])
    gama1 = np.arctan2(k12, k11)
    theta12_2 = np.arctan2(y1, x1) - gama1

    # 左后腿
    k21 = l1 + l2 * np.cos(theta23[0, 0])
    k22 = l2 * np.sin(theta23[0, 0])
    gama2 = np.arctan2(k22, k21)
    theta22_1 = np.arctan2(y2, x2) - gama2

    k21 = l1 + l2 * np.cos(theta23[1, 0])
    k22 = l2 * np.sin(theta23[1, 0])
    gama2 = np.arctan2(k22, k21)
    theta22_2 = np.arctan2(y2, x2) - gama2

    # 右后腿
    k31 = l1 + l2 * np.cos(theta33[0, 0])
    k32 = l2 * np.sin(theta33[0, 0])
    gama3 = np.arctan2(k32, k31)
    theta32_1 = np.arctan2(y3, x3) - gama3

    k31 = l1 + l2 * np.cos(theta33[1, 0])
    k32 = l2 * np.sin(theta33[1, 0])
    gama3 = np.arctan2(k32, k31)
    theta32_2 = np.arctan2(y3, x3) - gama3

    # 右前腿
    k41 = l1 + l2 * np.cos(theta43[0, 0])
    k42 = l2 * np.sin(theta43[0, 0])
    gama4 = np.arctan2(k42, k41)
    theta42_1 = np.arctan2(y4, x4) - gama4

    k41 = l1 + l2 * np.cos(theta43[1, 0])
    k42 = l2 * np.sin(theta43[1, 0])
    gama4 = np.arctan2(k42, k41)
    theta42_2 = np.arctan2(y4, x4) - gama4

    # 输出角度 theta2大腿
    
    ftheta = np.zeros((2, 8))
    ftheta[0, 0] = np.degrees(theta12_1) + 90
    ftheta[1, 0] = np.degrees(theta12_2) + 90

    ftheta[0, 2] = np.degrees(theta22_1) + 90
    ftheta[1, 2] = np.degrees(theta22_2) + 90

    ftheta[0, 4] = np.degrees(theta32_1) + 90
    ftheta[1, 4] = np.degrees(theta32_2) + 90

    ftheta[0, 6] = np.degrees(theta42_1) + 90
    ftheta[1, 6] = np.degrees(theta42_2) + 90

    # theta3 小腿
    ftheta[0, 1] = np.degrees(theta13[0, 0])
    ftheta[1, 1] = np.degrees(theta13[1, 0])

    ftheta[0, 3] = np.degrees(theta23[0, 0])
    ftheta[1, 3] = np.degrees(theta23[1, 0])

    ftheta[0, 5] = np.degrees(theta33[0, 0])
    ftheta[1, 5] = np.degrees(theta33[1, 0])

    ftheta[0, 7] = np.degrees(theta43[0, 0])
    ftheta[1, 7] = np.degrees(theta43[1, 0])

    # 输出角度
    print("theta2(大腿) theta1(小腿)的两组解为左前、左后、右后、右前：")
    print(ftheta[1,0],ftheta[1,1])
    print(ftheta[0,2],ftheta[0,3])
    print(ftheta[0,4],ftheta[0,5])
    print(ftheta[1,6],ftheta[1,7])
  
    return ftheta;

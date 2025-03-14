#! /user/bin/env/ python3
#coding=utf-8
# 没用
import rospy
import numpy as np
import math
import force_VMC
from sensor_msgs.msg import JointState
pi =math.pi
# from self_stabilized import self_stabilize
import self_stabilized
import time
import threading
import queue
# 转移到质心坐标系下的平移矩阵
T1 = ([[1, 0, 0 ,338.64],[0, 1,0 ,-35.91],[0, 0, 1, -82.5],[0, 0, 0, 1]]);
T2 = ([[1, 0, 0 ,-338.64],[0, 1,0 ,-35.91],[0, 0, 1, -82.5],[0, 0, 0, 1]]);
T3 = ([[1, 0, 0 ,-338.64],[0, 1,0 ,-35.91],[0, 0, 1, 82.5],[0, 0, 0, 1]]);
T4 = ([[1, 0, 0 ,338.64],[0, 1,0 ,-35.91],[0, 0, 1, 82.5],[0, 0, 0, 1]]);
# zqt_coor, zht_coor, yht_coor, yqt_coor = np.zeros((3,)),np.zeros((3,)),np.zeros((3,)),np.zeros((3,))
# zqt_theta,zht_theta,yht_theta,yqt_theta = [0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0]
def jointstate_callback(data):
    global zqt_theta
    zqt_theta = [0.0,0.0]
    global zht_theta
    zht_theta = [0.0,0.0]
    global yht_theta
    yht_theta = [0.0,0.0]
    global yqt_theta
    yqt_theta = [0.0,0.0]
    # 获取所有关节的名称
    joint_names = data.name
    # 获取所有关节的位置信息
    joint_positions = data.position
    # 输出每个关节的名称和位置信息
    # for name, position in zip(joint_names, joint_positions):
    #     # rospy.loginfo("Joint: %s, Position: %.4f", name, position)
    #     print("")
    # 假设我们想要获取特定关节的位置信息，比如关节 "joint1"
    if "zqt_joint2" in joint_names:
        joint1_index = joint_names.index("zqt_joint2")
        zqt_joint2_position = joint_positions[joint1_index]
        zqt_theta[0] = -(zqt_joint2_position)*180/pi
    if "zqt_joint3" in joint_names:
        joint1_index = joint_names.index("zqt_joint3")
        zqt_joint3_position = joint_positions[joint1_index]
        zqt_theta[1] = -float((zqt_joint3_position-zqt_joint2_position)*180/pi)-90;
    if "zht_joint2" in joint_names:
        joint1_index = joint_names.index("zht_joint2")
        zht_joint2_position = joint_positions[joint1_index]
        zht_theta[0] = -zht_joint2_position*180/pi;
    if "zht_joint3" in joint_names:
        joint1_index = joint_names.index("zht_joint3")
        zht_joint3_position = joint_positions[joint1_index]
        zht_theta[1] = -float((zht_joint3_position-zht_joint2_position)*180/pi)-90;
    if "yht_joint2" in joint_names:
        joint1_index = joint_names.index("yht_joint2")
        yht_joint2_position = joint_positions[joint1_index]
        yht_theta[0] = -yht_joint2_position*180/pi;
    if "yht_joint3" in joint_names:
        joint1_index = joint_names.index("yht_joint3")
        yht_joint3_position = joint_positions[joint1_index]
        yht_theta[1] = -float((yht_joint3_position-yht_joint2_position)*180/pi)-90;
    if "yqt_joint2" in joint_names:
        joint1_index = joint_names.index("yqt_joint2")
        yqt_joint2_position = joint_positions[joint1_index]
        yqt_theta[0] = -yqt_joint2_position*180/pi;
    if "yqt_joint3" in joint_names:
        joint1_index = joint_names.index("yqt_joint3")
        yqt_joint3_position = joint_positions[joint1_index]
        yqt_theta[1] = -float((yqt_joint3_position+yqt_joint2_position)*180/pi)-90;
    print("---------------------------------------------------------------")
    print("左前 左后 右后 右前 theta1 theta2:")
    global zqt_coor,zht_coor,yht_coor,yqt_coor
    T04 = forward_solver_mdh(zqt_theta)
    #print(T04)
    # print(zqt_theta)
    zqt_coor =np.dot(T1,T04[:,3])
    print(zqt_coor)
    T04 = forward_solver_mdh(zht_theta)
    zht_coor = np.dot(T2,T04[:,3])
    print(zht_coor)
    # print(zht_theta)
    T04 = forward_solver_mdh(yht_theta)
    yht_coor = np.dot(T3,T04[:,3])
    print(yht_coor)
    # print(yht_theta)
    T04 = forward_solver_mdh(yqt_theta)
    yqt_coor =np.dot(T4,T04[:,3])
    print(yqt_coor)

    # print(yqt_theta)
    # return zqt_theta,zht_theta,yht_theta,yqt_theta
    
def forward_solver_mdh(theta):
    # 建立机器人DH参数
    DH_JXB = np.array([[0, 0, 0, 0],
                       [0, 0, 105, 0],
                       [0, 140, -16, 0],
                       [0, 140, 29, 0]])
    d = DH_JXB[:, 2]
    a = DH_JXB[:, 1]
    alpha = DH_JXB[:, 0] * pi / 180
    offset = np.array([0, 0, 0, 0])
    
    # 设置初始关节角度并转换为弧度制
    theta1 = 0;theta4 = 0;
    # theta2 是大腿
    # theta = (theta + offset) * pi / 180
    theta3 = theta[0]*pi/180;
    theta2 = theta[1]*pi/180;
    # theta2, theta3 = theta
    
    T01 = np.array([[np.cos(theta1), -np.sin(theta1), 0, a[0]],
                    [np.sin(theta1) * np.cos(alpha[0]), np.cos(theta1) * np.cos(alpha[0]), -np.sin(alpha[0]), -d[0] * np.sin(alpha[0])],
                    [np.sin(theta1) * np.sin(alpha[0]), np.cos(theta1) * np.sin(alpha[0]), np.cos(alpha[0]), d[0] * np.cos(alpha[0])],
                    [0, 0, 0, 1]])
    
    T12 = np.array([[np.cos(theta2), -np.sin(theta2), 0, a[1]],
                    [np.sin(theta2) * np.cos(alpha[1]), np.cos(theta2) * np.cos(alpha[1]), -np.sin(alpha[1]), -d[1] * np.sin(alpha[1])],
                    [np.sin(theta2) * np.sin(alpha[1]), np.cos(theta2) * np.sin(alpha[1]), np.cos(alpha[1]), d[1] * np.cos(alpha[1])],
                    [0, 0, 0, 1]])
    
    T23 = np.array([[np.cos(theta3), -np.sin(theta3), 0, a[2]],
                    [np.sin(theta3) * np.cos(alpha[2]), np.cos(theta3) * np.cos(alpha[2]), -np.sin(alpha[2]), -d[2] * np.sin(alpha[2])],
                    [np.sin(theta3) * np.sin(alpha[2]), np.cos(theta3) * np.sin(alpha[2]), np.cos(alpha[2]), d[2] * np.cos(alpha[2])],
                    [0, 0, 0, 1]])
    
    T34 = np.array([[np.cos(theta4), -np.sin(theta4), 0, a[3]],
                    [np.sin(theta4) * np.cos(alpha[3]), np.cos(theta4) * np.cos(alpha[3]), -np.sin(alpha[3]), -d[3] * np.sin(alpha[3])],
                    [np.sin(theta4) * np.sin(alpha[3]), np.cos(theta4) * np.sin(alpha[3]), np.cos(alpha[3]), d[3] * np.cos(alpha[3])],
                    [0, 0, 0, 1]])
    
    # 计算末端位置
    T04 = np.dot(np.dot(np.dot(T01, T12), T23), T34)
    # 返回末端的旋转矩阵
    return T04[:4, :4]
def angle_forward():
    rospy.init_node("angle_forward_solve");
    angle_feedback = rospy.Subscriber("/xingtian/joint_states",JointState,jointstate_callback)
    # time.sleep(0.5)
# 求解VMC
def force_VMC_solve():
    global zqt_coor,zht_coor,yht_coor,yqt_coor
# Xe Ze 是期望位置，X，Z是实际位置，
    print("------------------------------------------")
    print(zqt_theta)
    rospy.logwarn("torque计算---------------------------------------")
    print(self_stabilized.zqt_x)
# def cal_vmc_single_leg(Xe,Ze,X,Z,theta2,theta3,X_last = 0,Z_last = 0):
    zqt_torque = force_VMC.cal_vmc_single_leg(self_stabilized.zqt_x,self_stabilized.zqt_z,zqt_coor[0],zqt_coor[1],zqt_theta[1],zqt_theta[0],0,0)
    zht_torque = force_VMC.cal_vmc_single_leg(self_stabilized.zht_x,self_stabilized.zht_z,zht_coor[0],zht_coor[1],zht_theta[1],zht_theta[0],0,0)
    yht_torque = force_VMC.cal_vmc_single_leg(self_stabilized.yht_x,self_stabilized.yht_z,yht_coor[0],yht_coor[1],yht_theta[1],yht_theta[0],0,0)
    yqt_torque = force_VMC.cal_vmc_single_leg(self_stabilized.yqt_x,self_stabilized.yqt_z,yqt_coor[0],yqt_coor[1],yqt_theta[1],yqt_theta[0],0,0)
    print(zqt_torque)
    print()
    print(zht_torque)
    print()
    print(yht_torque)
    print()
    print(yqt_torque)
    return zqt_torque,zht_torque,yht_torque,yqt_torque   
# def stabilize_thread():
    # while True:
        # self_stabilized.self_stabilize()
# def angle_forward_thread():
    # while True:
        # angle_forward()

if __name__ =='__main__':
  
    while True:
        # angle_forward()
        # time.sleep(1)
        # self_stabilized.self_stabilize()
        # time.sleep(1)
        zqt_torque,zht_torque,yht_torque,yqt_torque = force_VMC_solve()
        
        # time.sleep(1)
        # rospy.spin()
    
    

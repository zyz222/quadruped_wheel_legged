#! /user/bin/env/ python3
#coding=utf-8
import rospy
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math
import statistics
import matplotlib.pyplot as plt
pi = math.pi
# 导入应该导入的是函数
from angle_insolve import angle_insolve
from geometry_msgs.msg import WrenchStamped
global max_torque
max_torque = np.array([50.0,50.0])
# 初始化一个两行四列数组，存储最终角度
global ftheta_addmition
ftheta_addmition = [[0.0] * 4 for _ in range(2)]
# joint_callback 在这里得到的是实际角度 zqt zht yht yqt
def calculate_omega(prev_theta, curr_theta, dt):
    omega = [0.0, 0.0]
    if len(prev_theta) == 2 and len(curr_theta) == 2:
        omega[0] = (curr_theta[0] - prev_theta[0]) / dt
        omega[1] = (curr_theta[1] - prev_theta[1]) / dt
    return omega
def calculate_alpha(prev_omega, curr_omega, dt):
    alpha = [0.0, 0.0]
    if len(prev_omega) == 2 and len(curr_omega) == 2:
        alpha[0] = (curr_omega[0] - prev_omega[0]) / dt
        alpha[1] = (curr_omega[1] - prev_omega[1]) / dt
    return alpha
# 这里来处理传感器得到的扭矩值
topic_data = {}
# 对传感器数据进行一个中值滤波，均值滤波会出现错误的结果
# 定义滤波器的窗口大小
WINDOW_SIZE = 5
def median_filter(values):
    # 如果历史数据长度超过窗口大小，则移除最早的数据
    if len(values) > WINDOW_SIZE:
        values.pop(0)
    # 计算历史数据的中值
    median_value = statistics.median(values)
    return median_value
# 接收传感器数据
def torque_callback(data,topic_name):
    global topic_data
    torque_values = [0.0]*8
    # force = data.wrench.force
    torque = data.wrench.torque
    topic_data[topic_name]=torque.y
    # 0-7 左左右右
    torque_values[0] = [topic_data["/torque_zqt2"] for topic in topics]
    torque_values[1] = [topic_data["/torque_zqt3"] for topic in topics]
    torque_values[2] = [topic_data["/torque_zht2"] for topic in topics]
    torque_values[3] = [topic_data["/torque_zht3"] for topic in topics]
    torque_values[4] = [topic_data["/torque_yht2"] for topic in topics]
    torque_values[5] = [topic_data["/torque_yht3"] for topic in topics]
    torque_values[6] = [topic_data["/torque_yqt2"] for topic in topics]
    torque_values[7] = [topic_data["/torque_yqt3"] for topic in topics]
    # 对所有关节的扭矩值进行中值滤波并返回滤波后的值
    global smoothed_torque
    smoothed_torque  = [0.0]*8
    smoothed_torque[0] = median_filter(torque_values[0])
    smoothed_torque[1] = median_filter(torque_values[1])
    smoothed_torque[2] = median_filter(torque_values[2])
    smoothed_torque[3] = median_filter(torque_values[3])
    smoothed_torque[4] = median_filter(torque_values[4])
    smoothed_torque[5] = median_filter(torque_values[5])
    smoothed_torque[6] = median_filter(torque_values[6])
    smoothed_torque[7] = median_filter(torque_values[7])
    
def addmition_zq(vmc_torque):
    # 设定初始条件 这里应该是反馈回来的数值，joint_callback得到！
    global dt,ftheta_addmition
    dt = 1  # 时间步长
    # 这个应该是利用VMC算法给的扭矩。目前VMC还没写，暂定手动给定。
    global given_control_input
    given_control_input = np.array([0,0])  # 给定的控制输入
    given_control_input = vmc_torque  # vmc输入扭矩控制输入
    global M,D,K,tolerance,max_iterations
    M = np.array([[0.0, 0.0], [0.0, 0.0]])  # 惯性矩阵
    D = np.array([[0.5, 0.0], [0.0, 0.5]])  # 阻尼矩阵
    K = np.array([[300.0, 0.0], [0.0, 300.0]])  # 刚度矩阵
    tolerance = 0.05  # 终止条件：控制误差的容忍度
    max_iterations = 2  # 最大迭代次数
    ftheta_addmition[0][0],ftheta_addmition[1][0] = addmition_zqt(zqt_theta,zqt_omega,zqt_alpha)
    return ftheta_addmition[0][0],ftheta_addmition[1][0]
def addmition_zh(vmc_torque):
    # 设定初始条件 这里应该是反馈回来的数值，joint_callback得到！
    global dt,ftheta_addmition
    dt = 1  # 时间步长
    # 这个应该是利用VMC算法给的扭矩。目前VMC还没写，暂定手动给定。
    global given_control_input
    given_control_input = np.array([0,0])  # 给定的控制输入
    given_control_input = vmc_torque  # vmc输入扭矩控制输入
    global M,D,K,tolerance,max_iterations
    M = np.array([[0.0, 0.0], [0.0, 0.0]])  # 惯性矩阵
    D = np.array([[0.5, 0.0], [0.0, 0.5]])  # 阻尼矩阵
    K = np.array([[300.0, 0.0], [0.0, 300.0]])  # 刚度矩阵
    tolerance = 0.05  # 终止条件：控制误差的容忍度
    max_iterations = 2  # 最大迭代次数
    ftheta_addmition[0][1],ftheta_addmition[1][1] = addmition_zht(zht_theta,zht_omega,zht_alpha)
    return ftheta_addmition[0][1],ftheta_addmition[1][1]
def addmition_yh(vmc_torque):
    # 设定初始条件 这里应该是反馈回来的数值，joint_callback得到！
    global dt,ftheta_addmition
    dt = 1  # 时间步长
    # 这个应该是利用VMC算法给的扭矩。目前VMC还没写，暂定手动给定。
    global given_control_input
    given_control_input = np.array([0,0])  # 给定的控制输入
    given_control_input = vmc_torque  # vmc输入扭矩控制输入
    global M,D,K,tolerance,max_iterations
    M = np.array([[0.0, 0.0], [0.0,0.0]])  # 惯性矩阵
    D = np.array([[0.5, 0.0], [0.0, 0.5]])  # 阻尼矩阵
    K = np.array([[300.0, 0.0], [0.0, 300.0]])  # 刚度矩阵
    tolerance = 0.05  # 终止条件：控制误差的容忍度
    max_iterations = 2  # 最大迭代次数
    ftheta_addmition[0][2],ftheta_addmition[1][2] = addmition_yht(yht_theta,yht_omega,yht_alpha)
    return ftheta_addmition[0][2],ftheta_addmition[1][2]
def addmition_yq(vmc_torque):
    # 设定初始条件 这里应该是反馈回来的数值，joint_callback得到！
    global dt,ftheta_addmition
    dt = 1  # 时间步长
    # 这个应该是利用VMC算法给的扭矩。目前VMC还没写，暂定手动给定。
    global given_control_input
    given_control_input = np.array([0,0])  # 给定的控制输入
    given_control_input = vmc_torque  # vmc输入扭矩控制输入
    global M,D,K,tolerance,max_iterations
    M = np.array([[0.0, 0.0], [0.0, 0.0]])  # 惯性矩阵
    D = np.array([[0.5, 0.0], [0.0, 0.5]])  # 阻尼矩阵
    K = np.array([[500.0, 0.0], [0.0, 500.0]])  # 刚度矩阵
    tolerance = 0.05  # 终止条件：控制误差的容忍度
    max_iterations = 2  # 最大迭代次数
    ftheta_addmition[0][3],ftheta_addmition[1][3] = addmition_yqt(yqt_theta,yqt_omega,yqt_alpha)
    return ftheta_addmition[0][3],ftheta_addmition[1][3]

def addmition_zqt(initial_theta,initial_theta_dot,initial_theta_double_dot):
        # 迭代计算
    global dt,ftheta_addmition
    global M,D,K,tolerance,max_iterations
    global given_control_input,theta_zqt_p
    for i in range(max_iterations):
        control_input = given_control_input
            # 更新关节状态
        initial_theta = (np.linalg.inv(K).dot(control_input-D.dot(initial_theta_dot)-M.dot(initial_theta_double_dot)))+theta_zqt_p
        # 输出最终的关节角度
        ftheta_addmition[0][0] = float(initial_theta[0])  #大腿
        ftheta_addmition[1][0] = float(initial_theta[1])  #小腿
    return ftheta_addmition[0][0],ftheta_addmition[1][0]
    # print("左前腿最终关节角度(小腿、大腿):", initial_theta)
def addmition_zht(initial_theta,initial_theta_dot,initial_theta_double_dot):
        # 迭代计算
    global dt,ftheta_addmition
    global M,D,K,tolerance,max_iterations
    global given_control_input,theta_zht_p
    for i in range(max_iterations):
        control_input = given_control_input
            # 更新关节状态
        initial_theta = (np.linalg.inv(K).dot(control_input-D.dot(initial_theta_dot)-M.dot(initial_theta_double_dot)))+theta_zht_p
        # 输出最终的关节角度
        ftheta_addmition[0][1] = initial_theta[0]
        ftheta_addmition[1][1] = initial_theta[1]
    return ftheta_addmition[0][1],ftheta_addmition[1][1]
    # print("左后腿最终关节角度(小腿、大腿):", initial_theta)
def addmition_yht(initial_theta,initial_theta_dot,initial_theta_double_dot):
        # 迭代计算
    global dt,ftheta_addmition
    global M,D,K,tolerance,max_iterations
    global given_control_input,theta_yht_p
    for i in range(max_iterations):
        control_input = given_control_input
        # 更新关节状态
        initial_theta = (np.linalg.inv(K).dot(control_input-D.dot(initial_theta_dot)-M.dot(initial_theta_double_dot)))+theta_yht_p
        # 输出最终的关节角度
        ftheta_addmition[0][2] = initial_theta[0]
        ftheta_addmition[1][2] = initial_theta[1]
        # print("右后腿最终关节角度(小腿、大腿):", initial_theta)
    return ftheta_addmition[0][2],ftheta_addmition[1][2]
    
def addmition_yqt(initial_theta,initial_theta_dot,initial_theta_double_dot):
        # 迭代计算
    global dt,ftheta_addmition
    global M,D,K,tolerance,max_iterations
    global given_control_input,theta_yqt_p
    for i in range(max_iterations):
            # 计算控制误差
        
            # 计算控制输入
            # control_input = M.dot(initial_theta_double_dot) + D.dot(initial_theta_dot) + K.dot(error)
        control_input = given_control_input
            # 更新关节状态
        initial_theta = (np.linalg.inv(K).dot(control_input-D.dot(initial_theta_dot)-M.dot(initial_theta_double_dot)))+theta_yqt_p
       
        # 输出最终的关节角度
        ftheta_addmition[0][3] = initial_theta[0]
        ftheta_addmition[1][3] = initial_theta[1]
    return ftheta_addmition[0][3],ftheta_addmition[1][3]
    # print("右前腿最终关节角度(小腿、大腿):", initial_theta)
# 用来计算到底输出什么样的角度。是位置控制还是导纳控制
def calculate_angle_zq(current_angle,pos_angle,current_torque,vmc_torque):
    global max_torque
    # 扭矩判断，只判断y轴方向的扭矩。
    error_angle = abs(current_angle) - abs(pos_angle)
    error_torque  =abs(current_torque) - abs(vmc_torque)
    # 进行位置判断，推测角度有没有到达实际位置。
    theta = pos_angle;
    if (abs(error_angle) < [0.1,0.1]).all() and ((error_torque) < [0,0]).all():
        print("左前位置控制")
        theta = pos_angle;
    elif (abs(error_angle) < [0.1,0.1]).all() and (abs(error_torque) > [0,0]).all():
        print("左前导纳控制0")
        theta = addmition_zq(vmc_torque);
    elif (abs(error_angle) > [0.1,0.1]).all() and (abs(error_torque) > [0,0]).all():
        print("左前导纳控制1")
        theta = addmition_zq(vmc_torque);
    elif (abs(error_angle) < [0.1,0.1]).all() and (abs(current_torque > max_torque)).all():
        print("左前导纳控制2")
        theta = addmition_zq(vmc_torque);
    else:
        print("error")
    global ftheta_addmition
    ftheta_addmition[0][0]=theta[0]
    ftheta_addmition[1][0]=theta[1]
def calculate_angle_zh(current_angle,pos_angle,current_torque,vmc_torque):
    global max_torque
    # 扭矩判断，只判断y轴方向的扭矩。
    error_angle = abs(current_angle) - abs(pos_angle)
    error_torque  =abs(current_torque) - abs(vmc_torque)
    theta = pos_angle;
    if (abs(error_angle) < [0.1,0.1]).all() and ((error_torque) < [0,0]).all():
        print("左后位置控制")
        theta = pos_angle;
    elif (abs(error_angle) < [0.1,0.1]).all() and (abs(error_torque) > [0,0]).all():
        print("左后导纳控制0")
        theta = addmition_zh(vmc_torque);
    elif (abs(error_angle) > [0.1,0.1]).all() and (abs(error_torque) > [0,0]).all():
        print("左后导纳控制1")
        theta = addmition_zh(vmc_torque);
    elif (abs(error_angle) < [0.1,0.1]).all() and (abs(current_torque > max_torque)).all():
        print("左后导纳控制2")
        theta = addmition_zh(vmc_torque);
    else:
        print("error")
    ftheta_addmition[0][1]=theta[0]
    ftheta_addmition[1][1]=theta[1]
def calculate_angle_yh(current_angle,pos_angle,current_torque,vmc_torque):
    global max_torque
    # 扭矩判断，只判断y轴方向的扭矩。
    error_angle = abs(current_angle) - abs(pos_angle)
    error_torque  =abs(current_torque) - abs(vmc_torque)
    # 进行位置判断，推测角度有没有到达实际位置。
    theta = pos_angle;
    if (abs(error_angle) < [0.05,0.05]).all() and ((error_torque) < [0,0]).all():
        print("右后位置控制")
        theta = pos_angle;
    elif (abs(error_angle) < [0.05,0.05]).all() and (abs(error_torque) > [0,0]).all():
        print("右后导纳控制0")
        theta = addmition_yh(vmc_torque);
    elif (abs(error_angle) > [0.05,0.05]).all() and (abs(error_torque) > [0,0]).all():
        print("右后导纳控制1")
        theta = addmition_yh(vmc_torque);
    elif (abs(error_angle) < [0.05,0.05]).all() and ((abs(current_torque) - max_torque)>[0,0]).all():
        print("右后导纳控制2")
        theta = addmition_yh(vmc_torque);
    else:
        print("error")
    ftheta_addmition[0][2]=theta[0]
    ftheta_addmition[1][2]=theta[1]
def calculate_angle_yq(current_angle,pos_angle,current_torque,vmc_torque):
    global max_torque
    # 扭矩判断，只判断y轴方向的扭矩。
    error_angle = abs(current_angle) - abs(pos_angle)
    error_torque  =abs(current_torque) - abs(vmc_torque)
    # 进行位置判断，推测角度有没有到达实际位置。
    theta = pos_angle;
    if (abs(error_angle) < [0.05,0.05]).all() and ((error_torque) < [0,0]).all():
        print("右前位置控制")
        theta = pos_angle;
    elif (abs(error_angle) < [0.01,0.01]).all() and (abs(error_torque) > [0,0]).all():
        print("右前导纳控制0")
        theta = addmition_yq(vmc_torque);
    elif (abs(error_angle) > [0.01,0.01]).all() and (abs(error_torque) > [0,0]).all():
        print("右前导纳控制1")
        theta = addmition_yq(vmc_torque);
    elif (abs(error_angle) < [0.01,0.01]).all() and (abs(current_torque > max_torque)).all():
        print("右前导纳控制2")
        theta = addmition_yq(vmc_torque);
    else:
        print("error")
    ftheta_addmition[0][3]=theta[0]
    ftheta_addmition[1][3]=theta[1]
def jointstate_callback(data):
    # 用来中间变量存储数据
    prev_zqt_theta = [0.0, 0.0]
    prev_zht_theta = [0.0, 0.0]
    prev_yht_theta = [0.0, 0.0]
    prev_yqt_theta = [0.0, 0.0]
    prev_zqt_omega = [0.0, 0.0]
    prev_zht_omega = [0.0, 0.0]
    prev_yht_omega = [0.0, 0.0]
    prev_yqt_omega = [0.0, 0.0]
    global zqt_theta
    zqt_theta = np.array([0.0,0.0])
    global zht_theta
    zht_theta = np.array([0.0,0.0])
    global yht_theta
    yht_theta = np.array([0.0,0.0])
    global yqt_theta
    yqt_theta = np.array([0.0,0.0])
    # 获取所有关节的名称
    joint_names = data.name
    # 获取所有关节的位置信息
    joint_positions = data.position
    # 输出每个关节的名称和位置信息
    # for name, position in zip(joint_names, joint_positions):
    #     # rospy.loginfo("Joint: %s, Position: %.4f", name, position)
    #     print("")
    if "zqt_joint2" in joint_names:
        joint1_index = joint_names.index("zqt_joint2")
        zqt_joint2_position = joint_positions[joint1_index]
        zqt_theta[0] = -(zqt_joint2_position)
    if "zqt_joint3" in joint_names:
        joint1_index = joint_names.index("zqt_joint3")
        zqt_joint3_position = joint_positions[joint1_index]
        zqt_theta[1] = -float((zqt_joint3_position))-pi/2;
    if "zht_joint2" in joint_names:
        joint1_index = joint_names.index("zht_joint2")
        zht_joint2_position = joint_positions[joint1_index]
        zht_theta[0] = -zht_joint2_position;
    if "zht_joint3" in joint_names:
        joint1_index = joint_names.index("zht_joint3")
        zht_joint3_position = joint_positions[joint1_index]
        zht_theta[1] = -float((zht_joint3_position))-pi/2;
    if "yht_joint2" in joint_names:
        joint1_index = joint_names.index("yht_joint2")
        yht_joint2_position = joint_positions[joint1_index]
        yht_theta[0] = -yht_joint2_position;
    if "yht_joint3" in joint_names:
        joint1_index = joint_names.index("yht_joint3")
        yht_joint3_position = joint_positions[joint1_index]
        yht_theta[1] = -float((yht_joint3_position))-pi/2;
    if "yqt_joint2" in joint_names:
        joint1_index = joint_names.index("yqt_joint2")
        yqt_joint2_position = joint_positions[joint1_index]
        yqt_theta[0] = -yqt_joint2_position;
    if "yqt_joint3" in joint_names:
        joint1_index = joint_names.index("yqt_joint3")
        yqt_joint3_position = joint_positions[joint1_index]
        yqt_theta[1] = -float((yqt_joint3_position))-pi/2;

    # 计算角速度
    global zqt_omega, zht_omega, yht_omega, yqt_omega
    zqt_omega = calculate_omega(prev_zqt_theta, zqt_theta, 1)
    zht_omega = calculate_omega(prev_zht_theta, zht_theta, 1)
    yht_omega = calculate_omega(prev_yht_theta, yht_theta, 1)
    yqt_omega = calculate_omega(prev_yqt_theta, yqt_theta, 1)
    # print("角速度为：")
    # print("----------------------------------------")
    # print(zqt_omega,zht_omega,yht_omega,yqt_omega)
    # 计算角加速度
    global zqt_alpha, zht_alpha, yht_alpha, yqt_alpha
    zqt_alpha = calculate_alpha(prev_zqt_omega, zqt_omega, 1)
    zht_alpha = calculate_alpha(prev_zht_omega, zht_omega, 1)
    yht_alpha = calculate_alpha(prev_yht_omega, yht_omega, 1)
    yqt_alpha = calculate_alpha(prev_yqt_omega, yqt_omega, 1)
    # print("角加速度为：")
    # print("----------------------------------------")
    # print(zqt_alpha,zht_alpha,yht_alpha,yqt_alpha)
    # 更新前一时刻的角度值和角速度值
    prev_zqt_theta = zqt_theta
    prev_zht_theta = zht_theta
    prev_yht_theta = yht_theta
    prev_yqt_theta = yqt_theta
    prev_zqt_omega = zqt_omega
    prev_zht_omega = zht_omega
    prev_yht_omega = yht_omega
    prev_yqt_omega = yqt_omega
    # return zqt_theta,zht_theta,yht_theta,yqt_theta
    # 这里存储的角度是小腿角度在前，大腿角度在后
    # 这个ftheta是期望角度 

    # print("theta2 theta1的两组解为左前、左后、右后、右前：")
    # print(ftheta[1,0],ftheta[1,1])
    # print(ftheta[0,2],ftheta[0,3])
    # print(ftheta[0,4],ftheta[0,5])
    # print(ftheta[1,6],ftheta[1,7])
    # print(ftheta)
    # theta_d 应该是给定的ftheta
    global theta_zqt_p,theta_zht_p,theta_yht_p,theta_yqt_p
    # 先是小腿，后是大腿 # 目标关节角度
    theta_zqt_p =np.array ([float(ftheta[1,1]*pi/180),float(ftheta[1,0]*pi/180)])  
    theta_zht_p =np.array ([float(ftheta[0,3]*pi/180),float(ftheta[0,2]*pi/180)])
    theta_yht_p =np.array ([float(ftheta[0,5]*pi/180),float(ftheta[0,4]*pi/180)])
    theta_yqt_p =np.array ([float(ftheta[1,7]*pi/180),float(ftheta[1,6]*pi/180)])
    # vmc_torque_callback(data)
    print("--------------------")
    global smoothed_torque,ftheta_addmition
    # print(zqt_vmc_torque)
    # 这里同样，先小腿后大腿
    calculate_angle_zq(zqt_theta,theta_zqt_p,np.array([smoothed_torque[0],smoothed_torque[1]]),np.array([zqt_vmc_torque[1],zqt_vmc_torque[0]]))  
    calculate_angle_zh(zht_theta,theta_zht_p,np.array([smoothed_torque[2],smoothed_torque[3]]),np.array([zht_vmc_torque[1],zht_vmc_torque[0]]))    
    calculate_angle_yh(yht_theta,theta_yht_p,np.array([smoothed_torque[4],smoothed_torque[5]]),np.array([yht_vmc_torque[1],yht_vmc_torque[0]]))    
    calculate_angle_yq(yqt_theta,theta_yqt_p,np.array([smoothed_torque[6],smoothed_torque[7]]),np.array([yqt_vmc_torque[1],yqt_vmc_torque[0]]))      
    print("-----------------------------------")
    print("实际角度")
    print(theta_zqt_p*180/pi)
    print(theta_zht_p*180/pi)
    print(theta_yht_p*180/pi)
    print(theta_yqt_p*180/pi)
    print("-----------------------------------")
    print("输出角度")
    for row in zip(*ftheta_addmition):
        for value in row:
            print(value*180/pi, end="\t")
        print()
    vmc_angle_p = Float64MultiArray()
    vmc_angle = Float64MultiArray()
    vmc_angle_p.data= [theta_zqt_p[0]*180/pi,theta_zqt_p[1]*180/pi,
                       theta_zht_p[0]*180/pi,theta_zht_p[1]*180/pi,
                       theta_yht_p[0]*180/pi,theta_yht_p[1]*180/pi,
                       theta_yqt_p[0]*180/pi,theta_yqt_p[1]*180/pi]
    vmc_angle.data = [ftheta_addmition[0][0]*180/pi,ftheta_addmition[1][0]*180/pi,
                      ftheta_addmition[0][1]*180/pi,ftheta_addmition[1][1]*180/pi,
                      ftheta_addmition[0][2]*180/pi,ftheta_addmition[1][2]*180/pi,
                      ftheta_addmition[0][3]*180/pi,ftheta_addmition[1][3]*180/pi
                      ]
    vmc_angle_p1.publish(vmc_angle_p)
    vmc_angle1.publish(vmc_angle)
    rospy.sleep(0.1)
   
def vmc_torque_callback(msg):
    global zqt_vmc_torque,zht_vmc_torque,yht_vmc_torque,yqt_vmc_torque
    zqt_vmc_torque = msg.data[0:2]
    zht_vmc_torque = msg.data[2:4]
    yht_vmc_torque = msg.data[4:6]
    yqt_vmc_torque = msg.data[6:8]
    # vmc_torque = msg.data
    # zqt_vmc_torque = vmc_torque[0]
    # print(zqt_vmc_torque,zht_vmc_torque,yht_vmc_torque,yqt_vmc_torque)
def angle_insolve_callback(msg):
 
    coor = msg.data
    global ftheta
    ftheta=angle_insolve(coor[0],coor[1],coor[2],coor[3],coor[4],coor[5],coor[6],coor[7])
     
def addmition_control():
    rospy.init_node('addmition_control_node',anonymous=True)
    rospy.Subscriber('/xingtian/vmc_torque',Float64MultiArray,vmc_torque_callback)
    global topics
    topics = ["/torque_zqt2", "/torque_zqt3", "/torque_zqt4","/torque_zht4", "/torque_zht2", "/torque_zht3", "/torque_yht2", "/torque_yht3","/torque_yht4", "/torque_yqt2", "/torque_yqt3", "/torque_yqt4"]
    for topic in topics:
        rospy.Subscriber(topic,WrenchStamped,torque_callback,callback_args=topic) 
    angle_feedback = rospy.Subscriber("/xingtian/joint_states",JointState,jointstate_callback)
    joint_control = rospy.Subscriber("/xingtian/joint_control",Float64MultiArray,angle_insolve_callback) 
    
    global vmc_angle_p1,vmc_angle1
    vmc_angle1 = rospy.Publisher("/xingtian/vmc_angle",Float64MultiArray,queue_size=10)
    vmc_angle_p1 = rospy.Publisher("/xingtian/vmc_angle_p",Float64MultiArray,queue_size=10)
    # plot_curves(angle_feedback)
    
    rospy.spin()  
if __name__ =="__main__":
   
    addmition_control()
#! /user/bin/env/ python3
#coding=utf-8
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import tf.transformations as tf
import math
import numpy as np
import threading
from numpy.linalg import inv

gx_org,gy_org,gz_org,ax_org,ay_org,az_org = 0,0,0,0,0,0
quaternion = (0,0,0,1)
def imu_callback(data):
    # 获取四元数消息中的姿态信息
    global gx_org,gy_org,gz_org,ax_org,ay_org,az_org,quaternion
    orientation_q = data.orientation
    gx_org = data.angular_velocity.x
    gy_org = data.angular_velocity.y
    gz_org = data.angular_velocity.z
    ax_org = data.linear_acceleration.x
    ay_org = data.linear_acceleration.y
    az_org = data.linear_acceleration.z
    print("0")
    print(ax_org)
    # 将四元数消息转换为四元数
    quaternion = (
        orientation_q.x,
        orientation_q.y,
        orientation_q.z,
        orientation_q.w)
    # 将四元数转换为欧拉角
    # euler = tf.euler_from_quaternion(quaternion)
    # roll_cal = math.atan2(2*(orientation_q.y*orientation_q.z+orientation_q.w*orientation_q.x),(orientation_q.w*orientation_q.w-orientation_q.x*orientation_q.x-orientation_q.y*orientation_q.y+orientation_q.z*orientation_q.z))
    # pitch_cal = math.asin(-2*(orientation_q.x*orientation_q.z-orientation_q.w*orientation_q.y))
    # yaw_cal = -math.atan2(2*( orientation_q.x*orientation_q.y+orientation_q.w*orientation_q.z),(orientation_q.w*orientation_q.w+orientation_q.x*orientation_q.x-orientation_q.y*orientation_q.y-orientation_q.z*orientation_q.z))
    # 输出欧拉角
    # roll = euler[0]
    # pitch = euler[1]
    # yaw = euler[2]
   
    # x, y, z, w = quaternion
    # rotation_matrix = np.array([
    #     [1 - 2*y*y - 2*z*z,   2*x*y - 2*z*w,       2*x*z + 2*y*w],
    #     [2*x*y + 2*z*w,       1 - 2*x*x - 2*z*z,   2*y*z - 2*x*w],
    #     [2*x*z - 2*y*w,       2*y*z + 2*x*w,       1 - 2*x*x - 2*y*y]
    # ])
    # # 根据欧拉角计算旋转矢量
    # global rotationxyz
    # rotationxyz = euler_to_rotMatxyz(yaw,pitch,roll)
    # global rotationzyx
    # rotationzyx = euler_to_rotMatzyx(yaw,pitch,roll)
    # print("Received Euler Angles (roll, pitch, yaw):", roll, pitch, yaw)
    # print(roll_cal,pitch_cal,yaw_cal)
    # print("计算旋转矩阵")
    # print(rotationxyz)
    # print(rotationzyx)
    # time.sleep(2)
    # 这里下边开始的是卡尔曼滤波。
    # 采样周期
    
    
    # IMU任务
    # imu_task()
def imu_task():
    global pitch, roll, yaw, q, q_p, q_e, P, P_p, dt, Q, R,Z_e,Z_p,H,H_t,P_temp1,P_temp2
    global K,K_temp1,K_temp2,K_temp3
    global gx_org,gy_org,gz_org,ax_org,ay_org,az_org,quaternion
    # global pitch, roll, yaw, q, q_p, q_e, P, P_p, dt, Q, R,Z_e,Z_p,H,H_t,P_temp1,P_temp2
    # global K,K_temp1,K_temp2,K_temp3
    dt = 0.0025  # 2.5ms
    # 欧拉角
    pitch = 0
    roll = 0
    yaw = 0

    # 过程协方差矩阵
    Qs = 0.1
    Q = np.diag([Qs, Qs, Qs, Qs])

    # 观测协方差矩阵
    Rs_a = 0.000001
    R = np.diag([Rs_a, Rs_a, Rs_a,Rs_a])

    # 单位矩阵
    I = np.eye(4)

    # 初始化四元数
    q = np.array([1, 0, 0, 0])
    q_p = np.array([1, 0, 0, 0])
    q_e = np.array([0, 0, 0, 0])

    # 初始化状态方程雅克比矩阵及其转置
    F = np.zeros((4, 4))
    F_t = np.zeros((4, 4))

    # 初始化观测值相关向量
    Z = np.zeros(4)
    Z_p = np.zeros(4)
    Z_e = np.zeros(4)

    # 初始化观测方程雅克比矩阵及其转置
    H = np.zeros((4, 4))
    H_t = np.zeros((4, 4))

    # 初始化状态协方差相关矩阵
    P = np.eye(4)
    P_p = np.eye(4)
    P_temp1 = np.zeros((4, 4))
    P_temp2 = np.zeros((4, 4))

    # 初始化扩展卡尔曼增益相关矩阵
    K = np.zeros((4, 4))
    K_temp1 = np.zeros((4, 4))
    K_temp2 = np.zeros((4, 4))
    K_temp3 = np.zeros((4, 4))
    print("1")
        
    # imu初始化
    gx_offset = gy_offset = gz_offset = 0
    for i in range(4000):
        gx_offset += gx_org
        gy_offset += gy_org
        gz_offset += gz_org
        # 假设延时函数在此处

    gx_offset /= 4000.0
    gy_offset /= 4000.0
    gz_offset /= 4000.0

    ax = ax_org
    # print("?????")
    # print(ax)
    ay = ay_org
    az = az_org
    # print(ax)
    # print("99999999999999999999999999999999999")
    norm = np.sqrt(ax**2 + ay**2 + az**2)
    if norm ==0:
        Z[0]=Z[1]=Z[2]=0;
    else:
        Z[0] = ax / norm
        Z[1] = ay / norm
        Z[2] = az / norm

    pitch = np.arcsin(Z[1])
    roll = np.arctan2(-Z[0], -Z[2] if Z[2] > 0 else -Z[2] + np.pi)
    yaw = 0

    q[0] = np.cos(pitch / 2) * np.cos(roll / 2) * np.cos(yaw / 2) - np.sin(pitch / 2) * np.sin(roll / 2) * np.sin(
            yaw / 2)
    q[1] = np.cos(roll / 2) * np.cos(yaw / 2) * np.sin(pitch / 2) + np.cos(pitch / 2) * np.sin(roll / 2) * np.sin(
            yaw / 2)
    q[2] = np.cos(pitch / 2) * np.cos(yaw / 2) * np.sin(roll / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)
    q[3] = np.cos(pitch / 2) * np.cos(roll / 2) * np.sin(yaw / 2) + np.cos(yaw / 2) * np.sin(pitch / 2) * np.sin(
            roll / 2)
    # print(q)
    # q = (0,0,0,1)
    # imu更新
    while True:
        gx = gx_org - gx_offset
        gy = gy_org - gy_offset
        gz = gz_org - gz_offset
        ax = ax_org
        ay = ay_org
        az = az_org
        # print(ax)
        # print(gy)
        # print(az)
            # 四元数更新雅克比矩阵及其转置矩阵计算
        F[0, :] = [1, -gx * dt * 0.5, -gy * dt * 0.5, -gz * dt * 0.5]
        F[1, :] = [gx * dt * 0.5, 1, gz * dt * 0.5, -gy * dt * 0.5]
        F[2, :] = [gy * dt * 0.5, -gz * dt * 0.5, 1, gx * dt * 0.5]
        F[3, :] = [gz * dt * 0.5, gy * dt * 0.5, -gx * dt * 0.5, 1]
        F_t = F.T
        # print(F)
        # 带误差的四元数更新
        q_p = np.dot(F, q) * dt
        # print(q)
        # print("111111111111111111")
        
            # 带误差的四元数单位化
        q_p /= np.linalg.norm(q_p)
        print(q_p)
            # 四元数转观测值
        Z_p = quaternion
        # print(Z_p)
            # 预测量雅克比矩阵及其转置矩阵计算
        H = np.eye(4)
        H_t = H.T

            # 协方差矩阵预测
        P_p = np.dot(np.dot(F, P), F.T) + Q

            # 计算卡尔曼增益矩阵
        K = np.dot(np.dot(P_p, H_t), inv(np.dot(np.dot(H, P_p), H_t) + R))
        
            # 更新协方差矩阵
        P = np.dot((I - np.dot(K, H)), P_p)

            # 四元数最优估计
        Z_e = Z - Z_p
        q_e = np.dot(K, Z_e)
        # print(q_e)
        q = q_p + q_e

            # 单位化四元数
        q /= np.linalg.norm(q)
        # 这里求解出的四元数不知道为什么和imu提供的四元数不一样。
        print(-q)
        print("----------------")
        print(quaternion)
            # 四元数转欧拉角
        euler = tf.euler_from_quaternion(quaternion)
        pitch = 57.3 * np.arcsin(2 * (q[2] * q[3] + q[0] * q[1]))
        roll = 57.3 * np.arctan2(2 * (q[0] * q[2] - q[1] * q[3]),
                                    q[0] ** 2 - q[1] ** 2 - q[2] ** 2 + q[3] ** 2)
        yaw += 57.3 * dt * (gz_org - gz_offset)
        print("3")
            # print(roll,pitch,yaw)
        print("----------------------------")
        # print(euler[0],euler[1],euler[2])





# def euler_to_rotVec(yaw, pitch, roll):
#     Rmat = euler_to_rotMatxyz(yaw, pitch, roll)
#     theta = math.acos(((Rmat[0, 0] + Rmat[1, 1] + Rmat[2, 2]) - 1) / 2)
#     sin_theta = math.sin(theta)
#     if sin_theta == 0:
#         rx, ry, rz = 0.0, 0.0, 0.0
#     else:
#         multi = 1 / (2 * math.sin(theta))
#         rx = multi * (Rmat[2, 1] - Rmat[1, 2]) * theta
#         ry = multi * (Rmat[0, 2] - Rmat[2, 0]) * theta
#         rz = multi * (Rmat[1, 0] - Rmat[0, 1]) * theta
#     return rx, ry, rz
#      # 根据欧拉角计算旋转矩阵
# def euler_to_rotMatxyz(yaw, pitch, roll):
#     Rz_yaw = np.array([
#     [np.cos(yaw), -np.sin(yaw), 0],
#     [np.sin(yaw),  np.cos(yaw), 0],
#     [          0,            0, 1]])
#     Ry_pitch = np.array([
#     [ np.cos(pitch), 0, np.sin(pitch)],
#     [             0, 1,             0],
#     [-np.sin(pitch), 0, np.cos(pitch)]])
#     Rx_roll = np.array([
#     [1,            0,             0],
#     [0, np.cos(roll), -np.sin(roll)],
#     [0, np.sin(roll),  np.cos(roll)]])
#     rotMatxyz = np.dot(Rz_yaw, np.dot(Ry_pitch, Rx_roll))
#     return rotMatxyz
   
# def euler_to_rotMatzyx(yaw, pitch, roll):
#     Rz_yaw = np.array([
#     [np.cos(yaw), -np.sin(yaw), 0],
#     [np.sin(yaw),  np.cos(yaw), 0],
#     [          0,            0, 1]])
#     Ry_pitch = np.array([
#     [ np.cos(pitch), 0, np.sin(pitch)],
#     [             0, 1,             0],
#     [-np.sin(pitch), 0, np.cos(pitch)]])
#     Rx_roll = np.array([
#     [1,            0,             0],
#     [0, np.cos(roll), -np.sin(roll)],
#     [0, np.sin(roll),  np.cos(roll)]])
#     rotMatzyx = np.dot(Rx_roll, np.dot(Ry_pitch, Rz_yaw))
#     return rotMatzyx

def imu_listener():
    rospy.init_node('imu_listener', anonymous=True)
    # rate = rospy.Rate(10)
    rospy.Subscriber("/imu", Imu, imu_callback)
    thread = threading.Thread(target=imu_task)
    # thread_1 = threading.Thread(target=imu_callback)
    thread.start()
    # thread_1.start()
    # rate.sleep()
    rospy.spin()

if __name__ == '__main__':

    imu_listener()

 
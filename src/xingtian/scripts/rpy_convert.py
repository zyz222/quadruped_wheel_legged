#! /user/bin/env/ python3
#coding=utf-8
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import tf.transformations as tf
import math
import numpy as np
import time
def imu_callback(data):
    # 获取四元数消息中的姿态信息
    orientation_q = data.orientation

    # 将四元数消息转换为四元数
    quaternion = (
        orientation_q.x,
        orientation_q.y,
        orientation_q.z,
        orientation_q.w)
    # q0 = orientation_q.w
    # q1 = orientation_q.x
    # q2 = orientation_q.y
    # q3 = orientation_q.z
    # 将四元数转换为欧拉角
    euler = tf.euler_from_quaternion(quaternion)
    roll_cal = math.atan2(2*(orientation_q.y*orientation_q.z+orientation_q.w*orientation_q.x),(orientation_q.w*orientation_q.w-orientation_q.x*orientation_q.x-orientation_q.y*orientation_q.y+orientation_q.z*orientation_q.z))
    pitch_cal = math.asin(-2*(orientation_q.x*orientation_q.z-orientation_q.w*orientation_q.y))
    yaw_cal = -math.atan2(2*( orientation_q.x*orientation_q.y+orientation_q.w*orientation_q.z),(orientation_q.w*orientation_q.w+orientation_q.x*orientation_q.x-orientation_q.y*orientation_q.y-orientation_q.z*orientation_q.z))
    # 输出欧拉角
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    x, y, z, w = quaternion
    rotation_matrix = np.array([
        [1 - 2*y*y - 2*z*z,   2*x*y - 2*z*w,       2*x*z + 2*y*w],
        [2*x*y + 2*z*w,       1 - 2*x*x - 2*z*z,   2*y*z - 2*x*w],
        [2*x*z - 2*y*w,       2*y*z + 2*x*w,       1 - 2*x*x - 2*y*y]
    ])
    # 根据欧拉角计算旋转矢量
    global rotationxyz
    rotationxyz = euler_to_rotMatxyz(yaw,pitch,roll)
    global rotationzyx
    rotationzyx = euler_to_rotMatzyx(yaw,pitch,roll)
    print("Received Euler Angles (roll, pitch, yaw):", roll, pitch, yaw)
    print(roll_cal,pitch_cal,yaw_cal)
    print("计算旋转矩阵")
    # print(rotationxyz)
    print(rotationzyx)
    time.sleep(2)
def euler_to_rotVec(yaw, pitch, roll):
    Rmat = euler_to_rotMatxyz(yaw, pitch, roll)
    theta = math.acos(((Rmat[0, 0] + Rmat[1, 1] + Rmat[2, 2]) - 1) / 2)
    sin_theta = math.sin(theta)
    if sin_theta == 0:
        rx, ry, rz = 0.0, 0.0, 0.0
    else:
        multi = 1 / (2 * math.sin(theta))
        rx = multi * (Rmat[2, 1] - Rmat[1, 2]) * theta
        ry = multi * (Rmat[0, 2] - Rmat[2, 0]) * theta
        rz = multi * (Rmat[1, 0] - Rmat[0, 1]) * theta
    return rx, ry, rz
     # 根据欧拉角计算旋转矩阵
def euler_to_rotMatxyz(yaw, pitch, roll):
    Rz_yaw = np.array([
    [np.cos(yaw), -np.sin(yaw), 0],
    [np.sin(yaw),  np.cos(yaw), 0],
    [          0,            0, 1]])
    Ry_pitch = np.array([
    [ np.cos(pitch), 0, np.sin(pitch)],
    [             0, 1,             0],
    [-np.sin(pitch), 0, np.cos(pitch)]])
    Rx_roll = np.array([
    [1,            0,             0],
    [0, np.cos(roll), -np.sin(roll)],
    [0, np.sin(roll),  np.cos(roll)]])
    rotMatxyz = np.dot(Rz_yaw, np.dot(Ry_pitch, Rx_roll))
    return rotMatxyz
   
def euler_to_rotMatzyx(yaw, pitch, roll):
    Rz_yaw = np.array([
    [np.cos(yaw), -np.sin(yaw), 0],
    [np.sin(yaw),  np.cos(yaw), 0],
    [          0,            0, 1]])
    Ry_pitch = np.array([
    [ np.cos(pitch), 0, np.sin(pitch)],
    [             0, 1,             0],
    [-np.sin(pitch), 0, np.cos(pitch)]])
    Rx_roll = np.array([
    [1,            0,             0],
    [0, np.cos(roll), -np.sin(roll)],
    [0, np.sin(roll),  np.cos(roll)]])
    rotMatzyx = np.dot(Rx_roll, np.dot(Ry_pitch, Rz_yaw))
    return rotMatzyx

def imu_listener():
    rospy.init_node('imu_listener', anonymous=True)
    rate = rospy.Rate(10)
    rospy.Subscriber("/imu", Imu, imu_callback)
    rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    
    # try:
    imu_listener()

    # except rospy.ROSInterruptException:
    #     pass
    # rospy.spin()
#!/usr/bin/env python3
# -*- coding: utf-8 -*
from math import sin,cos,pi
from std_msgs.msg import Float64MultiArray,String
import rospy
import pydog_trot
import pydog_walk
from sensor_msgs.msg import Imu
import tf.transformations as tf
# walk步态需要获取imu信息

# 全局变量
spd = 50
L = 1
R = 1
h = 50
speed = 0.02
Ts_t = 1
Ts_w = 5     #为walk步态调整的
t = 0

def imu_callback(data):
      # 获取四元数消息中的姿态信息
    global roll,pitch,yaw
    orientation_q = data.orientation

    # 将四元数消息转换为四元数
    quaternion = (
        orientation_q.x,
        orientation_q.y,
        orientation_q.z,
        orientation_q.w)
    euler = tf.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    print(euler)
    
# trot步态
def gait_trot(h):
    global t, P_
    if t >= (Ts_t - speed):  # 一个完整的运动周期结束 trot
        t = 0
    elif L == 0 and R == 0:
        t = 0
    else:
        t = t + speed
    P_ = pydog_trot.cal_t(t, 0, spd, h, L, L, R, R)
    print(P_)
    return P_

# walk 步态
def gait_walk(xf, walk_h, pitch, roll):
    global t, P_
    speed_w = 0.005                 #walk步态频率
    if t >= (Ts_w  + 2 - speed_w):  # 一个完整的运动周期结束 walk
        t = 0
    elif L == 0 and R == 0:
        t = 0
    else:
        t = t + speed_w                            #pitch roll xf        h ,     t ,r1,r2,r3,r4
    P_ = pydog_walk.cal_w(pitch, roll, xf, walk_h, t, L, L, R, R)
    print(P_)
    return P_

# 移动控制
def move(spd_, L_, R_):  
    global spd, L, R
    spd = spd_
    L = L_
    R = R_

# 回调函数，用于接收姿态控制消息
def gait_callback(msg):
    global h, gait_switch
    ch = msg.data
    h = 30  # 此处可能需要根据消息内容进行设定
    # 姿态控制切换
    if ch == '0':
        gait_switch = '0'
    elif ch == '1':
        gait_switch = '1'
    elif ch == '2':
        gait_switch = '2'
    elif ch == "h":
        h += 10.0
    print("控制方式:", ch)

# 主函数
def main():
    global gait_switch
    gait_switch = '2'
    rospy.init_node("xingtian_gait", anonymous=True)
    gait = rospy.Subscriber("/xingtian/posture_control", String, gait_callback)
    RPY = rospy.Subscriber("/imu",Imu,imu_callback)
    gait_coor = rospy.Publisher("/xingtian/joint_control", Float64MultiArray, queue_size=10)
    move(50,1,1)
    rospy.sleep(1)  # 等待节点初始化完成
    rate = rospy.Rate(100)  # 设置发布频率

    while not rospy.is_shutdown():
        if gait_switch == "1":
            coor = gait_trot(120)
            print("trot步态")
        elif gait_switch == "2":
            coor = gait_walk(120, 120, -pitch, -roll)  # 步态距离、步态高度、欧拉角 pitch、roll
            print("walk步态")

        coor_array = Float64MultiArray()
        coor_array.data = list(coor)
        gait_coor.publish(coor_array)
        rate.sleep()

if __name__ == "__main__":
    main()


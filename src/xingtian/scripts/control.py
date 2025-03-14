#! /user/bin/env/ python3
#coding=utf-8
import math
import rospy
from angle_insolve import angle_insolve
from std_msgs.msg import Float64
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

def velocity_callback(msg):
    global zqtvelocity,zhtvelocity,yhtvelocity,yqtvelocity
    global velocity_wheel,last_msg_time
    ch = msg.data
    last_msg_time = rospy.Time.now()  # 更新最后一次消息的时间戳
    if ch == "w":
        zqtvelocity = 20
        zhtvelocity = 20
        yhtvelocity = 20
        yqtvelocity = 20
        # print(zqtvelocity)
    if ch == 'a':
        zqtvelocity = -20
        zhtvelocity = -20
        yhtvelocity = 20
        yqtvelocity =  20
        # print(zqtvelocity)
    if ch == 'd':
        zqtvelocity = 20
        zhtvelocity = 20
        yhtvelocity = -20
        yqtvelocity =  -20
    if ch == 's':
        zqtvelocity = -20
        zhtvelocity = -20
        yhtvelocity = -20
        yqtvelocity = -20
    if ch == 'p':
        zqtvelocity = 0
        zhtvelocity = 0
        yhtvelocity = 0
        yqtvelocity = 0
    velocity_wheel[0] = zqtvelocity;
    velocity_wheel[1] = zhtvelocity;
    velocity_wheel[2] = yhtvelocity;
    velocity_wheel[3] = yqtvelocity;
    
def velocity_control_stop():
    global zqtvelocity, zhtvelocity, yhtvelocity, yqtvelocity
    global velocity_wheel,last_msg_time
    # 获取当前时间
    current_time = rospy.Time.now()
    # 计算与上次消息时间的时间差
    time_diff = current_time - last_msg_time
    # print(time_diff)
    if time_diff.to_sec() > 0.2:  # 如果超过0.2秒没有接收到消息
        # 将速度值设为0
        zqtvelocity = 0
        zhtvelocity = 0
        yhtvelocity = 0
        yqtvelocity = 0
        velocity_wheel[0] = zqtvelocity
        velocity_wheel[1] = zhtvelocity
        velocity_wheel[2] = yhtvelocity
        velocity_wheel[3] = yqtvelocity
def angle_insolve_callback(msg):
    global zqt_ctheta1,zqt_ctheta2,zht_ctheta1,zht_ctheta2
    global yht_ctheta1,yht_ctheta2,yqt_ctheta1,yqt_ctheta2
    coor = msg.data
    ftheta=angle_insolve(coor[0],coor[1],coor[2],coor[3],coor[4],coor[5],coor[6],coor[7])
        #落足点角度
        # theta2是大腿角度      theta1 是小腿角度    
        # 可正可负                  只能是负的      取第二组解
    zqt_ctheta2 = ftheta[0,0];     zqt_ctheta1 = ftheta[0,1];
        # 可正可负                  只能是正的      取第一组解
    zht_ctheta2 = ftheta[0,2];    zht_ctheta1 = ftheta[0,3];  
        # 可正可负                  只能是正的      取第一组解
    yht_ctheta2 = ftheta[0,4];       yht_ctheta1 = ftheta[0,5];    
        # 可正可负                  只能是负的     取第二组解
    yqt_ctheta2 = ftheta[0,6];       yqt_ctheta1 = ftheta[0,7]; 
    # print(0)


if __name__ == "__main__":
    rospy.init_node("control_node");
    pubyht1 = rospy.Publisher("/xingtian/yht1_velocity_controller/command", Float64,queue_size=10)
    pubyht2 = rospy.Publisher("/xingtian/yht2_position_controller/command", Float64,queue_size=10)
    pubyht3 = rospy.Publisher("/xingtian/yht3_position_controller/command", Float64,queue_size=10)
    pubyht4 = rospy.Publisher("/xingtian/yht4_position_controller/command", Float64,queue_size=10)
    pubyqt1 = rospy.Publisher("/xingtian/yqt1_velocity_controller/command", Float64,queue_size=10)
    pubyqt2 = rospy.Publisher("/xingtian/yqt2_position_controller/command", Float64,queue_size=10)
    pubyqt3 = rospy.Publisher("/xingtian/yqt3_position_controller/command", Float64,queue_size=10)
    pubyqt4 = rospy.Publisher("/xingtian/yqt4_position_controller/command", Float64,queue_size=10)
    pubzht1 = rospy.Publisher("/xingtian/zht1_velocity_controller/command", Float64,queue_size=10)
    pubzht2 = rospy.Publisher("/xingtian/zht2_position_controller/command", Float64,queue_size=10)
    pubzht3 = rospy.Publisher("/xingtian/zht3_position_controller/command", Float64,queue_size=10)
    pubzht4 = rospy.Publisher("/xingtian/zht4_position_controller/command", Float64,queue_size=10)
    pubzqt1 = rospy.Publisher("/xingtian/zqt1_velocity_controller/command", Float64,queue_size=10)
    pubzqt2 = rospy.Publisher("/xingtian/zqt2_position_controller/command", Float64,queue_size=10)
    pubzqt3 = rospy.Publisher("/xingtian/zqt3_position_controller/command", Float64,queue_size=10)
    pubzqt4 = rospy.Publisher("/xingtian/zqt4_position_controller/command", Float64,queue_size=10)
    # 订阅话题消息 用来关节角度和速度控制。
    global zqt_ctheta1,zqt_ctheta2,zht_ctheta1,zht_ctheta2
    global yht_ctheta1,yht_ctheta2,yqt_ctheta1,yqt_ctheta2
    zqt_ctheta1=zqt_ctheta2=zht_ctheta1=zht_ctheta2 = 0.0
    yht_ctheta1=yht_ctheta2=yqt_ctheta1=yqt_ctheta2 = 0.0
    joint_control = rospy.Subscriber("/xingtian/joint_control",Float64MultiArray,angle_insolve_callback)
    pi = math.pi
    # 初始化角度关节
    last_msg_time = rospy.Time(0)  # 初始时间戳为0
    joint_angles = [0.0 ,0.0 ,0.0 ,     #zqt_joint2, zqt_joint3, zqt_joint4),  1是往前抬  小腿往后抬
                    0.0 ,0.0 ,0.0 ,     #(zht_joint2, zht_joint3, zht_joint4), 1是往前抬腿
                    0.0 ,0.0 ,0.0 ,  #yht_joint2, yht_joint3, yht_joint4),  1是往前抬
                    0.0 ,0.0 ,0.0 ]  #(yqt_joint2, yqt_joint3, yqt_joint4)  1是往前抬
    global velocity_wheel
    velocity_wheel = [0.0,0.0,0.0,0.0] #zqt_wheel ,zht_wheel ,yht_wheel ,yqt_wheel        
    zqtvelocity = 0
    zhtvelocity = 0
    yhtvelocity = 0
    yqtvelocity = 0
    velocity_control = rospy.Subscriber("/xingtian/velocity_control",String,velocity_callback)
    
    # rate = rospy.Rate(1)
    rospy.loginfo("start control!");
    # addmition_control.addmition_control()
    while not rospy.is_shutdown():
        #下边将数值改为对应的变量，单位为弧度r
        msg = Float64()
        msg.data = [0.0]*16
        # 这里
        # 超时1秒未收到指令就停下来
        velocity_control_stop()
        # 模拟并联机构后实际控制机构
        # joint_angles[0] = -zqt_ctheta1*pi/180;                     #小腿  第二部分是弥补坐标系的转动
        # joint_angles[1] = -zqt_ctheta2*pi/180-zqt_ctheta1*pi/180;               #大腿
        # joint_angles[2] = zqt_ctheta1*pi/180;                           #髋关节   
        # # print(joint_angles[0]*180/pi,joint_angles[1]*180/pi,joint_angles[2]*180/pi)
        # joint_angles[3] =  -zht_ctheta1*pi/180; 
        # joint_angles[4] = -zht_ctheta2*pi/180-zht_ctheta1*pi/180;
        # joint_angles[5] = zht_ctheta1*pi/180;  
        # # print(joint_angles[3]*180/pi,joint_angles[4]*180/pi,joint_angles[5]*180/pi)
        # joint_angles[6] = -yht_ctheta1*pi/180;  
        # joint_angles[7] = -yht_ctheta2*pi/180-yht_ctheta1*pi/180;
        # joint_angles[8] = -yht_ctheta1*pi/180; 
        # # print(joint_angles[6]*180/pi,joint_angles[7]*180/pi,joint_angles[8]*180/pi)
        # joint_angles[9] = -yqt_ctheta1*pi/180;   
        # joint_angles[10] = -yqt_ctheta2*pi/180+yqt_ctheta1*pi/180;
        # joint_angles[11] = yqt_ctheta1*pi/180;
        # print(joint_angles[9]*180/pi,joint_angles[10]*180/pi,joint_angles[11]*180/pi)
        # 能够实现matlab逆解的角度  串联机构控制
        joint_angles[0] = -zqt_ctheta1*pi/180;                     #小腿  
        joint_angles[1] = -zqt_ctheta2*pi/180;                #大腿
        joint_angles[2] =  0;                   #髋关节   
        joint_angles[3] = -zht_ctheta1*pi/180; 
        joint_angles[4] = -zht_ctheta2*pi/180;
        joint_angles[5] = 0;  
        joint_angles[6] = -yht_ctheta1*pi/180;  
        joint_angles[7] = -yht_ctheta2*pi/180;
        joint_angles[8] = 0; 
        joint_angles[9] = -yqt_ctheta1*pi/180;   
        joint_angles[10] = -yqt_ctheta2*pi/180;
        joint_angles[11] = 0;
        
        # 初始化速度    数字是正的就往前转，数字是负的往后转
        
        # 左前腿
        msg.data[0] =-velocity_wheel[0];
        msg.data[1] =joint_angles[0];
        msg.data[2] =joint_angles[1];
        msg.data[3] =joint_angles[2];
        # 左后腿
        msg.data[4] =-velocity_wheel[1];
        msg.data[5] =joint_angles[3];
        msg.data[6] =joint_angles[4];
        msg.data[7] =joint_angles[5];
        # 右后腿
        msg.data[8] =velocity_wheel[2];
        msg.data[9] =joint_angles[6];
        msg.data[10] =joint_angles[7];
        msg.data[11] =joint_angles[8];
        # 右前腿
        msg.data[12] =velocity_wheel[3];
        msg.data[13] =joint_angles[9];
        msg.data[14] =joint_angles[10];
        msg.data[15] =joint_angles[11];
        #发布话题消息
        pubzqt1.publish(msg.data[0])
        # pubzqt2.publish(-2.0)
        pubzqt2.publish(msg.data[1])
        pubzqt3.publish(msg.data[2])
        # pubzqt3.publish(-0.2)
        pubzqt4.publish(msg.data[3])
        pubzht1.publish(msg.data[4])
        # pubzht2.publish(0.5)
        pubzht2.publish(msg.data[5])
        pubzht3.publish(msg.data[6])
        pubzht4.publish(msg.data[7])
        pubyht1.publish(msg.data[8])
        # pubyht2.publish(-0.5)
        pubyht2.publish(msg.data[9])
        pubyht3.publish(msg.data[10])
        pubyht4.publish(msg.data[11])
        pubyqt1.publish(msg.data[12])
        # pubyqt2.publish(-1.5)
        pubyqt2.publish(msg.data[13])
        pubyqt3.publish(msg.data[14])
        pubyqt4.publish(msg.data[15])
        # rate.sleep()
    rospy.spin()
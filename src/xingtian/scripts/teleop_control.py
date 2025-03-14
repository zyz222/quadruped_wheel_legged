#!/usr/bin/env python3
# -*- coding: utf-8 -*

import os
import sys
import rospy
import tty
import termios
from std_msgs.msg import String

# 初始化ROS节点
rospy.init_node('keyboard_publisher')

# 创建一个ROS话题发布者，用于发布键盘输入消息
pub_keyboard_velocity_input = rospy.Publisher('/xingtian/velocity_control', String, queue_size=10)
pub_keyboard_posture_input = rospy.Publisher('/xingtian/posture_control', String, queue_size=10)
# 读取键盘输入并发布消息
def keyboard_loop():
    # 显示提示信息
    print("Reading from keyboard")
    print("Press q to quit")
    print("wasd 控制方向 123切换姿态控制方法 zxcvbn 控制xyz方向的偏移,f恢复初始姿态")
    # 读取按键循环
    while not rospy.is_shutdown():
        # 获取标准输入的文件描述符
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        # 设置终端为原始模式，以便读取单个字符
        tty.setraw(fd)
        try:
            # 从标准输入读取一个字符
            ch1 = sys.stdin.read(1)
            ch2 = sys.stdin.read(1)
            # 如果按下了 q 键，则退出循环
            if ch1 == 'q':
                break
            # 在这里添加处理逻辑
            if ch1 == 'w':
                print("前进")
            if ch1 == 'a':
                print("左转")
            if ch1 == 's':
                print("后退")
            if ch1 == 'd':
                print("右转")
            if ch1 == 'p':
                print("停止运动")
            if ch2 == 'z':
                print("x方向偏移+10")
            if ch2 == 'x':
                print("x方向偏移-10")
            if ch2 == 'c':
                print("y方向偏移+10")
            if ch2 == 'v':
                print("y方向偏移-10")
            if ch2 == 'b':
                print("z方向偏移+10")
            if ch2 == 'n':
                print("z方向偏移-10")
            if ch2 == '1':
                print("绝对位置姿态控制")
            if ch2 == '2':
                print("增量式位置姿态控制")
            if ch2 == '3':
                print("平衡姿态控制")
            if ch2 == 'f':
                print("恢复初始姿态")
            # 将读取到的字符发布到话题中
            pub_keyboard_velocity_input.publish(ch1)
            pub_keyboard_posture_input.publish(ch2)
        finally:
            # 恢复终端设置
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    try:
        # 调用键盘输入循环函数
        keyboard_loop()
    except rospy.ROSInterruptException:
        pass
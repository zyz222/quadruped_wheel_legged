#! /user/bin/env/ python3
#coding=utf-8
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
import tf.transformations as tf
import math
import numpy as np
from std_msgs.msg import String
import force_VMC
from sensor_msgs.msg import JointState
pi = math.pi
# 处理imu数据，获取旋转矩阵
def imu_callback(data):
    # 获取四元数消息中的姿态信息
    orientation_q = data.orientation
    # 将四元数消息转换为四元数
    quaternion = (
        orientation_q.x,
        orientation_q.y,
        orientation_q.z,
        orientation_q.w)
    # 将四元数转换为欧拉角
    euler = tf.euler_from_quaternion(quaternion)
    roll_cal = math.atan2(2*(orientation_q.y*orientation_q.z+orientation_q.w*orientation_q.x),(orientation_q.w*orientation_q.w-orientation_q.x*orientation_q.x-orientation_q.y*orientation_q.y+orientation_q.z*orientation_q.z))
    pitch_cal = math.asin(-2*(orientation_q.x*orientation_q.z-orientation_q.w*orientation_q.y))
    yaw_cal = -math.atan2(2*( orientation_q.x*orientation_q.y+orientation_q.w*orientation_q.z),(orientation_q.w*orientation_q.w+orientation_q.x*orientation_q.x-orientation_q.y*orientation_q.y-orientation_q.z*orientation_q.z))
    # 输出欧拉角
    # global roll,pitch
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
    # global rotationzyx
    rotationzyx = euler_to_rotMatzyx(0,pitch,roll)
    print("Received Euler Angles (roll, pitch, yaw):", roll, pitch, yaw)
    # 这里要加一个新的旋转矩阵用于车身自平衡
    initialized = False
    # 在初始化时给变量赋值
    if not initialized:
        roll_cur = roll
        pitch_cur = pitch
        initialized = True
    roll_cur  = roll_cur - 0.5*roll
    pitch_cur = pitch_cur - 0.5*pitch
    print("---------------------------------------------")
    print(roll_cur,pitch_cur)
    rotation_balance = euler_to_rotMatzyx(0,pitch_cur,roll_cur)
    #画图用
   
    print(rotationzyx)
    leg_vector_solve(rotationzyx,rotation_balance,roll,pitch)
    pass
    # rospy.sleep(0.2)
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
    rotMatzyx = np.dot(Rx_roll, np.dot(Ry_pitch,Rz_yaw))
    return rotMatzyx

# 空间向量求解,末端坐标以及关节角度
# 这里应该写一个订阅和一个发布。4.9完成。。
def adjust_height(pitch,base_pos_z):
    adjust_high = 100*math.sin(pitch)
    pos[2] = base_pos_z + adjust_high
    return pos[2],adjust_high
def leg_vector_solve(rotation,rotation_balance,roll,pitch):
    # 定义轮腿车结构参数
    global high
    length = 418
    width_yao = 165
    width_foot = 400
    # 机身高度
    # high = 200
    roll_l =0;pitch_l= 0;
    k = 0;p = 0;
    # print(high)
    base_pos_z = pos[2]
    pos[2],adjust_high = adjust_height(pitch,250)
    # 下边这个计算是错误的，但是为了仿真就这么写了
    adjust_wheel_length = abs(adjust_high*math.tan(pitch))*5
    # adjust_wheel_length = abs(adjust_high/math.cos(pitch))+20
    rospy.logwarn(adjust_wheel_length)
    angle_ola  = Float64MultiArray()
    angle_ola.data = [pos[2],roll*180/pi,pitch*180/pi,adjust_wheel_length,adjust_high]
    angle_oula1.publish(angle_ola)
    print("--------------------------------------------------------------------")
    # print(adjust_high)
    rospy.logwarn(pos[2])
    # 定义目标位姿 就是质心在地面的投影到质心的向量坐标。竖直方向的
    # pos = np.mat([-10.0, 0.0, high]).T  # 示例期望位置
    if pos[1] != 0 :
        roll_l = math.atan2(pos[2], pos[1])
    if pos[0] != 0 :
        pitch_l = math.atan2(pos[2], pos[0])
        # print(pitch)
    if roll_l != 0:
        k = width_foot/math.tan(roll_l);
    if pitch_l != 0 :
        p = length/math.tan(pitch_l);
    body_stru = np.mat([[length/2, width_yao/2, -36],      # 左前
                        [-length/2, width_yao/2, -36],     # 左后
                        [-length/2, -width_yao/2, -36],    # 右后
                        [length/2, -width_yao/2, -36]]).T  # 右前

    footpoint_struc = np.mat([[length/2, width_foot/2, 0],      # 左前
                             [-length/2, width_foot/2, 0],     # 左后
                             [-length/2, -width_foot/2, 0],    # 右后
                             [length/2, -width_foot/2, 0]]).T  # 右前
    # 质心到足端的坐标。
    pos_footpoint = np.mat([[length/2,width_foot/2,-286],
                            [-length/2,width_foot/2,-286],
                            [-length/2,-width_foot/2,-286],
                            [length/2,-width_foot/2,-286]]).T
    # foot_body_struc = np.mat([[length/2, width_foot/2, -36],      # 左前
    #                          [-length/2, width_foot/2, -36],     # 左后
    #                          [-length/2, -width_foot/2, -36],    # 右后
    #                          [length/2, -width_foot/2, -36]]).T
    global model_switch
    tf_ab = np.mat(np.zeros((3, 4)))
    # 欧拉角的阈值
    threhold_rpy = 0.01
    # 坐标的阈值
    threshold = 1
    if model_switch == '1':
        for i in range(4):
            # print(000000000000000000000)
        # 在初始直立的姿态进行控制。相对于直立的姿态，缺点是不知道目前的状态，都是以原点为基础进行运算的。
        # 相当于pos给的是绝对值。
            tf_ab[:,i] = -pos - body_stru[:,i] + footpoint_struc[:,i]
    if model_switch == '2': 
        # 相当于pos给的是增量,目前不用。
        for i in range(4):
            # print(111111111111111111111111)
            tf_ab[:,i] = -pos - np.dot(rotation,body_stru[:,i])+footpoint_struc[:,i]
    if model_switch == '3': 
        for i in range(4): 
            #这里有问题了！！！！！
            # print(222222222222222222222222222)
            tf_ab[:,i] = np.dot(rotation_balance,pos_footpoint[:,i]) - body_stru[:,i]
            
            # last_tf_ab  = tf_ab.copy()
            # if abs(roll) < threhold_rpy and abs(pitch) < threhold_rpy:
            #     tf_ab = last_tf_ab
            # else:
            #     tf_ab[:,i] = np.dot(rotation_balance,pos_footpoint[:,i]) - body_stru[:,i]
            # print(np.dot(rotation_balance,pos_footpoint[:,i]))
            # print(tf_ab)
        # convert = tf_ab.copy()
        # 将第一列和第二列位置互换
        # tf_ab[:,[0,2]]=convert[:,[2,0]]
        # tf_ab[:, [0, 1]] = tf_ab[:, [1, 0]]
        # # 将第三列和第四列位置互换
        # tf_ab[:,[1,3]]=convert[:,[3,1]]
        # tf_ab[:, [2, 3]] = tf_ab[:, [3, 2]]
    print(tf_ab)
    print(pos)
    print(model_switch)
    # 计算期望坐标
    global zqt_x, zqt_z, zht_x, zht_z, yht_x, yht_z, yqt_x, yqt_z
    # 初始化是否已经赋值的标志变量
    initialized = False
    # 在初始化时给变量赋值
    if not initialized:
        zqt_x, zht_x, yht_x, yqt_x = tf_ab[0, 0]+209, tf_ab[0, 1]-209, tf_ab[0, 2]-209, tf_ab[0, 3]+209
        zqt_z = -math.sqrt(tf_ab[2, 0]**2) + k/2 + p/2
        zht_z = -math.sqrt(tf_ab[2, 1]**2) + k/2 - p/2
        yht_z = -math.sqrt(tf_ab[2, 2]**2) - k/2 - p/2
        yqt_z = -math.sqrt(tf_ab[2, 3]**2) - k/2 + p/2
        initialized = True

    # 在更新值时更新上一时刻的值
    zqt_x_prev, zht_x_prev, yht_x_prev, yqt_x_prev = zqt_x, zht_x, yht_x, yqt_x
    zqt_z_prev, zht_z_prev, yht_z_prev, yqt_z_prev = zqt_z, zht_z, yht_z, yqt_z

    # 更新当前时刻的值
    zqt_x = tf_ab[0, 0] + 209
    zht_x = tf_ab[0, 1] - 209
    yht_x = tf_ab[0, 2] - 209
    yqt_x = tf_ab[0, 3] + 209

    # zqt_z = -math.sqrt((tf_ab[1, 0]-200)**2 + tf_ab[2, 0]**2) + k/2 + p/2
    # zht_z = -math.sqrt((tf_ab[1, 1]-200)**2 + tf_ab[2, 1]**2) + k/2 - p/2
    # yht_z = -math.sqrt((tf_ab[1, 2]+200)**2 + tf_ab[2, 2]**2) - k/2 - p/2
    # yqt_z = -math.sqrt((tf_ab[1, 3]+200)**2 + tf_ab[2, 3]**2) - k/2 + p/2

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
    zqt_z = max(min(zqt_z, -50), -350)
    zht_z = max(min(zht_z, -50), -350)
    yqt_z = max(min(yqt_z, -50), -350)
    yht_z = max(min(yht_z, -50), -350)
    zqt_x = smooth_coordinate(zqt_x_prev, zqt_x, threshold)+adjust_wheel_length
    zht_x = smooth_coordinate(zht_x_prev, zht_x, threshold)-adjust_wheel_length
    yht_x = smooth_coordinate(yht_x_prev, yht_x, threshold)-adjust_wheel_length
    yqt_x = smooth_coordinate(yqt_x_prev, yqt_x, threshold)+adjust_wheel_length
    zqt_z = smooth_coordinate(zqt_z_prev, zqt_z, threshold)
    zht_z = smooth_coordinate(zht_z_prev, zht_z, threshold)
    yht_z = smooth_coordinate(yht_z_prev, yht_z, threshold)
    yqt_z = smooth_coordinate(yqt_z_prev, yqt_z, threshold)
     #画图用
    print("左前、左后、右后、右前腿期望坐标：")
    print(zqt_x, zqt_z, zht_x, zht_z, yht_x, yht_z, yqt_x, yqt_z)
    point_msg = Float64MultiArray()
    point_msg.data = np.array([zqt_x, zqt_z, zht_x, zht_z, yht_x, yht_z, yqt_x, yqt_z])
    position_pub.publish(point_msg)
    force_VMC_solve()

def jointstate_callback(data):
    # 轮足机器人末端坐标正解
    # 转移到质心坐标系下的平移矩阵
    T1 = ([[1, 0, 0 ,209],[0, 1,0 ,-35.91],[0, 0, 1, 82.5],[0, 0, 0, 1]]);
    T2 = ([[1, 0, 0 ,-209],[0, 1,0 ,-35.91],[0, 0, 1, 82.5],[0, 0, 0, 1]]);
    T3 = ([[1, 0, 0 ,-209],[0, 1,0 ,-35.91],[0, 0, 1, -82.5],[0, 0, 0, 1]]);
    T4 = ([[1, 0, 0 ,209],[0, 1,0 ,-35.91],[0, 0, 1, -82.5],[0, 0, 0, 1]]);
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
        zqt_theta[1] = -float((zqt_joint3_position)*180/pi)-90;
    if "zht_joint2" in joint_names:
        joint1_index = joint_names.index("zht_joint2")
        zht_joint2_position = joint_positions[joint1_index]
        zht_theta[0] = -zht_joint2_position*180/pi;
    if "zht_joint3" in joint_names:
        joint1_index = joint_names.index("zht_joint3")
        zht_joint3_position = joint_positions[joint1_index]
        zht_theta[1] = -float((zht_joint3_position)*180/pi)-90;
    if "yht_joint2" in joint_names:
        joint1_index = joint_names.index("yht_joint2")
        yht_joint2_position = joint_positions[joint1_index]
        yht_theta[0] = -yht_joint2_position*180/pi;
    if "yht_joint3" in joint_names:
        joint1_index = joint_names.index("yht_joint3")
        yht_joint3_position = joint_positions[joint1_index]
        yht_theta[1] = -float((yht_joint3_position)*180/pi)-90;
    if "yqt_joint2" in joint_names:
        joint1_index = joint_names.index("yqt_joint2")
        yqt_joint2_position = joint_positions[joint1_index]
        yqt_theta[0] = -yqt_joint2_position*180/pi;
    if "yqt_joint3" in joint_names:
        joint1_index = joint_names.index("yqt_joint3")
        yqt_joint3_position = joint_positions[joint1_index]
        yqt_theta[1] = -float((yqt_joint3_position)*180/pi)-90;
    # print("---------------------------------------------------------------")
    # print("左前 左后 右后 右前 theta1 theta2:")
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
    rospy.sleep(0.1)
    pass
    # print(yqt_theta)
    # return zqt_theta,zht_theta,yht_theta,yqt_theta   
    # 正解
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
# 求解VMC扭矩
def force_VMC_solve():
    global zqt_coor,zht_coor,yht_coor,yqt_coor
    global zqt_theta,zht_theta,yht_theta,yqt_theta
    # Xe Ze 是期望位置，X，Z是实际位置，
    # print("------------------------------------------")
    # print(zqt_theta)
    # rospy.logwarn("torque计算---------------------------------------")
    # print(zqt_x)
    # def cal_vmc_single_leg(Xe,Ze,X,Z,theta2,theta3,X_last = 0,Z_last = 0):
    zqt_torque = force_VMC.cal_vmc_single_leg(zqt_x,zqt_z,zqt_coor[0],zqt_coor[1],zqt_theta[1],zqt_theta[0],0,0)
    zht_torque = force_VMC.cal_vmc_single_leg(zht_x,zht_z,zht_coor[0],zht_coor[1],zht_theta[1],zht_theta[0],0,0)
    yht_torque = force_VMC.cal_vmc_single_leg(yht_x,yht_z,yht_coor[0],yht_coor[1],yht_theta[1],yht_theta[0],0,0)
    yqt_torque = force_VMC.cal_vmc_single_leg(yqt_x,yqt_z,yqt_coor[0],yqt_coor[1],yqt_theta[1],yqt_theta[0],0,0)
    # print(zqt_torque)
    # print()
    # print(zht_torque)
    # print()
    # print(yht_torque)
    # print()
    # print(yqt_torque)
    publish_vmc_torque(zqt_torque,zht_torque,yht_torque,yqt_torque)
def publish_vmc_torque(zqt_torque,zht_torque,yht_torque,yqt_torque):

    data = Float64MultiArray()
    data.data = [
        zqt_torque[0],zqt_torque[1],
        zht_torque[0],zht_torque[1],
        yht_torque[0],yht_torque[1],
        yqt_torque[0],yqt_torque[1],
    ]
    vmc_torque.publish(data)
def posture_callback(msg):
    global high,pos,model_switch,adjust_high
    ch = msg.data
    # 姿态控制切换
    if ch == '1':
        model_switch = '1'
        pos = np.mat([0.0, 0.0, high]).T 
    if ch == '2':
        model_switch = '2'
        pos = np.mat([0.0, 0.0, high]).T 
    if ch == '3':
        model_switch = '3'
        pos = np.mat([0.0, 0.0, high]).T 
    if ch =='z':
        pos[0]+=10
    if ch =='x':
        pos[0]-=10
    if ch =='c':
        pos[1]+=10
    if ch =='v':
        pos[1]-=10
    if ch =='b':
        high+=10
        pos[2] = high 
    if ch =='n':
        high-=10
        pos[2] = high 
    if ch =='f':
        pos = np.mat([0.0, 0.0, high]).T 
    
    # 假设 pos 是一个包含三个坐标值的列表或数组

    # 设定边界值
    min_x, max_x = -100, 100
    min_y, max_y = -100, 100
    min_z, max_z = 50, 380

    # 遍历坐标的每个维度
    for i in range(3):
        # 判断并修正坐标值
        if i == 0:
            if pos[i] > max_x:
                pos[i] = max_x
            elif pos[i] < min_x:
                pos[i] = min_x

        if i == 1:
            if pos[i] < min_y:
                pos[i] = min_y
            elif pos[i] > max_y:
                pos[i] = max_y

        if i == 2:
            if pos[i] < min_z:
                pos[i] = min_z
            elif pos[i] > max_z:
                pos[i] = max_z

    print("pos位置坐标")
    print(pos)
    print("控制方式:",ch)
   

def main():
    rospy.init_node('pos_control_ros', anonymous=True)
    global high,pos,model_switch
    # roll = pitch =0.0
    high = 200
    pos = np.mat([0.0, 0.0, 0.0]).T 
    model_switch = None
    posture = rospy.Subscriber("/xingtian/posture_control",String,posture_callback)
    angle_feedback = rospy.Subscriber("/xingtian/joint_states",JointState,jointstate_callback)
    imu = rospy.Subscriber("/imu", Imu, imu_callback)
    global vmc_torque
    vmc_torque = rospy.Publisher("/xingtian/vmc_torque", Float64MultiArray,queue_size=10)
    global position_pub
    position_pub = rospy.Publisher("/xingtian/joint_control",Float64MultiArray,queue_size=10)
    global angle_oula1,angle_oula2,angle_oula3
    angle_oula1 = rospy.Publisher("/xingtian/angle_ola1",Float64MultiArray,queue_size=10)
    angle_oula2 = rospy.Publisher("/xingtian/angle_ola2",Float64,queue_size=10)
    angle_oula3 = rospy.Publisher("/xingtian/angle_ola3",Float64,queue_size=10)
    rospy.spin()
if __name__ =="__main__":
    main()
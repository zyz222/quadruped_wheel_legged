#!/usr/bin/env python3
# -*- coding: utf-8 -*
# walk步态，一个腿一个腿走
# import pydog_gait
from math import sin,cos,pi,tan
import pydog_attitude
#中间变量设定
x1_s=0;x2_s=0;x3_s=0;x4_s=0;y1_s=0;y2_s=0;y3_s=0;y4_s=0
xs=0
faai=0.25
Ts=4
Height_body = 250
max_G_x = 30
alpha = 0.5  #调整重心所用时间占比
def smooth_adjustment(t, start_t, end_t, max_adjustment):
        duration = end_t - start_t
        elapsed = t - start_t
        adjustment = (elapsed / duration) * max_adjustment
        return adjustment

    # if t < Ts * faai * alpha:  # 前半个周期调整重心
    #     G_x = smooth_adjustment(t, 0, Ts * faai * alpha, max_G_x)
    #     G_z = 0
    #     return 209 + G_x, -Height_body + G_z, -209 + G_x, -Height_body + G_z, -209 + G_x, -Height_body + G_z, 209 + G_x, -Height_body + G_z
def cal_adjust(CG_Y,l,xk,sita,period):   #计算重心偏移量函数,使得机器人重心投影始终落在三角形面
  result=(CG_Y+period*(l+xk)/4+110*tan(sita*1.5))  #计算高度暂时固定在典型的110,1.5是角度 pitch ? P环值
  #print(result)   #打印出调整量，用于调试
  return result

def cal_w(pitch,roll,xf,h,t,r1,r4,r2,r3):   #WALK步态主计算函数，相序 1-4-3-2
    # global d,z
    global x1_s,x2_s,x3_s,x4_s,y1_s,y2_s,y3_s,y4_s
    # 重心调整，重心前移

 #开始步态计算
    # if t<Ts*faai+Ts*faai*alpha:
    #     if t<Ts*faai*alpha:    #迈出腿1
    #         G_x = smooth_adjustment(t,0,Ts*faai*alpha,max_G_x)
    #         #print("腿1")
    #         G_z = 0
    #         return x1_s+209+G_x,y1_s-Height_body+G_z,x2_s-209+G_x,y2_s-Height_body+G_z,x3_s-209+G_x,y3_s-Height_body+G_z,x4_s+209+G_x,y4_s-Height_body+G_z
    #     else:    # else:
    #         print("11")
    #         sigma=2*pi*(t-Ts*faai*alpha)/(faai*Ts)
    #         zep=h*(1-cos(sigma))/2
    #         xep_b=(xf-xs)*((sigma-sin(sigma))/(2*pi))+xs
    #             #输出y
    #         y1=zep
    #         y2=0
    #         y3=0
    #         y4=0
    #             #输出x
    #         x1=xep_b*r1
    #         x2=0
    #         x3=0
    #         x4=0
    #         G_x = 50
    #         G_z = 0
    #         x1_s=x1;x2_s=x2;x3_s=x3;x4_s=x4;y1_s=y1;y2_s=y2;y3_s=y3;y4_s=y4
    #             # return x1_s+209,y1_s-Height_body,x2_s-209,y2_s-Height_body,x3_s-209,y3_s-Height_body,x4_s+209,y4_s-Height_body
    #         return x1_s+209+G_x,y1_s-Height_body+G_z,x2_s-209+G_x,y2_s-Height_body+G_z,x3_s-209+G_x,y3_s-Height_body+G_z,x4_s+209+G_x,y4_s-Height_body+G_z
    # elif t<2*Ts*faai+alpha*Ts*faai:    #迈出腿2
    #     #print("腿2")
    #     # if t<2*Ts*faai*(1+alpha)/2:
    #     #     G_x = smooth_adjustment(t,Ts*faai,2*Ts*faai*(1+alpha)/2,max_G_x)
    #     #     #print("腿1")
    #     #     G_z = 0
    #     #     return x1_s+209+G_x,y1_s-Height_body+G_z,x2_s-209+G_x,y2_s-Height_body+G_z,x3_s-209+G_x,y3_s-Height_body+G_z,x4_s+209+G_x,y4_s-Height_body+G_z
    #     #     # else:   
    #     # else:
    #         print("22")
    #         # pydog_gait.t=pydog_gait.t+speed
    #         # t=t-faai*Ts
    #         print(t)
    #         sigma=2*pi*(t-faai*Ts*alpha-Ts*faai)/(faai*Ts)
    #         zep=h*(1-cos(sigma))/2
    #         xep_b=(xf-xs)*((sigma-sin(sigma))/(2*pi))+xs
    #         #输出y
    #         y1=0
    #         y2=0
    #         y3=0
    #         y4=zep
    #         #输出x
    #         x1=xf
    #         x2=0
    #         x3=0
    #         x4=xep_b*r2
    #         G_x = 50
    #         G_z = 0
    #         x1_s=x1;x2_s=x2;x3_s=x3;x4_s=x4;y1_s=y1;y2_s=y2;y3_s=y3;y4_s=y4
    #         # return x1_s+209,y1_s-Height_body,x2_s-209,y2_s-Height_body,x3_s-209,y3_s-Height_body,x4_s+209,y4_s-Height_body
    #         return x1_s+209+G_x,y1_s-Height_body+G_z,x2_s-209+G_x,y2_s-Height_body+G_z,x3_s-209+G_x,y3_s-Height_body+G_z,x4_s+209+G_x,y4_s-Height_body+G_z
    # elif t<3*Ts*faai+2*Ts*faai*alpha:        #迈出腿3
    #     #print("腿3")
    #     if t<3*Ts*faai+Ts*faai*alpha:
    #         G_x = smooth_adjustment(t,2*Ts*faai+alpha*Ts*faai,3*Ts*faai+Ts*faai*alpha,-150)
    #         G_z = 0
    #         return x1_s+209+G_x,y1_s-Height_body+G_z,x2_s-209+G_x,y2_s-Height_body+G_z,x3_s-209+G_x,y3_s-Height_body+G_z,x4_s+209+G_x,y4_s-Height_body+G_z
    #     else:
    #         print("33")
    #         # pydog_gait.t=pydog_gait.t+speed
    #         # t=t-faai*Ts*2
    #         print(t)
    #         sigma=2*pi*(t-2*Ts*faai-2*alpha*Ts*faai)/(faai*Ts)
    #         zep=h*(1-cos(sigma))/2
    #         xep_b=(xf-xs)*((sigma-sin(sigma))/(2*pi))+xs
    #         #输出y
    #         y1=0
    #         y2=0
    #         y3=zep
    #         y4=0
    #         #输出x
    #         x1=xf
    #         x2=0
    #         x3=xep_b*r3
    #         x4=xf
    #         G_x = -150
    #         G_z = 0
    #         x1_s=x1;x2_s=x2;x3_s=x3;x4_s=x4;y1_s=y1;y2_s=y2;y3_s=y3;y4_s=y4
    #         # return x1_s+209,y1_s-Height_body,x2_s-209,y2_s-Height_body,x3_s-209,y3_s-Height_body,x4_s+209,y4_s-Height_body
    #         return x1_s+209+G_x,y1_s-Height_body+G_z,x2_s-209+G_x,y2_s-Height_body+G_z,x3_s-209+G_x,y3_s-Height_body+G_z,x4_s+209+G_x,y4_s-Height_body+G_z

    # elif t<4*Ts*faai+2*Ts*faai*alpha:    #迈出腿4
    #     print("腿4")
    #     # if abs(pitch)>0.2 or abs(roll) > 0.2:
    #     #     a = pydog_attitude.attitude_adjust(pitch,roll,Height_body)  #第三个是机身高度
    #     #     return a
    #     # else:
    #     print(t)
    #     sigma=2*pi*(t-3*Ts*faai-2*Ts*faai*alpha)/(faai*Ts)
    #     zep=h*(1-cos(sigma))/2
    #     xep_b=(xf-xs)*((sigma-sin(sigma))/(2*pi))+xs
    #         #输出y
    #     y1=0
    #     y2=zep
    #     y3=0
    #     y4=0
    #         #输出x
    #     x1=xf
    #     x2=xep_b*r4
    #     x3=xf
    #     x4=xf
    #     G_x = -150
    #     G_z = 0
    #     x1_s=x1;x2_s=x2;x3_s=x3;x4_s=x4;y1_s=y1;y2_s=y2;y3_s=y3;y4_s=y4
    #         # return x1_s+209,y1_s-Height_body,x2_s-209,y2_s-Height_body,x3_s-209,y3_s-Height_body,x4_s+209,y4_s-Height_body
    #     return x1_s+209+G_x,y1_s-Height_body+G_z,x2_s-209+G_x,y2_s-Height_body+G_z,x3_s-209+G_x,y3_s-Height_body+G_z,x4_s+209+G_x,y4_s-Height_body+G_z
    # elif t>=4*Ts*faai+2*Ts*faai*alpha:    #足端坐标归零
    #     if x1_s > 0 :
    #         x1_s = x1_s-x1_s*0.01
    #     elif x1_s<0:
    #         x1_s = x1_s+x1_s*0.01
        
    #     x1_s=x1_s;x2_s=x1_s;x3_s=x1_s;x4_s=x1_s;
    #     y1_s=0;y2_s=0;y3_s=0;y4_s=0
    #     # G_x = smooth_adjustment(t,4*Ts*faai+2*Ts*faai*alpha,4*Ts*faai+2*Ts*faai*alpha+1,100)
    #     G_x = 0
    #     G_z = 0
    #     return x1_s+209+G_x,y1_s-Height_body+G_z,x2_s-209+G_x,y2_s-Height_body+G_z,x3_s-209+G_x,y3_s-Height_body+G_z,x4_s+209+G_x,y4_s-Height_body+G_z
        
        # # t = 0 
        # if abs(pitch)>0.2 or abs(roll) > 0.2:
        #     a = pydog_attitude.attitude_adjust(pitch,roll,Height_body)  #第三个是机身高度
        #     return a
        # elif x1_s > 0 :
        #     x1_s = x1_s-x1_s*0.005
        # elif x1_s<0:
        #     x1_s = x1_s+x1_s*0.005
        
        # x1_s=x1_s;x2_s=x1_s;x3_s=x1_s;x4_s=x1_s;
        # y1_s=0;y2_s=0;y3_s=0;y4_s=0
        # return x1_s+209,y1_s-Height_body,x2_s-209,y2_s-Height_body,x3_s-209,y3_s-Height_body,x4_s+209,y4_s-Height_body
# 之前的walk
    #开始步态计算
    if t<Ts*faai:    #迈出腿1
        #print("腿1")
        # if abs(pitch) >0.2 or abs(roll) > 0.2:
        #     a = pydog_attitude.attitude_adjust(pitch,roll,Height_body)  #第三个是机身高度
        #     return a
        # else:
            print("11")
            sigma=2*pi*t/(faai*Ts)
            zep=h*(1-cos(sigma))/2
            xep_b=(xf-xs)*((sigma-sin(sigma))/(2*pi))+xs
            #输出y
            y1=zep
            y2=0
            y3=0
            y4=0
            #输出x
            x1=xep_b*r1
            x2=0
            x3=0
            x4=0
            G_x = 50
            G_z = 0
            x1_s=x1;x2_s=x2;x3_s=x3;x4_s=x4;y1_s=y1;y2_s=y2;y3_s=y3;y4_s=y4
            # return x1_s+209,y1_s-Height_body,x2_s-209,y2_s-Height_body,x3_s-209,y3_s-Height_body,x4_s+209,y4_s-Height_body
            return x1_s+209+G_x,y1_s-Height_body+G_z,x2_s-209+G_x,y2_s-Height_body+G_z,x3_s-209+G_x,y3_s-Height_body+G_z,x4_s+209+G_x,y4_s-Height_body+G_z
    elif t<2*Ts*faai:    #迈出腿2
        #print("腿2")
        if abs(pitch) >0.2 or abs(roll) >0.2:
            a = pydog_attitude.attitude_adjust(pitch,roll,Height_body)  #第三个是机身高度
            return a
        else:
            print("22")
            # pydog_gait.t=pydog_gait.t+speed
            # t=t-faai*Ts
            print(t)
            sigma=2*pi*(t-faai*Ts)/(faai*Ts)
            zep=h*(1-cos(sigma))/2
            xep_b=(xf-xs)*((sigma-sin(sigma))/(2*pi))+xs
            #输出y
            y1=0
            y2=0
            y3=0
            y4=zep
            #输出x
            x1=xf
            x2=0
            x3=0
            x4=xep_b*r2
            G_x = 50
            G_z = 0
            x1_s=x1;x2_s=x2;x3_s=x3;x4_s=x4;y1_s=y1;y2_s=y2;y3_s=y3;y4_s=y4
            # return x1_s+209,y1_s-Height_body,x2_s-209,y2_s-Height_body,x3_s-209,y3_s-Height_body,x4_s+209,y4_s-Height_body
            return x1_s+209+G_x,y1_s-Height_body+G_z,x2_s-209+G_x,y2_s-Height_body+G_z,x3_s-209+G_x,y3_s-Height_body+G_z,x4_s+209+G_x,y4_s-Height_body+G_z
    elif t<3*Ts*faai:    #迈出腿3
        #print("腿3")
        if abs(pitch) >0.2 or abs(roll) > 0.2:
            a = pydog_attitude.attitude_adjust(pitch,roll,Height_body)  #第三个是机身高度
            return a
        else:
            print("33")
            # pydog_gait.t=pydog_gait.t+speed
            # t=t-faai*Ts*2
            print(t)
            sigma=2*pi*(t-2*Ts*faai)/(faai*Ts)
            zep=h*(1-cos(sigma))/2
            xep_b=(xf-xs)*((sigma-sin(sigma))/(2*pi))+xs
            #输出y
            y1=0
            y2=0
            y3=zep
            y4=0
            #输出x
            x1=xf
            x2=0
            x3=xep_b*r3
            x4=xf
            G_x = -50
            G_z = 0
            x1_s=x1;x2_s=x2;x3_s=x3;x4_s=x4;y1_s=y1;y2_s=y2;y3_s=y3;y4_s=y4
            # return x1_s+209,y1_s-Height_body,x2_s-209,y2_s-Height_body,x3_s-209,y3_s-Height_body,x4_s+209,y4_s-Height_body
            return x1_s+209+G_x,y1_s-Height_body+G_z,x2_s-209+G_x,y2_s-Height_body+G_z,x3_s-209+G_x,y3_s-Height_body+G_z,x4_s+209+G_x,y4_s-Height_body+G_z

    elif t<4*Ts*faai:    #迈出腿4
        print("腿4")
        if abs(pitch)>0.2 or abs(roll) > 0.2:
            a = pydog_attitude.attitude_adjust(pitch,roll,Height_body)  #第三个是机身高度
            return a
        else:
            print(t)
            sigma=2*pi*(t-3*Ts*faai)/(faai*Ts)
            zep=h*(1-cos(sigma))/2
            xep_b=(xf-xs)*((sigma-sin(sigma))/(2*pi))+xs
            #输出y
            y1=0
            y2=zep
            y3=0
            y4=0
            #输出x
            x1=xf
            x2=xep_b*r4
            x3=xf
            x4=xf
            G_x = -50
            G_z = 0
            x1_s=x1;x2_s=x2;x3_s=x3;x4_s=x4;y1_s=y1;y2_s=y2;y3_s=y3;y4_s=y4
            # return x1_s+209,y1_s-Height_body,x2_s-209,y2_s-Height_body,x3_s-209,y3_s-Height_body,x4_s+209,y4_s-Height_body
            return x1_s+209+G_x,y1_s-Height_body+G_z,x2_s-209+G_x,y2_s-Height_body+G_z,x3_s-209+G_x,y3_s-Height_body+G_z,x4_s+209+G_x,y4_s-Height_body+G_z
    elif t>=4*Ts*faai:    #足端坐标归零
        # t = 0 
        if abs(pitch)>0.2 or abs(roll) > 0.2:
            a = pydog_attitude.attitude_adjust(pitch,roll,Height_body)  #第三个是机身高度
            return a
        elif x1_s > 0 :
            x1_s = x1_s-x1_s*0.005
        elif x1_s<0:
            x1_s = x1_s+x1_s*0.005
        
        x1_s=x1_s;x2_s=x1_s;x3_s=x1_s;x4_s=x1_s;
        y1_s=0;y2_s=0;y3_s=0;y4_s=0
        return x1_s+209,y1_s-Height_body,x2_s-209,y2_s-Height_body,x3_s-209,y3_s-Height_body,x4_s+209,y4_s-Height_body

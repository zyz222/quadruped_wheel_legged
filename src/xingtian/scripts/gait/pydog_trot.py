#!/usr/bin/env python3
# -*- coding: utf-8 -*
from math import sin,cos,pi

faai=0.5
Ts=1

def cal_t(t,xs,xf,h,r1,r2,r3,r4):    #小跑步态执行函数  xs是足端起点，xf是足端落点。
    if t<=Ts*faai:
        sigma=2*pi*t/(faai*Ts)
        zep=h*(1-cos(sigma))/2
        xep_b=(xf-xs)*((sigma-sin(sigma))/(2*pi))+xs
        xep_z=(xs-xf)*((sigma-sin(sigma))/(2*pi))+xf
        #输出y
        y1=zep
        y2=0
        y3=zep
        y4=0
        #输出x
        x1=-xep_z*r1
        x2=-xep_b*r2
        x3=-xep_z*r3
        x4=-xep_b*r4
        return x1+209,y1-300,x2-209,y2-300,x3-209,y3-300,x4+209,y4-300
    elif t>Ts*faai and t<=Ts:
        sigma=2*pi*(t-Ts*faai)/(faai*Ts)
        zep=h*(1-cos(sigma))/2;
        xep_b=(xf-xs)*((sigma-sin(sigma))/(2*pi))+xs
        xep_z=(xs-xf)*((sigma-sin(sigma))/(2*pi))+xf
        #输出y
        y1=0
        y2=zep
        y3=0
        y4=zep
        #输出x
        x1=-xep_b*r1
        x2=-xep_z*r2
        x3=-xep_b*r3
        x4=-xep_z*r4
        return x1+209,y1-300,x2-209,y2-300,x3-209,y3-300,x4+209,y4-300

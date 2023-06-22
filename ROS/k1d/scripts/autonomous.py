#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from cmath import inf
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
erro = 0
scan_data = None
dir = 1
velocidade_linear = 0.15
velocidade_angular = 0.3
erro_max_frente = 1.5
dist_bump = 0.17

def filtroinf(x):
    if x == inf:
        return 3.5
    else:
        return x

def scaneou(dado):
    global erro, scan_data
    scan_data = dado.ranges
    readings = dado.ranges
    readings = list(map(lambda x: filtroinf(x), readings))
    #print(f"75 graus: {readings[75]}, 105 graus: {readings[105]}, 225 graus: {readings[225]} e 285 graus: {readings[285]}")
    #print(f"0 graus: {readings[0]}, 90 graus: {readings[90]}, 180 graus: {readings[180]} e 270 graus: {readings[270]}")
    if dir == -1:
        if readings[75] > readings[285]:
            try:
                erro = readings[75]/readings[285]
            except ZeroDivisionError:
                erro = 999
        else:
            try:
                erro = -readings[285]/readings[75]
            except ZeroDivisionError:
                erro = -999
    elif dir == 1:
        if readings[105] > readings[255]:
            try:
                erro = readings[105]/readings[255]
            except ZeroDivisionError:
                erro = 999
        else:
            try:
                erro = -readings[255]/readings[105]
            except ZeroDivisionError:
                erro = -999
    print(f"erro: {erro}, dir: {dir}")


if __name__=="__main__":
    rospy.init_node("autonomous")
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    last = rospy.Time.now()
    while not rospy.is_shutdown():
        if dir == 1 and scan_data is not None and scan_data[0] < dist_bump:
            if erro < erro_max_frente:
                print('bump')
                dir = -1
                velocidade = Twist(Vector3(velocidade_linear, 0, 0), Vector3(0, 0, 0))
                velocidade_saida.publish(velocidade)
                rospy.sleep(0.5)
                continue
            else:
                erro *= 9999
        elif dir == -1 and scan_data is not None and scan_data[180] < dist_bump:
            if erro < erro_max_frente:
                print('bump')
                dir = 1
                velocidade = Twist(Vector3(-velocidade_linear, 0, 0), Vector3(0, 0, 0))
                velocidade_saida.publish(velocidade)
                rospy.sleep(0.5)
                continue
            else:
                erro *= 9999
        if abs(erro) < erro_max_frente:
            velocidade = Twist(Vector3(dir *  velocidade_linear, 0, 0), Vector3(0, 0, 0))
            velocidade_saida.publish(velocidade)
        elif erro > 0:
            velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, velocidade_angular))
        else:
            velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, -velocidade_angular))
        velocidade_saida.publish(velocidade)
        rospy.sleep(0.1)

#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from cmath import inf
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
erro = 0
K = 0.3
scan_data = None
dir = 1

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
    if dir == 1:
        if readings[75] > readings[285]:
            erro = readings[75]/readings[285]
        else:
            erro = -readings[285]/readings[75]
    else:        
        if readings[105] > readings[255]:
            erro = -readings[105]/readings[255]
        else:
            erro = readings[255]/readings[105]
    print(erro)


if __name__=="__main__":
    rospy.init_node("autonomous")
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    last = rospy.Time.now()
    while not rospy.is_shutdown():
        if dir == 1 and scan_data is not None and scan_data[0] < 0.15:
            if erro < 3:
                print('bump')
                dir = -1
                velocidade = Twist(Vector3(0.15, 0, 0), Vector3(0, 0, 0))
                velocidade_saida.publish(velocidade)
                rospy.sleep(0.5)
                continue
            else:
                erro *= 9999
        elif dir == -1 and scan_data is not None and scan_data[180] < 0.15:
            if erro < 3:
                print('bump')
                dir = 1
                velocidade = Twist(Vector3(-0.15, 0, 0), Vector3(0, 0, 0))
                velocidade_saida.publish(velocidade)
                rospy.sleep(0.5)
                continue
            else:
                erro *= 9999
        if abs(erro) < 3:
            velocidade = Twist(Vector3(dir *  0.15, 0, 0), Vector3(0, 0, 0))
            velocidade_saida.publish(velocidade)
        elif erro > 0:
            velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, K))
        else:
            velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, -K))
        velocidade_saida.publish(velocidade)
        #velocidade_saida.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
        rospy.sleep(0.1)



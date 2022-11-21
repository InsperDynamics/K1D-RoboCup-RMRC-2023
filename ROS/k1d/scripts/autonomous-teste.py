#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from cmath import inf
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
erro = 0
K = 0.030
scan_data = None
dir = 1
off = False



def filtroinf(x):
    if x == inf:
        return 3.5
    else:
        return x

def scaneou(dado):
    global erro, scan_data, off
    scan_data = dado.ranges
    readings = dado.ranges
    readings = list(map(lambda x: filtroinf(x), readings))
    if off:
        erro = sum(readings[220:230]) - sum(readings[130:140])
    else:        
        erro = sum(readings[40:50]) - sum(readings[310:320])
    print("Erro: " + str(erro))

def check_bump():
    global scan_data
    if scan_data != None:
        #print(min(scan_data[5:355]))
        return min(scan_data[5:355]) < 0.2
    return False



if __name__=="__main__":
    rospy.init_node("autonomous")
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    last = rospy.Time.now()
    while not rospy.is_shutdown():
        if check_bump() and (rospy.Time.now() - last > rospy.Duration(5)):
            print("bump")
            K *= -1
            off = not off
            last = rospy.Time.now()
        if abs(erro) < 3:
            #print('reto')
            if off:
                velocidade = Twist(Vector3(-0.3, 0, 0), Vector3(0, 0, 0))
            else:       
                velocidade = Twist(Vector3(0.3, 0, 0), Vector3(0, 0, 0))
            velocidade_saida.publish(velocidade)
            rospy.sleep(0.1)
            continue
        else:
            #print("girando")
            velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, erro*K))
        velocidade_saida.publish(velocidade)
        rospy.sleep(0.1)



#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time

import rospy
from std_msgs.msg import Float32

def calcolaR(a, b, t):
    return a*(t**2) + b*t,

def talker():
    start_time = time.time()
    pub = rospy.Publisher('risultato', Float32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    a = input("Valore di a:")
    b = input("Valore di b:")
    while not rospy.is_shutdown():
        tempo = time.time() - start_time
        tempo_float = calcolaR(a, b, tempo)[0] #Non capisco perche restituisce un array
        rospy.loginfo(tempo_float)
        pub.publish(tempo_float)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

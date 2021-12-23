#! /usr/bin/env python3
# -- coding:utf-8 --

import rospy
from geometry_msgs.msg import Twist, Vector3

v = 0.1  # Velocidade linear
w = 1.5708 # Velocidade angular
rel = 0

if __name__ == "__main__":
    rospy.init_node("roda_exemplo")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)

    try:
        while not rospy.is_shutdown():
            if rel <= 10:
                vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
                pub.publish(vel)
            elif rel > 10 and rel <= 15:
                vel = Twist(Vector3(0,0,0), Vector3(0,0,w))
                pub.publish(vel)
                rel = 0

            rel += 1
            rospy.sleep(1.0)
            #print(rel)
    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
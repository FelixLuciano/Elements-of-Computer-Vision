#! /usr/bin/env python3
# -*- coding:utf-8 -*-

# Rodar com 
# roslaunch my_simulation rampa.launch


from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped

from nav_msgs.msg import Odometry
from std_msgs.msg import Header


import visao_module


bridge = CvBridge()

cv_image = None
media = []
origem_x = 0
centro = [0, 0]

range_red = ((-20/2, 100, 60), (20/2, 255, 255))
range_green = ((100/2, 100, 100), (140/2, 255, 255))
range_blue = ((150/2, 50, 100), (210/2, 255, 255))
range_yelow = ((35/2, 50, 100), (70/2, 255, 255))

ve_cor = True
active_color_range = range_green
next_color_range = range_yelow

area = 0.0 # Variavel com a area do maior contorno

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

x = 0
y = 0
z = 0 
id = 0


# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    # print("frame")
    global cv_image
    global origem_x
    global media
    global centro
    global active_color_range
    global next_color_range
    global ve_cor
    global resultados

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs

    try:
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        cv_image = temp_image.copy()
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        active_color_mask = cv2.inRange(hsv, active_color_range[0], active_color_range[1])
        active_color_contours, tree1 = cv2.findContours(active_color_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        next_color_mask = cv2.inRange(hsv, next_color_range[0], next_color_range[1])
        next_color_contours, tree2 = cv2.findContours(next_color_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        

        if active_color_contours is not None and len(active_color_contours) > 0:
            ve_cor = True
            M = cv2.moments(active_color_contours[0])

            if M["m00"] != 0.0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centro = (cx, cy)
                cv2.circle(cv_image, centro, 5, (255, 255, 255), 3)

            active_color_area = cv2.contourArea(active_color_contours[0])

            if next_color_contours != None and len(next_color_contours) > 0:
                next_color_area = cv2.contourArea(next_color_contours[0])

                if next_color_area > active_color_area / 10:
                    print("Proxima cor detectada")
                    active_color_range = next_color_range

                    if next_color_range == range_yelow:
                        next_color_range = range_blue
                    elif next_color_range == range_blue:
                        next_color_range = range_red
                    elif next_color_range == range_red:
                        next_color_range = range_green
                    elif next_color_range == range_green:
                        next_color_range = range_yelow

        else:
            ve_cor = False

        origem_y = int(cv_image.shape[0] / 2)
        origem_x = int(cv_image.shape[1] / 2)
        cv2.circle(cv_image, (origem_x, origem_y), 5, (0, 0, 0), 3)

        # ATENÇÃO: ao mostrar a imagem aqui, não podemos usar cv2.imshow() dentro do while principal!! 
        cv2.imshow("cv_image", cv_image)
        cv2.imshow("active", active_color_mask)
        cv2.imshow("nect", next_color_mask)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)
        pass

scan = []

def scan_lidar(scan_new):
    global scan
    scan = scan_new.ranges
    
v = 0.4
w = 0

if __name__=="__main__":
    rospy.init_node("Q3")

    topico_imagem = "/camera/image/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scan_lidar)

    try:
        vel = Twist(Vector3(v,0,0), Vector3(0,0,w))
        
        while not rospy.is_shutdown():
            if len(scan) > 0:
                short_r = scan[345:360]
                short_l = scan[0:15]
                min_short_r = min(short_r)
                min_short_l = min(short_l)
                
                wide_r = scan[270:360]
                wide_l = scan[0:90]
                min_wide_r = min(wide_r)
                min_wide_l = min(wide_l)

                if ve_cor:
                    if centro[0] - origem_x >= 0:
                        w = -0.5
                    else:
                        w = 0.5

                if min_short_r < 1.0 or min_short_l < 1.0:
                    v = 0
                    w = 0.5
                elif min_wide_r < 0.5:
                    v = 0.0
                    w = 0.3
                elif min_wide_l < 0.5:
                    v = 0.0
                    w = -0.3
                else:
                    v = 0.4
            
            vel = Twist(Vector3(v,0,0), Vector3(0,0,w))

            velocidade_saida.publish(vel)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")



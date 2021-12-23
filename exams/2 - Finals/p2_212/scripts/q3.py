#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import numpy as np
import rospy
import cv2

from lib.robot import Robot
from lib.indeciso import Indeciso
from lib.line_follower import LineFollower
from lib.aruco import Aruco
from lib.odometry import Odometry



class Robot_Q3(Robot, Indeciso, LineFollower, Aruco, Odometry):
    def __init__(self, *arks, **kwargs):
        super(Robot_Q3, self).__init__(*arks, **kwargs)

        print("inicializando...")

        self.origin = None
        self.ids = []
        self.angles = []
        self.start_id = None
        self.start_angle = None


    def init(self):
        self.origin = (self.position.x, self.position.y)

        self.stop_robot()
        self.move_joints_init()


    def loop(self):
        corners, ids, rejectedImgPoints = self.get_aruco(self.frame, do_draw=False)

        if self.start_id == None and len(ids) > 0:
            print("Definindo posição de origem...")
            self.start_id = ids[0]
            self.start_angle = self.get_orientation_angle()

        if len(self.ids) == 0:
            print("Escaneando por creepers amarelos ao redor...")
            see_start_id = True
            do_scan = True

            while do_scan:
                corners, ids, rejectedImgPoints = self.get_aruco(self.frame, do_draw=False)

                yellow_mask = cv2.inRange(self.hsv, (40/2, 100, 100), (70/2, 255, 255))
                cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, np.ones([5, 5]), yellow_mask)
                yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

                green_mask = cv2.inRange(self.hsv, (80/2, 100, 100), (140/2, 255, 255))
                cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, np.ones([5, 5]), green_mask)
                green_contours, _ = cv2.findContours(green_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

                # blue_mask = cv2.inRange(self.hsv, (170/2, 100, 100), (220/2, 255, 255))
                # cv2.morphologyEx(blue_mask, cv2.MORPH_OPENyellow_mask, np.ones([5, 5]), blue_mask)
                # blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

                if len(corners) != 0:
                    for corner, _id in zip(corners, ids):
                        aruco_x = corner[0][0][0]
                        
                        for contour in yellow_contours:
                            creeper_x, creeper_y, ceeper_w, creeper_h = cv2.boundingRect(contour)

                            if np.abs(creeper_x - aruco_x) < 20 and self.frame.shape[1] / 2 - aruco_x < 5:
                                if not _id in self.ids:
                                    angle = self.get_orientation_angle()

                                    print(f"Encontrado creeper de ID {_id}!")
                                    self.ids.append(_id)
                                    self.angles.append(angle)
                        
                        for contour in green_contours:
                            creeper_x, creeper_y, ceeper_w, creeper_h = cv2.boundingRect(contour)

                            if np.abs(creeper_x - aruco_x) < 20:
                                if _id == self.start_id and see_start_id == False:
                                    do_scan = False
                                    print("Escaneamento concluido!")

                if not self.has_aruco(ids, self.start_id) and np.abs(self.get_turn_offset(self.start_angle, self.get_orientation_angle())) > np.pi / 3:
                    see_start_id = False

                self.cmd.angular.z = .1

                self.update_cmd()
                rospy.sleep(.1)

            print("Alinhando com a posição de origem...")
            self.set_angle(self.start_angle)

        for angle, _id in zip(self.angles, self.ids):
            print(f"Alinhando com o creeper de ID {_id}...")
            self.set_angle(angle)

            print(f"Derrubando creeper de ID {_id}...")
            self.walk_straight(3)

            print("Voltando a posição de origem...")
            self.walk_to(self.origin)



if __name__ == "__main__":
    rospy.init_node("Q3")
    robot = Robot_Q3(True)

    robot.init()

    try:        
        while not robot.ctrl_c:
            robot.loop()
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")



#! /usr/bin/env python3
# -*- coding:utf-8 -*-

# Rodar com 
# roslaunch my_simulation rampa.launch


# from __future__ import print_function, division
# import rospy
# import numpy as np
# import numpy
# import tf
# import math
# import cv2
# import time
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Image, CompressedImage, LaserScan
# from cv_bridge import CvBridge, CvBridgeError
# from numpy import linalg
# from tf import transformations
# from tf import TransformerROS
# import tf2_ros
# from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped

# from nav_msgs.msg import Odometry
# from std_msgs.msg import Header


# import visao_module


# bridge = CvBridge()

# cv_image = None
# media = []
# centro = []

# area = 0.0 # Variavel com a area do maior contorno

# resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

# x = 0
# y = 0
# z = 0 
# id = 0


# # A função a seguir é chamada sempre que chega um novo frame
# def roda_todo_frame(imagem):
#     print("frame")
#     global cv_image
#     global media
#     global centro
#     global resultados

#     now = rospy.get_rostime()
#     imgtime = imagem.header.stamp
#     lag = now-imgtime # calcula o lag
#     delay = lag.nsecs

#     try:
#         temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
#         cv_image = temp_image.copy()
#         # ATENÇÃO: ao mostrar a imagem aqui, não podemos usar cv2.imshow() dentro do while principal!! 
#         cv2.imshow("cv_image", cv_image)
#         cv2.waitKey(1)
#     except CvBridgeError as e:
#         print('ex', e)
    
# if __name__=="__main__":
#     rospy.init_node("Q3")

#     topico_imagem = "/camera/image/compressed"

#     recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

#     print("Usando ", topico_imagem)

#     velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

#     try:
#         vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
        
#         while not rospy.is_shutdown():


#             velocidade_saida.publish(vel)
#             rospy.sleep(0.1)

#     except rospy.ROSInterruptException:
#         print("Ocorreu uma exceção com o rospy")



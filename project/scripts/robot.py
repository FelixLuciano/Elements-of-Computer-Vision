#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import numpy as np
import rospy
import cv2

from lib.control import Control
from lib.line_follower import LineFollower
from lib.creeper import Creeper
from lib.mobilenet import MobileNet


class Robot(Control, LineFollower, Creeper, MobileNet):
    def __init__(self, params, *arks, **kwargs):
        """Objeto mãe do robô.

        Herda State, Odometry, Laser, Control, Camera, LineFollower, Aruco e Creeper.
        """
        super(Robot, self).__init__(name="mini", *arks, **kwargs)

        self.creeper_color, self.creeper_id, self.base = params
        self.saw_aruco_100 = False
        self.saw_aruco_200_1 = False
        self.saw_aruco_200_2 = False
        self.vira_direita = False
        self.did_left_turn = False
        self.did_right_turn = False
        self.rounds = 0
        self.track_pos = None
        self.track_angle = None
        self.has_creeper = False
        self.base_pos = None
        self.base_angle = None
        self.no_creeper = False


    def init(self):
        """Rotina de inicialização do robô.
        """
        try:
            while not self.interrupt:
                self.loop()
                self.rate.sleep()
        except rospy.ROSInterruptException:
            pass


    def loop(self):
        """Rotinas de laço do robô.
        """
        detections = self.mobilenet_detect(self.frame)
        corners, ids, rejectedImgPoints = self.get_aruco(self.frame, do_draw=False)
        creepers = self.get_creepers(self.creeper_color)
        
        if self.creeper_id in creepers and not self.has_creeper:
            print("Creeper encontrado! Iniciando processo de captura...")
            self.track_pos = (self.position.x, self.position.y)
            self.track_angle = self.get_orientation_angle()
            linear_speed, angular_speed = self.get_orientation_offset_speeds(creepers[self.creeper_id], self.frame)

            while not self.get_laser_inrange(-30, 30, 0.23):
                creepers = self.get_creepers(self.creeper_color)
                if self.creeper_id in creepers:
                    linear_speed, angular_speed = self.get_orientation_offset_speeds(creepers[self.creeper_id], self.frame)
                self.vel.linear.x = linear_speed
                self.vel.angular.z = angular_speed
                self.update_vel()

            while np.abs(angular_speed) > 1E-1:
                creepers = self.get_creepers(self.creeper_color)
                if self.creeper_id in creepers:
                    linear_speed, angular_speed = self.get_orientation_offset_speeds(creepers[self.creeper_id], self.frame)
                self.vel.linear.x = 0.0
                self.vel.angular.z = angular_speed
                self.update_vel()
            
            self.stop_robot()
            rospy.sleep(0.5)
            print("Abre a garra")
            self.claw = -1.0
            self.update_joints()
            rospy.sleep(3.0)
            print("Estende o braço")
            self.arm = 0.0
            self.update_joints()
            rospy.sleep(3.0)
            print("Fecha garra")
            self.claw = 0.0
            self.update_joints()
            rospy.sleep(3.0)
            print("Levanta o braço")
            self.arm = 1.5
            self.update_joints()
            rospy.sleep(3.0)
            print("Creeper capturado com sucesso! Voltando a pista...")
            self.has_creeper = True
            self.walk_to(self.track_pos)
            self.set_angle(self.track_angle)
        
        for detection in detections:
            name, confidence, pos_1, pos_2 = detection
            if name == self.base:
                self.track_pos = (self.position.x, self.position.y)
                self.track_angle = self.get_orientation_angle()

                if self.base_pos == None and self.base_angle == None:
                    print("Base encontrada! Salvando localização para mais tarde...")
                    self.base_pos = (self.position.x, self.position.y)
                    detection_point = ((pos_1[1] + pos_2[1]) // 2, (pos_1[0] + pos_2[0]) // 2)
                    linear_speed, angular_speed = self.get_orientation_offset_speeds(detection_point, self.frame)

                    while np.abs(angular_speed) > 3E-1:
                        detections2 = self.mobilenet_detect(self.frame)
                        
                        for detection2 in detections2:
                            name, confidence, pos_12, pos_22 = detection2
                            if name == self.base:
                                detection_point = ((pos_12[1] + pos_22[1]) // 2, (pos_12[0] + pos_22[0]) // 2)
                                linear_speed, angular_speed = self.get_orientation_offset_speeds(detection_point, self.frame)
                                self.vel.linear.x = 0.0
                                self.vel.angular.z = angular_speed
                                self.update_vel()
                                break
                    self.base_angle = self.get_orientation_angle()
                    print("Localização salva!")
                    if not self.has_creeper:
                        self.set_angle(self.track_angle)
                    else:
                        print("Levando creeper para a base!")
                        self.walk_straight(10)
                        print("Abre a garra")
                        self.claw = -1.0
                        self.update_joints()
                        rospy.sleep(3.0)
                        print("abaixa o braço")
                        self.arm = -1.5
                        self.update_joints()
                        rospy.sleep(3.0)
                        print("Fecha garra")
                        self.claw = 0.0
                        self.update_joints()
                        rospy.sleep(3.0)
                        self.has_creeper = False
                        self.base_pos = True
                        self.base_angle = True
                        self.walk_to(self.track_pos)
                        self.set_angle(self.track_angle)
                break

        if self.rounds < 2:
            if not self.saw_aruco_100 and self.has_aruco(ids, 100):
                self.saw_aruco_100 = True
                print("Aruco 100 detectado!")

            if not self.saw_aruco_200_1 and self.has_aruco(ids, 200):
                self.saw_aruco_200_1 = True
                print("Aruco 200 detectado! A curva será para a esquerda.")

            if not self.saw_aruco_200_2 and self.saw_aruco_100 and self.has_aruco(ids, 200) :
                self.saw_aruco_200_2 = True
                print("Aruco 200 detectado! A curva será para a direita.")

            if not self.vira_direita and self.saw_aruco_100 and self.saw_aruco_200_2:
                self.vira_direita = True

            if self.get_coordinates_distances((self.start_pos.x, self.start_pos.y))[0] > 4.0:
                if not self.did_left_turn and not self.did_right_turn:
                    self.did_left_turn = True
                    print("O robô já está bem longe!")

                if self.did_left_turn and not self.did_right_turn and self.saw_aruco_200_2:
                    self.did_right_turn = True
                    print("O robô já está perto de terminar sua jornada!")

            lines_mask = self.get_lines_mask()
            near_lines = self.mask_near_lines(lines_mask)
            lines_mean, lines_std = self.get_mask_description(near_lines)

            if lines_std and lines_std[1] > self.frame.shape[1] / 7:
                mask_l, mask_c, mask_r = self.mask_far_lines(lines_mask)

                if self.vira_direita:
                    lines_mean, lines_std = self.get_mask_description(mask_r)
                elif not self.has_aruco(ids, 200):
                    lines_mean, lines_std = self.get_mask_description(mask_c)
                else:
                    lines_mean, lines_std = self.get_mask_description(mask_l)

            linear_speed, angular_speed = self.get_orientation_offset_speeds(lines_mean, self.frame)

            self.vel.linear.x = linear_speed
            self.vel.angular.z = angular_speed

            if self.get_coordinates_distances((self.start_pos.x, self.start_pos.y))[0] < 0.15:
                if self.rounds == 0 and self.did_left_turn:
                    self.rounds = 1
                    print("Primeira volta do trevo completa!")
                elif self.rounds == 1 and self.did_right_turn:
                    self.rounds = 2
                    print("Segunda volta do trevo completa!")
        
        else:
            print("Trevo concluido!")
            self.walk_to((self.start_pos.x, self.start_pos.y))

            if self.creeper_pos is not None and self.creeper_angle is not None and not self.no_creeper:
                print("Creeper ao resgate!")
                self.walk_to(self.creeper_pos)
                self.set_angle(self.creeper_angle)

                while not self.get_laser_inrange(-30, 30, 0.25):
                    creepers = self.get_creepers(self.creeper_color)
                    if self.creeper_id in creepers:
                        linear_speed, angular_speed = self.get_orientation_offset_speeds(creepers[self.creeper_id], self.frame)
                    self.vel.linear.x = linear_speed
                    self.vel.angular.z = angular_speed
                    self.update_vel()
                
                self.no_creeper = True

            # self.stop_robot()
            # print("Levanta o braço e abre a garra")
            # self.arm = 1.5
            # self.claw = -1.0
            # self.update_joints()
            # rospy.sleep(3.0)
            # print("Fecha garra")
            # self.claw = 0.0
            # self.update_joints()
            # rospy.sleep(3.0)

            # print("De volta ao começo!")
            # self.walk_to((self.start_pos.x, self.start_pos.y))

            # print("Levando creeper para a base!")
            # self.walk_to(self.base_pos)
            # self.set_angle(self.base_angle)
            # self.walk_straight(10)

            # print("Abre a garra")
            # self.claw = -1.0
            # self.update_joints()
            # rospy.sleep(3.0)
            # print("abaixa o braço")
            # self.arm = 1.5
            # self.update_joints()
            # rospy.sleep(3.0)
            # print("Fecha garra")
            # self.claw = 0.0
            # self.update_joints()
            # rospy.sleep(3.0)

            print("De volta ao começo!")
            self.walk_to((self.start_pos.x, self.start_pos.y))

            print("Fim")
            exit()

        self.update_vel()

        cv2.imshow("frame", self.frame)
        cv2.waitKey(1)

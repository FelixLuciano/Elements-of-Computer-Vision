#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import numpy as np
from nav_msgs.msg import Odometry as _Odometry
from tf import transformations


class Odometry(object):
    def __init__(self):
        """Rotina e o métodos de leitura da odometria do robô.
        """        
        super(Odometry, self).__init__()

        self.odometry_subscriber = rospy.Subscriber("/odom", _Odometry, self.odometry_callback)

        self.start_pos = None
        self.odometry_callback(_Odometry())


    def odometry_callback(self, data):
        """Callback do subscriber do node da odometria que atualiza os valores da posição e orientação.

        Args:
            data (Odometry): Dados da odometria fornecida pelo rospy.
        """
        self.position = data.pose.pose.position
        self.orientation = data.pose.pose.orientation

        if self.start_pos == None:
            self.start_pos = self.position


    def get_orientation_angle(self):
        """Obtém o valor em graus da orientação do robô de acordo com a odometria em relação ao plano xy.

        Returns:
            float: Ângulo em graus da orientação do robô sobre o plano xy.
        """
        orientation = (self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w)

        return transformations.euler_from_quaternion(orientation)[2]


    @staticmethod
    def get_turn_offset(start, end):
        """Obtém a diferença entre dois ângulos. Ou ainda, A menor distância angular entre duas orientações.

        Assumindo `start` < `end`.

        Args:
            start (float): Ângulo inicial (em radianos).
            end ([type]): Ângulo final (em radianos).

        Returns:
            float: Diferença entre os ângulos (em radianos).
        """
        offset = end - start

        if np.abs(offset) > np.pi:
            offset -= 2 * np.pi * np.sign(offset)

        return offset


    def get_coordinates_distances(self, coords):
        """Obtém as distâncias linear e angular que o robô está, de acordo com a odometria, de uma determinada coordenada.

        Args:
            coords (list): Coordenadas x e y, respectivamente, das coordenadas que deseja-se obter as distâncias

        Returns:
            float: Distância linear do robô (em metros).
            float: Distância angular do robô (em radianos).
        """
        pos = (self.position.x, self.position.y)
        delta_pos = (coords[0] - pos[0], coords[1] - pos[1])
        distance = np.hypot(*delta_pos)
        angle = np.arctan2(delta_pos[1], delta_pos[0])

        return distance, angle

#! /usr/bin/env python3
# -*- coding:utf-8 -*-

# import time

import numpy as np
import rospy

from sensor_msgs.msg import LaserScan


class Laser(object):
    def __init__(self):
        """Rotina e o métodos de leitura do sensor Lidar do robô.
        """
        super(Laser, self).__init__()

        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.scan_subscriber)
        self.laser = np.array([0] * 360)


    def scan_subscriber(self, scan):
        """Callback do subscriber do node do scaner que atualiza os valores do scan.

        Args:
            scan (LaserScan): Leitura do sensor fornecida pelo rospy.
        """
        self.laser = np.array(scan.ranges)


    def get_laser_view(self, start_angle, end_angle):
        """Obtém os valores das leituras do lidar em um intervalo de ângulo de visão.

        Sendo o intervalo em um intervalo de -180º e 180º. E 0º o angulo para frente do robô, crescendo em sentido anti-horário com ponto de referência de cima.

        Args:
            start_angle (int): Ângulo inicial de segmentação das leituras do lidar (em graus).
            end_angle (int): Ângulo final de segmentação das leituras do lidar (em graus).

        Returns:
            array: Valores das leituras do lidar no intervalo (em metros).
        """
        if start_angle == end_angle:
            return self.laser[start_angle:end_angle + 1]

        elif start_angle > end_angle:
            start_angle, end_angle = end_angle, start_angle

        inrange = [start_angle <= angle < end_angle for angle in range(0, 180)] + [start_angle <= angle <= end_angle for angle in range(-180, 0)]

        return self.laser[inrange]


    def get_laser_inrange(self, start_angle, end_angle, max_radius, min_radius=0.0):
        """Obtém se há alguma detecção dentro de um intervalo de valores para um inervalo de ângulo de visão.

        Args:
            start_angle (int): Ângulo inicial de segmentação das leituras do lidar (em graus).
            end_angle (int): Ângulo final de segmentação das leituras do lidar (em graus).
            max_radius (float): Raio máximo de detecção (em metros).
            min_radius (float, optional): Raio mínimo de detecção (em metros). Padrão é 0 m.

        Returns:
            bool: Se há alguma detecção para os intervalo definidos.
        """
        view = self.get_laser_view(start_angle, end_angle)

        inrange = [min_radius <= angle <= max_radius for angle in view]
        nulls = view != 0
        detections = np.bitwise_and(inrange, nulls)

        return any(detections)

#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import numpy as np
import cv2

from .camera import Camera


class LineFollower(Camera):
    def __init__(self):
        """Métodos para Controle do robô conforme linhas na câmera.

        Herda Camera.
        """
        super(LineFollower, self).__init__()


    def get_lines_mask(self):
        """Obtém a máscara com os segmentos mais próximos das faixas amarelas da pista onde o robô deve andar.

        Herda Camera.
        Returns:
            array: Máscaras das faixas amarelas da pista.
        """
        if self.is_simulation:
            lines = cv2.inRange(self.hsv.copy(), (40/2, 150, 150), (60/2, 255, 255))
        else:
            lines = cv2.inRange(self.hsv.copy(), (40/2, 64, 64), (65/2, 255, 255))

        kernel_erode = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, ksize=(5,5))
        lines = cv2.erode(lines, kernel_erode)

        kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, ksize=(3,3))
        lines = cv2.dilate(lines, kernel_dilate)

        return lines


    def get_mask_description(self, mask):
        """Obtém as medidas de descrição dos contornos de uma máscara.

        Args:
            mask (array): Máscara a obter as medidas de descrição

        Returns:
            list: Coordenadas y e x, respectivamente, da média dos centros de massa poderada pela área de cada de cada contorno.
            list: Desvio em y e x, respectivamente, padrão dos centros de massa dos contornos.
        """
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

        if len(contours) > 0:
            origin = self.get_frame_center_pos(mask)
            y_list = []
            x_list = []
            area_list = []

            for contour in contours:
                m = cv2.moments(contour)
                area = cv2.contourArea(contour)
                
                try:
                    cy = m["m01"] / m["m00"]
                except ZeroDivisionError:
                    cy = origin[1]

                try:
                    cx = m["m10"] / m["m00"]
                except ZeroDivisionError:
                    cx = origin[0]
                    
                y_list.append(cy)
                x_list.append(cx)
                area_list.append(area)

            y_mean = int(np.average(y_list, weights=area_list))
            x_mean = int(np.average(x_list, weights=area_list))
            
            y_var = int(np.std(y_list))
            x_var = int(np.std(x_list))

            return (y_mean, x_mean), (y_var, x_var)
        else:
            return None, None


    def get_orientation_offset_speeds(self, point, frame):
        """Obtém os valores das velocidades linear e angular que orientarão o robô em direção a um ponto sobre um dado frame.

        Args:
            point (list): Lista com as coordenadas y e x, respecitvamente, do ponto sobre a tela o qual deve-se obter as velocidades de orientação.
            frame (array): Frame o qual o ponto está projetado.

        Returns:
            float: velocidade linear
            float: velocidade angular
        """
        origin = self.get_frame_center_pos(frame)

        if point is not None:
            offset_x = point[1] - origin[1]
            linear_speed = np.max(.26 * (1 - np.abs(offset_x) / origin[1]), 0)

            angular_speed = -.8 * offset_x / origin[1]
        
        else:
            linear_speed = 0
            angular_speed = 1.82

        return linear_speed, angular_speed


    def mask_near_lines(self, frame):
        """Obtém as máscaras com os segmentos mais distanted das faixas amarelas da pista onde o robô pode andar.

        Returns:
            list: Lista de máscaras das faixas amarelas da pista.
        """
        return self.mask_frame(frame, (0.75, 1.00), (0.15, 0.85))


    def mask_far_lines(self, frame):
        """Obtém as máscaras com os segmentos mais distanted das faixas amarelas da pista onde o robô pode andar.

        Returns:
            list: Lista de máscaras das faixas amarelas da pista.
        """
        mask_l = self.mask_frame(frame, (0.55, 1.00), (0.00, 0.45))
        mask_c = self.mask_frame(frame, (0.55, 1.00), (0.40, 0.70))
        mask_r = self.mask_frame(frame, (0.55, 1.00), (0.65, 1.00))

        return mask_l, mask_c, mask_r

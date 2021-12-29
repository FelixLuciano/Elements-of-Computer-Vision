#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import numpy as np
import cv2

from .aruco import Aruco


CREEPERS_RANGES = {
    "blue": ((160/2, 75, 75), (210/2, 255, 255)),
    "orange": ((0, 100, 100), (10, 255, 255)),
    "green": ((58, 50, 50), (68, 255, 255))
}


class Creeper(Aruco):
    def __init__(self):
        """Métodos para detecção de Creepers utilizando o Aruco do OpenCV

        Herda Aruco.
        """
        super(Creeper, self).__init__()


    def get_creepers(self, color):
        """Encontra o id ea posição na tela de Creepers no frame da câmera do robô de acordo com uma dada cor.

        Args:
            color (string): Nome da cor dos creepers que devem ser detectados.

        Returns:
            list: Dicionário com o id e a posição no frame dos creepers encontrados para a cor selecionada.
        """
        corners, ids, rejectedCorners = self.get_aruco(self.frame, do_draw=False)

        mask = cv2.inRange(self.hsv.copy(), CREEPERS_RANGES[color][0], CREEPERS_RANGES[color][1])
        cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones([5, 5]), mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        creepers = {}

        if len(corners) != 0:
            for corner, _id in zip(corners, ids):
                _id = int(_id)
                aruco_cx = int(sum(c[0] for c in corner[0]) / len(corner[0]))
                aruco_cy = int(sum(c[1] for c in corner[0]) / len(corner[0]))
                
                for contour in contours:
                    x, y , width, height = cv2.boundingRect(contour)

                    if x <= aruco_cx <= x + width and np.abs((y + height/2) - aruco_cy) <= height * 1.2:
                        creepers[_id] = (y, x)

        return creepers

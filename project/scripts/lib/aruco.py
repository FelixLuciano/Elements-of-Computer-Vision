#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import cv2


class Aruco(object):
    def __init__(self):
        """Métodos para detecção de códigos aruco utilizando o Aruco do OpenCV.
        """
        super(Aruco, self).__init__()


    @staticmethod
    def get_aruco(frame, do_draw=True):
        """
            Utiliza biblioteca Arudo do OpenCV para detectar códigos em um determinado frame.

        Args:
            frame (Array): Frame a ser escaneado por códigos.
            do_draw (bool, optional): Condição se deve desenhar os cantos dos códigos no frame. Padrão é True.

        Returns:
            array: Coordenadas dos cantos de cada aruco detectado.
            list: IDs detectados.
            array: Coordenadas dos cantos de cada detecção rejeitada.
        """        
        gray = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2GRAY)
        arucos = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        corners, ids, rejectedCorners = cv2.aruco.detectMarkers(gray, arucos)

        ids_flat = None
        if ids is not None:
            ids_flat = [_id for row in ids for _id in row]

        if do_draw:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        return corners, ids_flat, rejectedCorners


    @staticmethod
    def has_aruco(id_map, *ids):
        """Verifica se o código de algum aruco foi detectado em uma lista de IDs.

        Args:
            id_map (list): Lista de códigos detectados.
            *ids (int): Código(s) de interesse.

        Returns:
            [bool]: Se (todos) o(s) código(s) de interesse está(ão) incluso na lista
        """        
        return id_map is not None and all([_id in id_map for _id in ids])

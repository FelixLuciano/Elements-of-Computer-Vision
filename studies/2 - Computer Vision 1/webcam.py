#!/usr/bin/python
# -*- coding: utf-8 -*-

# Este NÃO é um programa ROS

import cv2
import os,sys, os.path
import numpy as np
import fotogrametria

# ->>> !!!! FECHE A JANELA COM A TECLA ESC !!!! <<<<-

def calcular_angulo_e_distancia_na_image_da_webcam(img, f):
    """Não mude ou renomeie esta função
        ->>> !!!! FECHE A JANELA COM A TECLA ESC !!!! <<<<-
        deve receber a imagem da camera e retornar uma imagems com os contornos desenhados e os valores da distancia e o angulo.
    """
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    segmentado_magenta = fotogrametria.segmenta_circulo_magenta(img_hsv)
    contorno_magenta = fotogrametria.encontrar_maior_contorno(segmentado_magenta)
    centro_magenta = fotogrametria.encontrar_centro_contorno(contorno_magenta)

    segmentado_ciano = fotogrametria.segmenta_circulo_ciano(img_hsv)
    contorno_ciano = fotogrametria.encontrar_maior_contorno(segmentado_ciano)
    centro_ciano = fotogrametria.encontrar_centro_contorno(contorno_ciano)

    img_contornos = img_hsv.copy()
    cv2.drawContours(img_contornos, [contorno_magenta], -1, [255, 0, 0], 3)
    cv2.drawContours(img_contornos, [contorno_ciano], -1, [0, 0, 255], 3)

    D = fotogrametria.calcular_h(centro_ciano, centro_magenta)
    angulo = fotogrametria.calcular_angulo_com_horizontal_da_imagem(centro_ciano, centro_magenta)

    return img, D, angulo

def desenhar_na_image_da_webcam(img, distancia, angulo):
    """Não mude ou renomeie esta função
        ->>> !!!! FECHE A JANELA COM A TECLA ESC !!!! <<<<-
        deve receber a imagem da camera e retornar uma imagems com os contornos desenhados e a distancia e o angulo escrito em um canto da imagem.
    """

    return img

if __name__ == "__main__":
    cv2.namedWindow("preview")
    vc = cv2.VideoCapture(0)

    ## -> Mude o Foco <- ##
    f = fotogrametria.encontrar_foco(1, 1, 1)

    if vc.isOpened(): # try to get the first frame
        rval, frame = vc.read()
    else:
        rval = False

    while rval:
        img, distancia, angulo = calcular_angulo_e_distancia_na_image_da_webcam(frame, f)
        img = desenhar_na_image_da_webcam(img, distancia, angulo)
        cv2.imshow("preview", img)
        rval, frame = vc.read()
        key = cv2.waitKey(20)
        if key == 27: # exit on ESC
            break

    cv2.destroyWindow("preview")
    vc.release()
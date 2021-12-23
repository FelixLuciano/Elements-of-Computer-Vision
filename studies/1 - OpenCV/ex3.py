#!/usr/bin/python
# -*- coding: utf-8 -*-

# Este NÃO é um programa ROS

from __future__ import print_function, division 

import cv2
import os,sys, os.path
import numpy as np

print("Rodando Python versão ", sys.version)
print("OpenCV versão: ", cv2.__version__)
print("Diretório de trabalho: ", os.getcwd())


def crosshair(img, point, size, color):    
    """ Desenha um crosshair centrado no point.
        point deve ser uma tupla (x,y)
        color é uma tupla R,G,B uint8
    """
    x,y = point
    cv2.line(img,(x - size,y),(x + size,y),color,5)
    cv2.line(img,(x,y - size),(x, y + size),color,5)

def recorta_leopardo(bgr): 
    """Não mude ou renomeie esta função
        deve receber uma imagem bgr e devolver uma nova imagem com tudo em preto e o os pixels da caixa em granco
    """
    res = bgr.copy()

    pos_start = (0, 0)
    for y, row in enumerate(res):
        for x, pixel in enumerate(row):
            if np.array_equal(pixel, (255, 0, 0)):
                pos_start = (x, y)
                break

    pos_end = (0, 0)
    for y, row in enumerate(res):
        for x, pixel in enumerate(row):
            if np.array_equal(pixel, (0, 0, 255)):
                pos_end = (x, y)
                break

    res = res[pos_start[1]:pos_end[1], pos_start[0]:pos_end[0]]

    return res


if __name__ == "__main__":
    img = cv2.imread("snowleopard_red_blue_600_400.png")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Faz o processamento
    saida = recorta_leopardo(img)
    cv2.imwrite("ex3_recorte_leopardo.png", saida)


    # NOTE que a OpenCV terminal trabalha com BGR
    cv2.imshow('entrada', img)

    cv2.imshow('saida', saida)

    cv2.waitKey()
    cv2.destroyAllWindows()


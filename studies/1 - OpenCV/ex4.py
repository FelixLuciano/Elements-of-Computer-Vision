#!/usr/bin/python
# -*- coding: utf-8 -*-

# Este NÃO é um programa ROS

from __future__ import print_function, division 

import cv2
import os,sys, os.path
import numpy as np
from numpy.lib.function_base import delete

print("Rodando Python versão ", sys.version)
print("OpenCV versão: ", cv2.__version__)
print("Diretório de trabalho: ", os.getcwd())


def crosshair(img, point, size, color):    
    """ Desenha um crosshair centrado no point.
        point deve ser uma tupla (x,y)
        color é uma tupla R,G,B uint8
    """
    x,y = point
    cv2.line(img,(x - size,y),(x + size,y),color,1)
    cv2.line(img,(x,y - size),(x, y + size),color,1)

def antartida(bgr): 
    """Não mude ou renomeie esta função
        deve receber uma imagem bgr e devolver o recorte da imagem da antartida
    """
    res = bgr.copy()

    pos_start = [-1, -1]
    pos_end = [-1, -1]

    for y, row in enumerate(res):
        for x, pixel in enumerate(row):
            if not np.array_equal(pixel, (0, 0, 0)):
                pos_start = [x, y]
                break

        if pos_start[1] >= 0:
            break

    for x in range(pos_start[0], res.shape[1]):
        pixel = res[pos_start[1], x]
        
        if np.array_equal(pixel, (0, 0, 0)):
            pos_end[0] = x - 1
            break

    for y in range(pos_start[1], res.shape[0]):
        pixel = res[y, pos_end[0]]
        
        if np.array_equal(pixel, (0, 0, 0)):
            pos_end[1] = y - 1
            break

    res = res[pos_start[1]:pos_end[1], pos_start[0]:pos_end[0]]

    return res

def canada(bgr): 
    """Não mude ou renomeie esta função
        deve receber uma imagem bgr e devolver o recorte da imagem do canadá
    """
    res = bgr.copy()

    pos_start = [-1, -1]
    pos_end = [-1, -1]

    for y in range(res.shape[0] - 1, 0, -1):
        for x in range(res.shape[1] - 1, 0, -1):
            pixel = res[y, x]
            
            if not np.array_equal(pixel, (0, 0, 0)):
                pos_end = [x, y]
                break

        if pos_end[1] >= 0:
            break

    for x in range(pos_end[0], 0, -1):
        pixel = res[pos_end[1], x]
        
        if np.array_equal(pixel, (0, 0, 0)):
            pos_start[0] = x + 1
            break

    for y in range(pos_end[1], 0, -1):
        pixel = res[y, pos_start[0]]
        
        if np.array_equal(pixel, (0, 0, 0)):
            pos_start[1] = y + 1
            break

    res = res[pos_start[1]:pos_end[1], pos_start[0]:pos_end[0]]

    return res


if __name__ == "__main__":
    img = cv2.imread("ant_canada_250_160.png")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Faz o processamento
    antartida = antartida(img)
    canada = canada(img)
    cv2.imwrite( "ex4_antartida_resp.png", antartida)
    cv2.imwrite("ex5_canada_resp.png", canada)    


    # NOTE que a OpenCV terminal trabalha com BGR
    cv2.imshow('entrada', img)

    cv2.imshow('antartida', antartida)
    cv2.imshow('canada', canada)

    cv2.waitKey()
    cv2.destroyAllWindows()



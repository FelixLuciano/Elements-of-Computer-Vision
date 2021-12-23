#!/usr/bin/python
# -*- coding: utf-8 -*-

# Este NÃO é um programa ROS

from __future__ import print_function, division 

import cv2
import os,sys, os.path
import numpy as np
import math

from numpy.lib.function_base import _SIGNATURE

def encontrar_foco(D,H,h):
    """Não mude ou renomeie esta função
    Entradas:
       D - distancia real da câmera até o objeto (papel)
       H - a distancia real entre os circulos (no papel)
       h - a distancia na imagem entre os circulos
    Saída:
       f - a distância focal da câmera
    """
    f = (h/H)*D

    return f

def segmenta_circulo_ciano(hsv): 
    """Não mude ou renomeie esta função
    Entrada:
        hsv - imagem em hsv
    Saída:
        mask - imagem em grayscale com tudo em preto e os pixels do circulos ciano em branco
    """
    matiz = hsv[:,:,0]
    saturation = hsv[:,:,1]

    mask_ciano = np.bitwise_and(matiz >= 90, matiz <= 95)
    mask_saturation = saturation >= 180
    
    mask = np.zeros_like(matiz)
    mask[np.bitwise_and(mask_ciano, mask_saturation)] = 255

    return mask

def segmenta_circulo_magenta(hsv):
    """Não mude ou renomeie esta função
    Entrada:
        hsv - imagem em hsv
    Saída:
        mask - imagem em grayscale com tudo em preto e os pixels do circulos magenta em branco
    """
    matiz = hsv[:,:,0]
    saturation = hsv[:,:,1]

    mask_magenta = np.bitwise_and(matiz >= 148, matiz <= 153)
    mask_saturation = saturation >= 50
    
    mask = np.zeros_like(matiz)
    mask[np.bitwise_and(mask_magenta, mask_saturation)] = 255
    
    return mask

def encontrar_maior_contorno(segmentado):
    """Não mude ou renomeie esta função
    Entrada:
        segmentado - imagem em preto e branco
    Saída:
        contorno - maior contorno obtido (APENAS este contorno)
    """
    contorno, arvore = cv2.findContours(segmentado.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    maior = None
    maior_area = 0

    for c in contorno:
        area = cv2.contourArea(c)
        if area > maior_area:
            maior_area = area
            maior = c

    return maior

def encontrar_centro_contorno(contorno):
    """Não mude ou renomeie esta função
    Entrada:
        contorno: um contorno (não o array deles)
    Saída:
        (Xcentro, Ycentro) - uma tuple com o centro do contorno (no formato 'int')!!! 
    """
    M = cv2.moments(contorno)

    Xcentro = int(M["m10"] / M["m00"])
    Ycentro = int(M["m01"] / M["m00"])
    
    return (Xcentro, Ycentro)

def calcular_h(centro_ciano, centro_magenta):
    """Não mude ou renomeie esta função
    Entradas:
        centro_ciano - ponto no formato (X,Y)
        centro_magenta - ponto no formato (X,Y)
    Saída:
        distancia - a distancia Euclidiana entre os pontos de entrada 
    """
    distancia_x = centro_ciano[0] - centro_magenta[0]
    distancia_y = centro_ciano[1] - centro_magenta[1]    
    distancia = np.hypot(distancia_x, distancia_y)

    return distancia

def encontrar_distancia(f,H,h):
    """Não mude ou renomeie esta função
    Entrada:
        f - a distância focal da câmera
        H - A distância real entre os pontos no papel
        h - a distância entre os pontos na imagem
    Saída:
        D - a distância do papel até câmera
    """
    D = (H/h) * f

    return D

def calcular_distancia_entre_circulos(img):
    """Não mude ou renomeie esta função
    Deve utilizar as funções acima para calcular a distancia entre os circulos a partir da imagem BGR
    Entradas:
        img - uma imagem no formato BGR
    Saídas:
        h - a distância entre os os circulos na imagem
        centro ciano - o centro do círculo ciano no formato (X,Y)
        centro_magenta - o centro do círculo magenta no formato (X,Y)
        img_contornos - a imagem com os contornos desenhados
    """
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    segmentado_magenta = segmenta_circulo_magenta(img_hsv)
    contorno_magenta = encontrar_maior_contorno(segmentado_magenta)
    centro_magenta = encontrar_centro_contorno(contorno_magenta)

    segmentado_ciano = segmenta_circulo_ciano(img_hsv)
    contorno_ciano = encontrar_maior_contorno(segmentado_ciano)
    centro_ciano = encontrar_centro_contorno(contorno_ciano)
    
    h = calcular_h(centro_magenta, centro_ciano)

    img_contornos = img_hsv.copy()
    cv2.drawContours(img_contornos, [contorno_magenta], -1, [255, 0, 0], 3)
    cv2.drawContours(img_contornos, [contorno_ciano], -1, [0, 0, 255], 3)

    return h, centro_ciano, centro_magenta, img_contornos

def calcular_angulo_com_horizontal_da_imagem(centro_ciano, centro_magenta):
    """Não mude ou renomeie esta função
        Deve calcular o angulo, em graus, entre o vetor formato com os centros do circulos e a horizontal.
    Entradas:
        centro_ciano - centro do círculo ciano no formato (X,Y)
        centro_magenta - centro do círculo magenta no formato (X,Y)
    Saídas:
        angulo - o ângulo entre os pontos em graus
    """
    distancia_x = centro_ciano[0] - centro_magenta[0]
    distancia_y = centro_magenta[1] - centro_ciano[1]

    angulo_radiano = np.abs(np.arctan2(distancia_y, distancia_x))
    angulo = np.rad2deg(angulo_radiano)
 
    return angulo

#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import math

from sklearn.linear_model import LinearRegression
from biblioteca import crosshair

def segmenta_linha_branca(bgr):
    """Não mude ou renomeie esta função
        deve receber uma imagem e segmentar as faixas brancas
    """
    mask = cv2.inRange(bgr, (240, 240, 240), (255, 255, 255))
    kernel = np.ones((5, 5), np.uint8)
    morpho = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    return morpho

def estimar_linha_nas_faixas(img, mask):
    """Não mude ou renomeie esta função
        deve receber uma imagem preta e branca e retorna dois pontos que formen APENAS uma linha em cada faixa. Desenhe cada uma dessas linhas na iamgem.
         formato: [[(x1,y1),(x2,y2)], [(x1,y1),(x2,y2)]]
    """
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

    lines = []
    for contour in contours:
        x_array = contour[:, :, 0]
        y_array = contour[:, :, 1]
        y_array = y_array.reshape(-1, 1)
        reg = LinearRegression().fit(y_array, x_array)
        lm = reg.coef_
        o = reg.intercept_
        y1 = 0
        x1 = int(lm * y1 + o)
        y2 = img.shape[0]
        x2 = int(lm * y2 + o)
        point_1 = (x1, y1)
        point_2 = (x2, y2)

        cv2.line(img, point_1, point_2, (255, 0, 255), 2)
        lines.append((point_1, point_2))
   
    return lines

def calcular_equacao_das_retas(linhas):
    """Não mude ou renomeie esta função
        deve receber dois pontos que estejam em cada uma das faixas e retornar a equacao das duas retas. Onde y = h + m * x. Formato: [(m1,h1), (m2,h2)]
    """
    equations = []
    for linha in linhas:
        point_1, point_2 = linha
        x1, y1 = point_1
        x2, y2 = point_2
        m = (x2 - x1) / (y2 - y1)

        equations.append((m, x1))
    
    return equations

def calcular_ponto_de_fuga(img, equacoes):
    """Não mude ou renomeie esta função
        deve receber duas equacoes de retas e retornar o ponto de encontro entre elas. Desenhe esse ponto na imagem.
    """
    equation_1, equation_2 = equacoes
    m1, h1 = equation_1
    m2, h2 = equation_2
    y = int((h2 - h1) / (m1 - m2))
    x = int(h1 + y * m1)

    crosshair(img, (x, y), 5, (255, 0, 0))

    return (x, y)

        

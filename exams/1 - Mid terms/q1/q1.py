#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Este NÃO é um programa ROS

from __future__ import print_function, division 

import cv2 as cv
import os,sys, os.path
import numpy as np
from sklearn.linear_model import LinearRegression


def segmenta_nave (img):
    mask_ship = cv.inRange(hsv, (310/2, 0, 100), (330/2, 255, 255))

    kernel_dilate = cv.getStructuringElement(cv.MORPH_ELLIPSE, ksize=(3,3))
    mask_ship = cv.dilate(mask_ship, kernel_dilate)

    kernel_erode = cv.getStructuringElement(cv.MORPH_ELLIPSE, ksize=(5,5))
    mask_ship = cv.erode(mask_ship, kernel_erode)
    
    return mask_ship


def segmenta_feiches (img):
    mask_rays = cv.inRange(hsv, (165/2, 200, 100), (185/2, 255, 255))
    
    return mask_rays


def calcula_ponto_de_influencia (img, mask):
    lines = []
    contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

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
        lines.append((point_1, point_2))
    
    equations = []
    for line in lines:
        point_1, point_2 = line
        x1, y1 = point_1
        x2, y2 = point_2
        m = (x2 - x1) / (y2 - y1)

        equations.append((m, x1))
    
    (m1, h1) = equations[0]
    (m2, h2) = equations[1]

    try:
        y = int((h2 - h1) / (m1 - m2))
    except ZeroDivisionError:
        y = mask.shape[0]

    x = int(h1 + y * m1)

    cv.circle(img, (x, y), 5, (0, 0, 255), 5)

    return (x, y)


def obtem_ponto_dentro_de_mascara(img, mask, point):
    py, px = point

    if py >= 0 and py < mask.shape[0] and px >= 0 and px < mask.shape[1]:
        pixel = mask[py, px]

        if pixel > 0:
            img[mask > 0] = (255, 255, 255)

            cv.putText(img, "ACERTOU", (10, 40), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
            print("ACERTOU")

            return True

    return False

def obtem_distancia_dos_contornos (img, mask_ship, mask_rays):
    ship_pixels = np.where(mask_ship > 0)
    ship_bottom = max(ship_pixels[0])

    rays_pixels = np.where(mask_rays > 0)
    rays_top = min(rays_pixels[0])

    mask = np.bitwise_or(mask_ship, mask_rays)

    if rays_top - ship_bottom <= 0:
        img[mask == 0] = (0, 255, 255)

        return True

    return False

def obtem_fim_de_jogo (img, mask_ship, mask_rays):
    mask = np.bitwise_or(mask_ship, mask_rays)
    contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    return len(contours) < 3


print("Rodando Python versão ", sys.version)
print("OpenCV versão: ", cv.__version__)
print("Diretório de trabalho: ", os.getcwd())

# Arquivos necessários
video = "lasercannon.mp4"


if __name__ == "__main__":

    # Inicializa a aquisição da webcam
    cap = cv.VideoCapture(video)
    is_game_over = False

    print("Se a janela com a imagem não aparecer em primeiro plano dê Alt-Tab")

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if ret == False:
            #print("Codigo de retorno FALSO - problema para capturar o frame")
            #cap.set(cv.CAP_PROP_POS_FRAMES, 0)
            break

        # Our operations on the frame come here
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        mask_ship = segmenta_nave(hsv)
        mask_rays = segmenta_feiches(hsv)
        point = calcula_ponto_de_influencia(frame, mask_rays)
        hit = obtem_ponto_dentro_de_mascara(frame, mask_ship, point)
        check_danger = obtem_distancia_dos_contornos(frame, mask_ship, mask_rays)
        check_game_over = obtem_fim_de_jogo(frame, mask_ship, mask_rays)
        is_game_over = is_game_over or check_game_over
        mask = np.bitwise_or(mask_ship, mask_rays)

        if is_game_over:
            cv.putText(frame, "GAME OVER", (35, 325), cv.FONT_HERSHEY_SIMPLEX, 4, (255, 0, 255), 5)
            print("GAME OVER")
            
        elif check_danger:
            print("CUIDADO")

        # NOTE que em testes a OpenCV 4.0 requereu frames em BGR para o cv.imshow
        cv.imshow('imagem', frame)

        # Pressione 'q' para interromper o video
        if cv.waitKey(1000//30) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv.destroyAllWindows()


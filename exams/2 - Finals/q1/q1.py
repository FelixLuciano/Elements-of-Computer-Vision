#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Este NÃO é um programa ROS

from __future__ import print_function, division 

import cv2
import os,sys, os.path
import numpy as np

print("Rodando Python versão ", sys.version)
print("OpenCV versão: ", cv2.__version__)
print("Diretório de trabalho: ", os.getcwd())

# Arquivos necessários
video = "laserdefense.mp4"


if __name__ == "__main__":

    # Inicializa a aquisição da webcam
    cap = cv2.VideoCapture(video)

    print("Se a janela com a imagem não aparecer em primeiro plano dê Alt-Tab")

    atingida = False
    contagem = 0

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if ret == False:
            #print("Codigo de retorno FALSO - problema para capturar o frame")
            #cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            break

        # Our operations on the frame come here
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blank = np.zeros_like(frame)

        nave_mask = cv2.inRange(hsv, (0, 100, 100), (50, 255, 255))
        cv2.morphologyEx(nave_mask, cv2.MORPH_OPEN, np.ones([5, 5]), nave_mask)
        nave_contours, _ = cv2.findContours(nave_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        nave_x, nave_y, nave_w, nave_h = cv2.boundingRect(nave_contours[0])

        tiros_mask = cv2.inRange(hsv, (50, 100, 100), (255, 255, 255))
        tiros_contours, _ = cv2.findContours(tiros_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

        collide = False

        for contour in tiros_contours:
            x, y, w, h = cv2.boundingRect(contour)

            if x >= nave_x and x <= nave_x + nave_w:
                if y >= nave_y and y <= nave_y + nave_h:
                    collide = True

        if atingida == False and collide == True:
            contagem += 1
            atingida = True
        elif atingida == True and collide == False:
            atingida = False

        if atingida == True:
            frame[nave_mask != 0] = (255, 255, 255)
        elif contagem >= 10:
            frame[nave_mask != 0] = (128, 128, 128)

        cv2.putText(frame, str(contagem), (frame.shape[1] - 100, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 4, cv2.LINE_AA)


        # NOTE que em testes a OpenCV 4.0 requereu frames em BGR para o cv2.imshow
        cv2.imshow('imagem', frame)

        # Pressione 'q' para interromper o video
        if cv2.waitKey(1000//30) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


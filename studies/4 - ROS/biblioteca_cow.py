#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import math
import os

def load_mobilenet():
    """Não mude ou renomeie esta função
        Carrega o modelo e os parametros da MobileNet. 
        Retorna a rede carregada.
    """
    net = cv2.dnn.readNetFromCaffe("mobilenet_detection/MobileNetSSD_deploy.prototxt.txt", "mobilenet_detection/MobileNetSSD_deploy.caffemodel")

    return net


def detect(net, frame, CONFIDENCE, COLORS, CLASSES):
    """
        Recebe:
            net - a rede carregada
            frame - uma imagem colorida BGR
            CONFIDENCE - o grau de confiabilidade mínima da detecção
            COLORS - as cores atribídas a cada classe
            CLASSES - o array de classes
        Devolve: 
            img - a imagem com os objetos encontrados
            resultados - os resultados da detecção
    """
    image = frame.copy()
    (h, w) = image.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)

    # pass the blob through the network and obtain the detections and
    # predictions
    print("[INFO] computing object detections...")
    net.setInput(blob)
    detections = net.forward()

    results = []

    # loop over the detections
    for i in np.arange(0, detections.shape[2]):
        # extract the confidence (i.e., probability) associated with the
        # prediction
        confidence = detections[0, 0, i, 2]

        # filter out weak detections by ensuring the `confidence` is
        # greater than the minimum confidence

        if confidence > CONFIDENCE:
            # extract the index of the class label from the `detections`,
            # then compute the (x, y)-coordinates of the bounding box for
            # the object
            idx = int(detections[0, 0, i, 1])
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")

            # display the prediction
            label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
            print("[INFO] {}".format(label))

            cv2.rectangle(image, (startX, startY), (endX, endY), COLORS[idx], 2)

            y = startY - 15 if startY - 15 > 15 else startY + 15

            cv2.putText(image, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

            results.append((CLASSES[idx], confidence*100, (startX, startY),(endX, endY)))

    return image, results

def separar_caixa_entre_animais(img, resultados):
    """Não mude ou renomeie esta função
        recebe o resultados da MobileNet e retorna dicionario com duas chaves, 'vaca' e 'lobo'.
        Na chave 'vaca' tem uma lista de cada caixa que existe uma vaca, no formato: [ [min_X, min_Y, max_X, max_Y] , [min_X, min_Y, max_X, max_Y] , ...]. Desenhe um retângulo azul em volta de cada vaca
        Na chave 'lobo' tem uma lista de uma unica caixa que engloba todos os lobos da imagem, no formato: [min_X, min_Y, max_X, max_Y]. Desenhe um retângulo vermelho em volta dos lobos

    """
    img = img.copy()

    animais = {}
    animais['vaca'] = []
    animais['lobo'] = [0, 0, 0, 0]
    wolves = []

    for detection in resultados:
        (animal, confidence, start, end) = detection

        if animal == "cow":
            animais["vaca"].append([*start, *end])
            cv2.rectangle(img, start, end, (255, 0, 0), 2)

        elif animal == "horse":
            wolves.append([*start, *end])
            cv2.rectangle(img, start, end, (0, 0, 255), 2)

    animais['lobo'][0] = min([wolf[0] for wolf in wolves])
    animais['lobo'][1] = min([wolf[1] for wolf in wolves])
    animais['lobo'][2] = max([wolf[2] for wolf in wolves])
    animais['lobo'][3] = max([wolf[3] for wolf in wolves])

    return img, animais

def calcula_iou(boxA, boxB):
    """Não mude ou renomeie esta função
        Calcula o valor do "Intersection over Union" para saber se as caixa se encontram
    """
    x_start = max(boxA[0], boxB[0])
    y_start = max(boxA[1], boxB[1])
    x_end   = min(boxA[2], boxB[2])
    y_end   = min(boxA[3], boxB[3])

    inter_Area = (x_end - x_start) * (y_end - y_start)
    boxA_Area  = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    boxB_Area  = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])

    iou = inter_Area / (boxA_Area + boxB_Area - inter_Area)

    return iou

def checar_perigo(image, animais):
    """Não mude ou renomeie esta função
        Recebe as coordenadas das caixas, se a caixa de uma vaca tem intersecção com as do lobo, ela esta em perigo.
        Se estiver em perigo, deve escrever na imagem com a cor vermlha, se não, escreva com a cor azul.
        *Importante*: nesta função, não faça cópia da imagem de entrada!!
        
        Repita para cada vaca na imagem.
    """
    wolves = animais["lobo"]

    cv2.rectangle(image, (wolves[0], wolves[1]), (wolves[2], wolves[3]), (255, 0, 255), 2)

    for cow in animais["vaca"]:
        iou = calcula_iou(cow, wolves)

        if iou > 0:
            cv2.putText(image, "Perigo", (cow[0], cow[3]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        else:
            cv2.putText(image, "Nao perigo", (cow[0], cow[3]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    return

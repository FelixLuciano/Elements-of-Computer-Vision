#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import cv2
import numpy as np


CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
           "sofa", "train", "tvmonitor"]
CONFIDENCE = 0.7
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

class MobileNet(object):
    def __init__(self):
        """To do.
        """
        super(MobileNet, self).__init__()

        self.mobilenet = cv2.dnn.readNetFromCaffe("mobilenet_detection/MobileNetSSD_deploy.prototxt.txt", "mobilenet_detection/MobileNetSSD_deploy.caffemodel")


    def mobilenet_detect(self, frame):
        """
            Recebe:
                frame - uma imagem colorida BGR
            Devolve: 
                resultados - os resultados da detecção
        """
        image = frame.copy()
        (h, w) = image.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)
        
        self.mobilenet.setInput(blob)
        detections = self.mobilenet.forward()

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

                cv2.rectangle(image, (startX, startY), (endX, endY), COLORS[idx], 2)

                y = startY - 15 if startY - 15 > 15 else startY + 15

                cv2.putText(image, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

                results.append((CLASSES[idx], confidence*100, (startX, startY),(endX, endY)))

        return results

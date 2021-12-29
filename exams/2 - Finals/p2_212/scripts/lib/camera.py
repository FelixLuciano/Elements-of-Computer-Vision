#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import CompressedImage

class Camera(object):
    def __init__(self):
        """Rotina e o métodos de leitura da câmera do robô.
        """        
        super(Camera, self).__init__()

        self.bridge = CvBridge()
        self.camera_publisher = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.camera_callback, queue_size=4, buff_size = 2**24)
        self.frame = CompressedImage()

        rospy.on_shutdown(lambda: cv2.destroyAllWindows())


    def camera_callback(self, image):
        """Callback do subscriber do node da câmera que atualiza os valores do frame (BRG) e do hsv (HSV).

        Args:
            image (CompressedImage): Imagem da câmera fornecida pelo rospy.
        """        
        try:
            self.frame = self.bridge.compressed_imgmsg_to_cv2(image, "bgr8")

            if not self.is_simulation:
                self.frame = cv2.flip(self.frame, -1)

            self.hsv = cv2.cvtColor(self.frame.copy(), cv2.COLOR_BGR2HSV)

        except CvBridgeError as e:
            print('ex', e)
            pass


    @staticmethod
    def get_frame_center_pos(frame):
        """Método estático que retorna o centro de um dado frame.

        Args:
            frame (array): Frame o qual deseja-se a posição central.

        Returns:
            int: Valor da componente vertical das coordenada do ponto central.
            int: Valor da componente horizontal das coordenada do ponto central.
        """        
        center_y = frame.shape[0] // 2
        center_x = frame.shape[1] // 2

        return center_y, center_x


    @staticmethod
    def crop_frame(frame, vertical, horizontal):
        """Faz o recorte de um dado frame conforme intervalos de porcentagem da sua altura e comprimento.

        ex:
        ```python
        cropped = get_frame_rect(frame, (1/4, 3/4), (1/3, 2/3))
        ```

        Args:
            frame (array): Frame que deseja-se o recorte.
            vertical (list): Lista com os valores de range (inicial e final, respectivamente) em porcentagem da altura do recorte da imagem.
            horizontal (list): Lista com os valores de range em porcentagem do comprimento do recorte da imagem.

        Returns:
            array: frame recortado
        """            
        y_start = int(frame.shape[0] * vertical[0])
        y_end = int(frame.shape[0] * vertical[1])
        x_start = int(frame.shape[1] * horizontal[0])
        x_end = int(frame.shape[1] * horizontal[1])

        return frame[y_start:y_end, x_start:x_end]


    @staticmethod
    def mask_frame(frame, vertical, horizontal):
        """Faz o recorte de um dado frame conforme intervalos de porcentagem da sua altura e comprimento.

        ex:
        ```python
        cropped = get_frame_rect(frame, (1/4, 3/4), (1/3, 2/3))
        ```

        Args:
            frame (array): Frame que deseja-se o recorte.
            vertical (list): Lista com os valores de range (inicial e final, respectivamente) em porcentagem da altura do recorte da imagem.
            horizontal (list): Lista com os valores de range em porcentagem do comprimento do recorte da imagem.

        Returns:
            array: frame recortado
        """            
        y_start = int(frame.shape[0] * vertical[0])
        y_end = int(frame.shape[0] * vertical[1])
        x_start = int(frame.shape[1] * horizontal[0])
        x_end = int(frame.shape[1] * horizontal[1])

        blank = np.zeros_like(frame)

        blank[y_start:y_end, x_start:x_end] = frame[y_start:y_end, x_start:x_end]

        return blank

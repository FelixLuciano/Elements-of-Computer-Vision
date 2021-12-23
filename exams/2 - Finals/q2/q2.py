import cv2
import numpy as np


def encontra_figuras(img_bgr):
    """
    Cria e retorna uma nova imagem BGR com os
    pontos de fuga desenhados.

    NOTE: O critério para identificação das figuras é com base numa comparação das áreas e as medidas do "bounding rectangle" do contorno da forma.

    Entrada:
    - img_bgr: imagem original no formato BGR

    Saída:
    - resultado: imagem BGR com os nomes das figuras escritos 
    """
    resultado = img_bgr.copy()
    mask = cv2.inRange(resultado, (0, 0, 0), (1, 1, 1))

    cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones([3,3]), mask)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

    for contour in contours:
        x, y , width, height = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)

        rect_area = width * height
        radius = (width + height) / 4
        circle_area = np.pi * radius**2

        if rect_area - area < rect_area *.01:
            figname = "quadrado"
        elif circle_area - area < circle_area * 0.1:
            figname = "circulo"
        else:
            figname = "estrela"

        
        cv2.putText(resultado, figname, (x, y + height//2), cv2.FONT_HERSHEY_SIMPLEX, 1, (128, 128, 128), 2, cv2.LINE_AA)

    return resultado


if __name__ == "__main__":
    bgr = cv2.imread('bitmap.png')
    resultado = encontra_figuras(bgr)

    cv2.imwrite("figura_q2_resultado.png", resultado)

    cv2.imshow('Original', bgr)
    cv2.imshow('Nomes', resultado)
    cv2.waitKey()
    cv2.destroyAllWindows()

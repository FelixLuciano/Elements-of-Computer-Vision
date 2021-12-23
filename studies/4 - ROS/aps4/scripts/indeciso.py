#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
min_visaol = 3.5
min_visaor = 3.5

def scaneou(dado):
	global min_visaor
	global min_visaol
	dist = np.array(dado.ranges)
	visao_r = dist[315:360]
	visao_l = dist[0:45]
	min_visaor = min(visao_r)
	min_visaol = min(visao_l)
	#print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	#print("Leituras:")
	#print(np.array(dado.ranges).round(decimals=2))
	#print(min_visao)
	# #print("Intensities")
	# #print(np.array(dado.intensities).round(decimals=2))

if __name__=="__main__":

	rospy.init_node("le_scan")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
	
	while not rospy.is_shutdown():	
		#print(min_visao)

		#NOSSO VERSÃO DO ROBO INDENCISO (OBO)
		if min_visaol < 0.50:
			velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.3))

		elif min_visaor < 0.50:
			velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.3))

		else:
			velocidade = Twist(Vector3(0.3, 0, 0), Vector3(0, 0, 0))

		##### [CODIGO 2.4]
		# if min_visaor < 0.95 or min_visaol < 0.95:
		# 	velocidade = Twist(Vector3(-0.3, 0, 0), Vector3(0, 0, 0))

		# elif min_visaor > 1.05 or min_visaol > 1.05:
		# 	velocidade = Twist(Vector3(0.3, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(velocidade)
		rospy.sleep(0.5)
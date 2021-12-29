#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import time

import numpy as np
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

from .state import State
from .odometry import Odometry
from .laser import Laser


class Control(State, Odometry, Laser):
    def __init__(self, *arks, **kwargs):
        """Rotinas e o métodos de controle do atuadores do robô.

        Herda State, Odometry e Laser.
        """        
        super(Control, self).__init__(*arks, **kwargs)

        self.vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
        self.arm_publisher = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
        self.claw_publisher = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)

        self.vel = Twist()
        self.arm = -1.5
        self.claw = 0.0

        self.stop_robot()
        self.move_joints_down()


        rospy.on_shutdown(lambda: self.stop_robot())
      

    def update_vel(self):
        """Método para atualizar os comandos do atuador de velocidade.

        Isso ocorre porque a publicação em tópicos às vezes falha na primeira vez que você publica.

        Em sistemas de publicação contínua não há grande coisa, mas em sistemas que publicam apenas uma vez, É muito importante.
        """
        self.vel_publisher.publish(self.vel)


    def update_joints(self):
        """Método para atualizar os comandos dos atuadores do braço e da garra.
        """
        self.arm_publisher.publish(self.arm)
        self.claw_publisher.publish(self.claw)

        # Embora a proposta seja não obstruir o laço principal, o comando sleep é essencial para o funcionamento deste método.
        # self.rate.sleep()


    def move_joints_down(self):
        """Comando para deixar o braço do robô em posição de descanço, para baixo e com a garra fechada.
        """        
        self.arm = -1.5
        self.claw = 0
        
        self.update_joints()
    

    def move_joints_up(self):
        """Comando para deixar o braço do robô para cima e com a garra fechada.
        """
        self.arm = 1.5
        self.claw = -1
        
        self.update_joints()


    def stop_robot(self):
        """Comando para parar o robô.
        """        
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0

        self.update_vel()


    def set_angle(self, angle, max_speed=3.26):
        """Comando para que o robô se oriente para determinada direção em radianos conforme a odometria.

        Args:
            final_angle (float): Ângulo da direção a qual o robô deve se direcionar.
            max_speed (float, optional): Velocidade angular máxima que o robô assume enquanto rotaciona. O padrão é 3,26 rad/s.
        """        
        while True:
            actual_angle = self.get_orientation_angle()
            offset = self.get_turn_offset(actual_angle, angle)

            if np.abs(offset) > 1e-2:
                smooth_step = 1 - (1 - offset / np.pi)**2
                self.vel.angular.z = max_speed * smooth_step

                self.update_vel()
            else:
                break

        self.stop_robot()


    def turn_by(self, angle, max_speed):
        """Comando para que o robô rotacione uma determinada quantidade em radianos conforme.

        Args:
            angle (float): Ângulo em radianos que o robô deve rotacionar. Valores positivos rotacionam o robô em snetido horário.
            max_speed (float, optional): Velocidade angular máxima que o robô assume enquanto rotaciona. O padrão é 3,26 rad/s.
        """            
        actual_angle = self.get_orientation_angle()

        new_angle = actual_angle + angle

        if np.abs(new_angle) > np.pi:
            new_angle -= np.pi * np.sign(new_angle)
            new_angle *= -1
        
        self.set_angle(new_angle, max_speed)


    def turn_to(self, coords, max_speed):
        """Comando para que o robô se oriente a direção de determinada coordenada no plano conforme a odometria.

        Args:
            coords (list): Coordenadas (x e y, respectivamente) para qual o robê deve se orientar.
            max_speed (float, optional): Velocidade angular máxima que o robô assume enquanto rotaciona. O padrão é 3,26 rad/s.
        """        
        distance, angle = self.get_coordinates_distances(coords)

        self.set_angle(angle, max_speed)



    def walk_to(self, coords, max_linear_speed=0.26, max_angular_speed=3.26, max_distance=5E-2, obstacle_distance=3E-1):
        """Comando para que o robô vá para determinada coordenada, se orientando por odometria e corrigindo o curso dinamicamente.
        
        Parando quando atingir uma distância minima do objetivo ou caso encontre com algum obstáculo conforme detectado pelo sensor lidar.

        Args:
            coords (list): Coordenadas (x e y, respectivamente) para qual o robê deve ir.
            max_linear_speed (float, optional): Velocidade angular máxima que o robô assume enquanto rotaciona ou corrige a orientação. O padrão é 0.26 m/s.
            max_angular_speed (float, optional): Velocidade linear máxima que o robô assume enquanto anda. O padrão é 3.26 rad/s.
            max_distance (float, optional): Distância máxima que o robô deve estar das coordenadas para considerar que chegou. O padrão é 5E-2 m (5 cm).
            obstacle_distance (float, optional): Distância máxima que o robô deve de algum obstáculo em sua frente para abortar o comando e parar de andar. O padrão é 3E-1 m (30 cm).

        Returns:
            bool: Retorna se o robô chegou ao objetivo. Sendo True caso sim e False caso algum objeto o fez abortar antes durante o trageto.
        """        
        self.turn_to(coords, max_angular_speed)

        success = False

        while True:
            distance, final_angle = self.get_coordinates_distances(coords)

            if self.get_laser_inrange(-10, 10, obstacle_distance):
                break
            elif distance > max_distance:
                angle = self.get_orientation_angle()
                offset = self.get_turn_offset(angle, final_angle)

                self.vel.angular.z = max_angular_speed * offset
                self.vel.linear.x  = max_linear_speed * (1 - offset) * np.clip(4 * distance, 0.1, 1)

                self.update_vel()
            else:
                success = True
                break

        self.stop_robot()

        return success

    
    def walk_straight(self, distance, max_linear_speed=0.26, max_angular_speed=3.26, max_distance=5E-2, obstacle_distance=3E-1):
        """Comando para que o robô ande em linha reta por uma dada distância, se orientando por odometria e corrigindo o curso dinamicamente.
        
        Parando quando atingir uma distância minima do objetivo ou caso encontre com algum obstáculo conforme detectado pelo sensor lidar.

        Args:
            distance (float): Distância (em metros) que o robô deve andar.
            max_linear_speed (float, optional): Velocidade angular máxima que o robô assume enquanto rotaciona ou corrige a orientação. O padrão é 0.26 m/s.
            max_angular_speed (float, optional): Velocidade linear máxima que o robô assume enquanto anda. O padrão é 3.26 rad/s.
            max_distance (float, optional): Distância máxima que o robô deve estar das coordenadas para considerar que chegou. O padrão é 5E-2 m (5 cm).
            obstacle_distance (float, optional): Distância máxima que o robô deve de algum obstáculo em sua frente para abortar o comando e parar de andar. O padrão é 3E-1 m (30 cm).

        Returns:
            bool: Retorna se o robô chegou ao objetivo. Sendo True caso sim e False caso algum objeto o fez abortar antes durante o trageto.
        """        
        pos = (self.position.x, self.position.y)
        angle = self.get_orientation_angle()
        coords = (pos[0] + distance * np.cos(angle),  pos[1] + distance * np.sin(angle))

        return self.walk_to(coords, max_linear_speed, max_angular_speed, max_distance, obstacle_distance)

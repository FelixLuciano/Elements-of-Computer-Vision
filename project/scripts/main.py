#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from robot import Robot


if __name__ == "__main__":
  # cria o objeto do robô e o inicaliza.
  robot = Robot(is_simulation=True, params=("blue", 22, "car"))
  robot.init()

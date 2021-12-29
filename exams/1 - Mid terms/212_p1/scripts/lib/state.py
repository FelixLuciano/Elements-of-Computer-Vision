#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy


class State(object):
    def __init__(self, name, is_simulation=True):
        super(State, self).__init__()
        
        self.is_simulation = is_simulation
        self.interrupt = False

        rospy.on_shutdown(lambda: self.on_interrupt())

        print("Initializing...")
        rospy.init_node(name, anonymous=True)

        self.rate = rospy.Rate(10)  # 10 Hz


    def on_interrupt(self):
        print("Shutdown...")

        self.interrupt = True

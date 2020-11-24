#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from dhand.msg import Servo_move
from driver import *

# from sensor_msgs.msg import Image

import numpy as np
# import cv2
# from cv_bridge import CvBridge, CvBridgeError

import time

class Servo_m :
   def __init__ (self):
      self.b=Servo("ttyUSB0",False)
      
      self.b.alarm_reset()
      print "Alarm"
      time.sleep(1.5)

      self.b.modbus_on()
      print "Modbus on"
      time.sleep(1.5)

      self.b.servo_on()
      print "Servo On"
      time.sleep(1.5)

      self.b.home_return()
      print "Go Home"
      time.sleep(1.5)

      
      rospy.Subscriber("/dhand_grasp", Servo_move, self.callback)
      
   
   def callback(self,data):

      print [data.position,data.speed,data.acceleration,data.current_limit]
      #
      self.b.move_absolute_position(data.position,data.speed,data.acceleration,data.current_limit)
      
   def __del__(self):
      self.b.servo_off()
      print "Servo Off"

if __name__ == '__main__':
   rospy.init_node('servo_listener', anonymous=False)
   sm = Servo_m()
   rospy.spin()
   

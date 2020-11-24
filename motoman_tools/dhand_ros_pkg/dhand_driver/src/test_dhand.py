# Author : Thibault Barbie

import minimalmodbus
# import rospy
# from std_msgs.msg import String
import numpy as np
import time
import ipdb

class Servo:
    def __init__(self,name,debug):
        minimalmodbus.BAUDRATE=38400
        # minimalmodbus.BAUDRATE=230400

        self.s=minimalmodbus.Instrument("/dev/"+name,1)
        self.s.debug=debug

    def alarm_reset(self):
        # Alarm reset
        register=int('0407',16)
        print ('register ; ',register)
        self.s.write_bit(register,1,functioncode=5)

        # Restore normal status
        register=int('0407',16)
        self.s.write_bit(register,0,functioncode=5)

    def servo_on(self):
        # Servo ON
        register=int('0403',16)
        print ('register ; ',register)
        self.s.write_bit(register,1,functioncode=5)

    def servo_off(self):
        # Servo OFF
        register=int('0403',16)
        self.s.write_bit(register,0,functioncode=5)

    def modbus_on(self):
        # Modbus ON
        register=int('0427',16)
        print ('register ; ',register)
        self.s.write_bit(register,1,functioncode=5)

    def home_return(self):
        # Home return
        register=int('040B',16)
        self.s.write_bit(register,0,functioncode=5)
        self.s.write_bit(register,1,functioncode=5)

    def start_off(self):
        # Turn OFF the start signal
        register=int('0D00',16)
        self.s.write_register(register,4096,functioncode=6)

    def start_on(self):
        # Turn ON the start signal
        register=int('0D00',16)
        self.s.write_register(register,4104,functioncode=6)

    def pause(self):
        # Turn ON the pause
        register=int('0D00',16)
        self.s.write_register(register,16,functioncode=6)

    def ready(self):
        # Read if the servo is ready
        register=int('9005',16)
        print self.s.read_registers(register,1,functioncode=3)

    def move_absolute_position(self,position,speed,acceleration,push):
        # move_absolute_position(5,5,acceleration,push)
        # Move the servomotor to the position except if an obstacle is detected
        # position in mm, speed in mm/s, acceleration in G, push in percentage (0.2-0.7)
        register=int('9900',16)

        if (position<0):
            position=0
        elif (position>13):
            position=13 # Maximum value before touching the palm
        if (speed<0):
            speed=0
        elif (speed>20):
            speed=20 # Maximum value (in the manual)
        if (acceleration<0):
            acceleration=0
        elif (acceleration>0.209):
            acceleration=0.209 # Maximum value (empirical)
        if (push<0.2):
            push=0.2
        elif (push>0.7):
            push=0.7 # Maximum value said in manual

        l=[]
        l.append(0000)
        l.append(int(position*100))

        l.append(0000)
        l.append(10)

        l.append(0000)
        l.append(int(speed*100))

        l.append(int(acceleration*100))

        l.append(int(255*push))

        self.s.write_registers(register,l)

    def read_position(self):
        # Read the position of the servomotor
        register=int('9000',16)
        [a,b]= self.s.read_registers(register,2,functioncode=3)
        return b

    def read_alarm(self):
        # Read the alarm
        register=int('0500',16)
        print self.s.read_registers(register,6,functioncode=3)

    def read_position_completed(self):
        # Read if the position as been completed
        register=int('9014',16)
        print self.s.read_registers(register,1,functioncode=3)

    def read_torque(self):
        # Read the torque of the servomotor
        register=int('900C',16)
        print self.s.read_registers(register,2,functioncode=3)
    ####### M added
    def grip(self, grip_state=False):
        # Read the torque of the servomotor
        if (grip_state) :
          self.move_absolute_position(6.0, 10, 1, 0.5) ##dhand open
        else :
          self.move_absolute_position(-1, 10, 1, 0.5) ##dhand open
    def grip_toruqe(self, grip_state=False,Toruqe = 100):
        # Read the torque of the servomotor
        if (grip_state) :
          self.move_absolute_position(5.5, 10, 1, 0.5) ##dhand open
        else :
          self.move_absolute_position(-1.5, 10, 1, 0.5) ##dhand open


# class Servo_m :
#    def __init__ (self):
#       self.b=Servo("ttyUSB0",False)
#       self.b.modbus_on()
#       print "Modbus on"
#       time.sleep(1.5)

#       self.b.alarm_reset()
#       print "Alarm "
#       time.sleep(1.5)

#       self.b.servo_on()
#       print "Servo On"
#       time.sleep(1.5)

#       self.b.home_return()
#       print "Go Home"
#       time.sleep(1.5)


      # rospy.Subscriber("/dhand_grasp", Servo_move, self.callback)


   # def callback(self,data):

   #    print [data.position,data.speed,data.acceleration,data.current_limit]
   #    #
   #    self.b.move_absolute_position(data.position,data.speed,data.acceleration,data.current_limit)

   # def __del__(self):
   #    self.b.servo_off()
   #    print "Servo Off"

if __name__ == '__main__':
   # rospy.init_node('servo_listener', anonymous=False)
   # sm = Servo_m()
   # rospy.spin()
  dhand=Servo("ttyUSB0",True)
  print "set port"

  dhand.modbus_on()
  print "Modbus on"
  time.sleep(1.5)

  dhand.alarm_reset()
  print "Alarm Reset"
  time.sleep(1.5)

  dhand.servo_on()
  print "Servo On"
  time.sleep(1.5)

  dhand.home_return()
  print "home"
  time.sleep(3)

  dhand.move_absolute_position(5, 5, 1, 0.5)
  time.sleep(1.5)

# loop move
  # for i in range (1) :
  #   # ipdb.set_trace()
  #   print "Move finger"
  #   dhand.move_absolute_position(8, 50, 1, 0.1)
  #   time.sleep(1.5)
  #   dhand.move_absolute_position(3, 50, 1, 0.1)
  #   time.sleep(1.5)
  #   dhand.move_absolute_position(0, 50, 1, 0.1)
  #   time.sleep(1.5)
  # for i in range(2) :
  #   dhand.grip(True)
  #   time.sleep(2)
  #   dhand.grip(False)
  #   time.sleep(2)

  # while True :
  #   dhand.read_torque()
  #   time.sleep(0.1)


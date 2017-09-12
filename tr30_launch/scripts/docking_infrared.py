#!/usr/bin/env python
import rospy
import time
import serial
from std_msgs.msg import String

if __name__ == "__main__":
  rospy.init_node('docking_infrared', anonymous=True)
  pub = rospy.Publisher('/docking_obstacle', String, queue_size=10)
  serialport_name = rospy.get_param('~serialport_name', '/dev/ttyUSB0') 
  baudrate = rospy.get_param('~baudrate', '9600') 
  s = serial.Serial(serialport_name, baudrate)
  str = ''
  while 1:
    time.sleep(0.1)
    while s.inWaiting() > 0:
      str += s.read(1)
    if str != '':
      print '-------------------'
      print 'length:', len(str)
      for i in range(0,len(str)):
        print ord(str[i])
      str = ''
  

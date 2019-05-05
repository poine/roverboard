#!/usr/bin/env python
import platform, signal, sys
import math, numpy as np
import rospy, sensor_msgs
from sensor_msgs.msg import Joy

import time, serial, struct


#import odrive

class SerialOdrive:
    def __init__(self):
        self.device = '/dev/ttyACM0'
        self.ser = serial.Serial(port=self.device, baudrate=115200)

    def send(self, m, v):
        #s = struct.pack('cccc', chr(self.addr), chr(cmd), chr(data), chr(checksum))
        s = 'v {} {}\r\n'.format(m, v)
        self.ser.write(s)
        # request feedback
        self.ser.write('f {}\r\n'.format(m))
        pos, vel = self.ser.readline().split()
        print('m{}: p:{} v:{}'.format(m, pos, vel))
        
class Node:
    def __init__(self):
        self.odrive = SerialOdrive()
        rospy.init_node('nono_{}'.format('dumb_odrive_test'))
        rospy.Subscriber('/joy', sensor_msgs.msg.Joy, self.joy_callback)

    def run(self):
        while not rospy.is_shutdown():
            time.sleep(1)
            
         
    def joy_callback(self, msg):
        self.last_input = rospy.get_rostime()
        linear, angular = msg.axes[1], msg.axes[2] 
        #print linear
        self.odrive.send(0, linear*150+angular*-80)
        self.odrive.send(1, -linear*150+angular*-80)

    
def main():
    n = Node()
    n.run()

if __name__ == '__main__':
    np.set_printoptions(precision=7, suppress=True, linewidth=200)
    main()

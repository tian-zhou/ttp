#!/usr/bin/env python

"""
Description: 
    the program to communicate with Arduino via serial communication
    1) output -> control magnet
    2) input -> read force sensor feedback

Sample usage:
    from arduino import ARDUINO
    arduino = ARDUINO()
    arduino.connect()
    print 'force feedback:', arduino.readForceSensor()
    arduino.setMagnet(0)   

Author:
    Tian Zhou (leochou1991@gmail.com)

Date: 
    Nov 2, 2017

License: 
    GNU General Public License v3
"""

import serial
import glob
from time import sleep


class ARDUINO():
    def __init__(self):
        pass
    
    def connect(self, baud=9600):
        ports = glob.glob('/dev/ttyACM[0-9]*')

        res = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                res.append(port)
            except:
                pass
        if len(res) == 0:
            print "Arduino not found...exit..."
            exit(1)
        else:
            print "available ports: ", res

        port = res[0]
        print 'found Arduino at port: %s' % res[0]
        self.ser = serial.Serial(port, baud)
        
    def read(self):
        msg = self.ser.readline()
        return msg 

    def write(self, msg):
        self.ser.write(msg)

    def readForceSensor(self):
        # send "b" first and read the echoed back msg
        self.write("b *")
        sleep(0.3)

        # read the msg and decode
        msg = self.read()
        msg_split = msg.split(' ')
        f_up = int(msg_split[1])
        f_left = int(msg_split[3])
        f_down = int(msg_split[5])
        f_right = int(msg_split[7])
        checker = msg_split[8]
        assert(checker[:8] == 'abcd1234')
        f = {'up':f_up, 'left':f_left, 'down':f_down, 'right':f_right} 
        return f

    def setMagnet(self, level):
        # level in range [0,5]
        # print "set magnet level: %i" % level
        level_255 = int(255.0*(level/5.0))
        msg = "a " + str(level_255) + " *"
        self.ser.write(msg)
        # sleep(1)

def main():
    arduino = ARDUINO()
    arduino.connect()
    
    f = arduino.readForceSensor()
    print 'force feedback:', f 
    
    force = 0
    print "set force %i" % force
    arduino.setMagnet(force)
        
        
if __name__ == '__main__':
    main()



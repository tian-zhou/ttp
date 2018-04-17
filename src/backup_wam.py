#!/usr/bin/env python

"""
Description: 
    the program which governs communication with WAM robot
    get real-time heartbeat from WAM to know its position
    and send target joint position for WAM to go to

Sample usage:
    xxx

Author:
    Tian Zhou (leochou1991@gmail.com)

Date: 
    Nov 2, 2017

License: 
    GNU General Public License v3
"""

import socket
import rospy
import time


class WAM():
    def __init__(self):
        # init node for wam_node
        rospy.init_node('wam_node')

        # init the publisher
        # self.pub = rospy.Publisher('mm_bridge_output', String, queue_size=10)
    
    def init_socket(self, host, port, buflen):
        # init socket with WAM PC
        
        # create a socket object
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.buflen = buflen

        # connection to hostname on the port.
        re_try = 1
        while re_try:
            try:
                self.socket.connect((host, port))
                re_try = 0
            except:
                print("Connection failed. Retry after 0.1 seconds")
                re_try = 1
                time.sleep(0.1)
                continue

        print "Built socket connection with WAM PC..."
            
    def query_joint_position(self):
        self.socket.send('2')
        time.sleep(0.1)
        pac = self.socket.recv(self.buflen)
        print pac

    def move_joint(self, target):
        assert (len(target)==7)
        msg = '7 ' + ''.join([str(i)+' ' for i in target])
        print "move_joint msg: %s" % msg
        self.socket.send(msg)
        time.sleep(0.1)
        pac = self.socket.recv(self.buflen)
        assert (pac == 'complete')
    
    def go_home(self):
        print "go home WAM you are drunk!!!"
        self.socket.send('4')
        time.sleep(1)

if __name__ == '__main__':
    wam = WAM()
    wam.init_socket(host='128.46.125.212', port=4000, buflen=256)
    wam.query_joint_position()
    wam.move_joint([0, 0, 0, 0, 0, 0, 0])
    wam.go_home()


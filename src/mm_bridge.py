#!/usr/bin/env python

"""
Description: 
    the bridge program which receives socket packets from Multimodal.exe
    (Windows, C++) and the publishes the packet into the topic mm_bridge_output

Sample usage:
    from mm_bridge import MM_BRIDGE
    mm_bridge = MM_BRIDGE()
    mm_bridge.run()


Author:
    Tian Zhou (leochou1991@gmail.com)

Date: 
    Oct, 9, 2017

License: 
    GNU General Public License v3
"""


import rospy
from std_msgs.msg import String
import socket
import datetime
import time

class MM_BRIDGE:
    def __init__(self):
        # init node for mm_bridge_node
        rospy.init_node('mm_bridge_node')

        # init the publisher
        self.pub = rospy.Publisher('mm_bridge_output', String, queue_size=10)
        
    def init_socket(self, host, port, buflen):
        # init socket with Multimodal.exe in Windows C++
        
        # create a socket object
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.buflen = buflen

        # get local machine name
        # host = socket.gethostname()
        # host = '128.46.125.47'
        # port = 59500

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

        print "Socket connection with Multimodal.exe (Windows) is built..."
        
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # get a packet from Multimodal.exe
            if 0:
                pac = '2017_10_13_18_08_15_774 2.34 3.42 4.244 3.44 3.221 -2.11 -2.09 abcd1234 '
            else:
                # Receives self.buflen chars
                pac = self.socket.recv(self.buflen)
                
                # send ack back
                bytesSent = self.socket.send('Got it!\0')
                
            # publish the package
            self.pub.publish(pac)
            rate.sleep()

            # print something
            rospy.loginfo("received %i length Multimodal packet, publish to ROS..." % len(pac))
            # rospy.loginfo('send ack back with len %i' % bytesSent)


if __name__ == '__main__':
    mm_bridge = MM_BRIDGE()
    mm_bridge.init_socket(host='128.46.125.47', port=12345, buflen=4096)
    mm_bridge.run()


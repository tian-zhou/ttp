#!/usr/bin/env python

"""
Description: 
    the program which governs communication with WAM robot
    get real-time heartbeat from WAM to know its position
    and send target joint position for WAM to go to

Sample usage:
    wam = WAM()
    wam.init_socket(host='128.46.125.212', port=4000, buflen=256)
    wam.query_joint_position()
    wam.move_joint([0, 0, 0, 0, 0, 0, 0])
    wam.go_home()

Author:
    Tian Zhou (leochou1991@gmail.com)

Date: 
    Nov 2, 2017

License: 
    GNU General Public License v3
"""

import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import time
import socket
import numpy as np



def pprint(name, l):
    l_str = ''.join(['%.3f ' % item for item in l])
    print name + ': ' + l_str

class WAM:
    def __init__(self):
        # init node for wam_node
        rospy.init_node('wam_node')
        
        # everything about publishing robot joint state
        self.joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.joint_msg = JointState()
        self.joint_msg.name = ['wam/base_yaw_joint', 'wam/shoulder_pitch_joint', \
            'wam/shoulder_yaw_joint', 'wam/elbow_pitch_joint', 'wam/wrist_yaw_joint', \
            'wam/wrist_pitch_joint', 'wam/palm_yaw_joint']
        self.joint_msg_sep = 0 

        # everything about publishing robot cart state
        self.cart_pub = rospy.Publisher('cart_states', Pose, queue_size=10)
        self.cart_msg = Pose()
        
        # init the subscriber now
        rospy.Subscriber("wam_command", String, self.wam_command_msg_callback)
        self.move_wam_msg = None

    def clean_shutdown(self):
        """
        Exits example cleanly by moving head to neutral position and
        maintaining start state
        """
        print("\nExiting moveWAM_BRIDGE()...")
        if self.socket:
            self.go_home()
            self.socket.close()
            print "Socket closed properly..."

    def init_socket(self, host, port, buflen):
        # init socket with Multimodal.exe in Windows C++
        if host == 'local_file':
            file = open("/home/tzhou/Workspace/catkin_ws/src/ttp/model/WAM_IP.txt", "r") 
            host = file.read()
            print "recovered WAM IP %s from local file..." % host

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
        print "Heartbeat started to get real-time robot joint positions..."
        print "Wait for path planning result to move the robot..."
    
    def query_joint_pose(self):
        self.socket.send('2')
        time.sleep(0.01)
        pac = self.socket.recv(self.buflen)
        return pac

    def query_cart_pose(self):
        self.socket.send('9')
        time.sleep(0.01)
        pac = self.socket.recv(self.buflen)
        return pac

    def go_home(self):
        print "go home WAM you are drunk!!!"
        self.socket.send('4')
        time.sleep(1)
        
    def wam_command_msg_callback(self, msg):
        self.move_wam_msg = msg.data

    def publish_joint_pose(self, pac):
        pac = pac.split()[:7]
        robot_pos = [float(s[:8]) for s in pac]
        wam_joint = robot_pos[0:7]
        
        self.joint_msg.position = wam_joint 
        self.joint_msg.header.seq = self.joint_msg_sep
        self.joint_msg_sep += 1
        current_time = time.time()
        self.joint_msg.header.stamp.secs = int(current_time)
        self.joint_msg.header.stamp.nsecs = int((current_time-int(current_time))*1e9)
        self.joint_pub.publish(self.joint_msg)

    def publish_cart_pose(self, pac):
        pac = pac.split()[:8]
        cart_pos = [float(s[:8]) for s in pac]
        
        self.cart_msg.position.x = cart_pos[1]
        self.cart_msg.position.y = cart_pos[2]
        self.cart_msg.position.z = cart_pos[3]
        self.cart_msg.orientation.w = cart_pos[4]
        self.cart_msg.orientation.x = cart_pos[5]
        self.cart_msg.orientation.y = cart_pos[6]
        self.cart_msg.orientation.z = cart_pos[7]

        self.cart_pub.publish(self.cart_msg)
    
    def run(self):
        rospy.on_shutdown(self.clean_shutdown)
        while not rospy.is_shutdown():
            pac = self.query_joint_pose()
            self.publish_joint_pose(pac)
            
            pac = self.query_cart_pose()
            self.publish_cart_pose(pac)

            # we have to do it here, otherwise the callback function
            # mess up with the query joint/cart poses process
            if self.move_wam_msg:
                self.socket.send(self.move_wam_msg)
                time.sleep(0.01)
                pac = self.socket.recv(self.buflen)
                assert (pac == 'complete')

        rospy.signal_shutdown("run() finished...")

if __name__ == '__main__':
    try:
        wam = WAM()
        wam.init_socket(host='local_file', port=4000, buflen=256)
        wam.run()
    except KeyboardInterrupt:
        print("Ok ok, keyboard interrupt, quitting")
        sys.exit(1)
    else:
        print("Normal termination")
        sys.exit(2)
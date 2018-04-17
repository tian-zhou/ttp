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
from std_msgs.msg import Float32MultiArray
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
        self.wam_joint = None
        self.joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.joint_msg = JointState()
        self.joint_msg.name = ['wam/base_yaw_joint', 'wam/shoulder_pitch_joint', \
            'wam/shoulder_yaw_joint', 'wam/elbow_pitch_joint', 'wam/wrist_yaw_joint', \
            'wam/wrist_pitch_joint', 'wam/palm_yaw_joint']
        self.joint_rate = rospy.Rate(10)
        self.joint_msg_sep = 0 

        # init the subscriber now
        rospy.Subscriber("wam_bridge_planned_path", Float32MultiArray, self.path_msg_callback)
        self.path_point = None

        # self.socket_mutex = Lock()

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
        return pac 

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
        
    def path_msg_callback(self, msg):
        self.path_point = msg.data
        
    def decode_robot_joint_state_and_publish(self, pac):
        pac = pac.split()[:7]
        robot_pos = [float(s[:8]) for s in pac]
        wam_joint = robot_pos[0:7]
        
        # publish current joint position
        self.joint_msg.position = wam_joint 
        self.joint_msg.header.seq = self.joint_msg_sep
        self.joint_msg_sep += 1
        current_time = time.time()
        self.joint_msg.header.stamp.secs = int(current_time)
        self.joint_msg.header.stamp.nsecs = int((current_time-int(current_time))*1e9)
        self.joint_pub.publish(self.joint_msg)
            
    def notThereYet(self, target, current, thres):
        if not target:
            print "target not inited..."
            return 0
        # pprint('decoded joint_states', current)
        # pprint('decoded trget_joints', target)
        offset = np.subtract(current, target)
        dis = np.linalg.norm(offset)
        return dis>thres

    def run(self):
        rospy.on_shutdown(self.clean_shutdown)
        while not rospy.is_shutdown():
            pac = self.query_joint_position()
            self.decode_robot_joint_state_and_publish(pac)
                
            if self.notThereYet(self.path_point, self.joint_msg.position, thres=0.1):
                self.move_joint(self.path_point)
               
        rospy.signal_shutdown("run() finished...")

if __name__ == '__main__':
    try:
        wam = WAM()
        wam.init_socket(host='128.46.125.213', port=4000, buflen=256)
        wam.run()
    except KeyboardInterrupt:
        print("Ok ok, keyboard interrupt, quitting")
        sys.exit(1)
    else:
        print("Normal termination")
        sys.exit(2)

    


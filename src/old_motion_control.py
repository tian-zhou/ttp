#!/usr/bin/env python

"""
Description: 
    

Sample usage:


Author:
    Tian Zhou (leochou1991@gmail.com)

Date: 
    Oct, 9, 2017

License: 
    GNU General Public License v3
"""

import rospy
import sys
import numpy as np
from std_msgs.msg import Float32, String
import pandas as pd
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import time
from arduino import ARDUINO
import keyboard
import cv2

def pprint(name, l):
    l_str = ''.join(['%.3f ' % item for item in l])
    print name + ': ' + l_str

class MOTION_CONTROL:
    def __init__(self):
        # init node for mm_bridge
        rospy.init_node('motion_control_node')

        # init the subscriber
        rospy.Subscriber("cmd_pred", String, self.cmd_msg_callback)
        self.cmd_queue = pd.DataFrame(columns = ['item_id', 'item_pred', 
                    'intent_pred', 'timestamp', 'comment', 'status'])
        
        # init the publisher for sending to MoveWAM
        self.wam_command_pub = rospy.Publisher('wam_command', String, queue_size=10)
        self.wam_command_msg = String()
        
        # subscribe to current joint states
        rospy.Subscriber("joint_states", JointState, self.wam_joint_states_msg_callback)
        self.wam_joint_states = None
        self.joint_2norm_tol = 0.02

        rospy.Subscriber("cart_states", Pose, self.wam_cart_states_msg_callback)
        self.wam_cart_states = None
        self.cart_2norm_tol = 0.01

        # init arduino for magnet
        self.arduino = ARDUINO()
        self.arduino.connect()

        # for display
        # self.img_happy = cv2.imread('../data/robot.png')
        # self.img_cry = cv2.imread('../data/robot_cry.jpg')
        # cv2.imshow('WAM control', self.img_happy)
        # cv2.waitKey(1)

        print "init finished, waiting for motion control command..."
        
    def load_landmarks(self, landmark_path):
        self.df_landmark = pd.read_csv(landmark_path)
        # print '-' * 30
        # print 'loaded landmark_path at: %s' % landmark_path   
        # print self.df_landmark['name'].to_string(index=False)

    def cmd_msg_callback(self, msg):
        msg_split = msg.data.split(',')
        cmd = {}
        cmd['item_id'] = int(msg_split[0])
        cmd['item_pred'] = msg_split[1]
        cmd['intent_pred'] = float(msg_split[2])
        cmd['timestamp'] = msg_split[3]
        cmd['comment'] = msg_split[4]
        cmd['status'] = 'wait_in_queue'
        self.cmd_queue = self.cmd_queue.append(cmd, ignore_index=True)

    def wam_joint_states_msg_callback(self, msg):
        self.wam_joint_states = msg.position
    
    def wam_cart_states_msg_callback(self, msg):
        self.wam_cart_sates = msg

    def check_abort(self):
        key = cv2.waitKey(1)
        if key == ord('a'):
            #cv2.imshow('WAM control', self.img_cry)
            #cv2.waitKey(1)
            return True
        else:
            #cv2.imshow('WAM control', self.img_happy)
            #cv2.waitKey(1)
            return False
        
    def goto_pose(self, item_name, z_above = 0, fixQuat = 1):
        df_select = self.df_landmark.loc[(self.df_landmark['name'] == item_name)]
        targetCp = []
        for i in range(3):
            targetCp.append(df_select.iloc[0, i+8])
        targetCp[2] += z_above

        msg = '8 ' + ''.join([str(i)+' ' for i in targetCp])
        msg += '%i ' % fixQuat
        # print "<<< moving to cart pose: %s" % item_name   
        while not rospy.is_shutdown():
            self.wam_command_pub.publish(msg)
            currentCp = []
            currentCp.append(self.wam_cart_sates.position.x)
            currentCp.append(self.wam_cart_sates.position.y)
            currentCp.append(self.wam_cart_sates.position.z)
            offset = np.subtract(currentCp, targetCp[0:3])
            if np.linalg.norm(offset) < self.cart_2norm_tol:
                break
        # print ">>> reached cart pose: %s" % item_name  

    def goto_joint(self, item_name):
        df_select = self.df_landmark.loc[(self.df_landmark['name'] == item_name)]
        targetJp = []
        for i in range(7):
            targetJp.append(df_select.iloc[0, i+1])

        # publish the command to go to joint
        assert (len(targetJp) == 7)
        msg = '7 ' + ''.join([str(i)+' ' for i in targetJp])
        # print "<<< moving to joint pose: %s" % item_name     
        while not rospy.is_shutdown():
            self.wam_command_pub.publish(msg)
            offset = np.subtract(self.wam_joint_states, targetJp)
            # print 'self.wam_joint_states', self.wam_joint_states
            # print 'targetJp', targetJp
            # print 'np.linalg.norm(offset)', np.linalg.norm(offset)
            # print 'self.joint_2norm_tol', self.joint_2norm_tol
            if np.linalg.norm(offset) < self.joint_2norm_tol:
                break
        # print ">>> reached joint pose: %s" % item_name        
           
    def run(self):
        print_wait_for_command = True
        while not rospy.is_shutdown():
            # find to_do tasks whose "status" is "wait_in_queue"
            to_do_cmds = self.cmd_queue.loc[self.cmd_queue['status'] == 'wait_in_queue']
            
            # check to see if there are any to_do tasks
            if len(to_do_cmds) == 0:
                if print_wait_for_command:
                    print "waiting for new command request..."
                    print_wait_for_command = False
                continue

            # find the first task that is not finished
            cmd = to_do_cmds.iloc[0]
            cmd_index = to_do_cmds.iloc[[0]].index
            
            # check invalid command
            if cmd['item_pred'] not in self.df_landmark['name'].values:
                print "invalid command: [%s]" % cmd['item_pred']
                self.cmd_queue.at[cmd_index, 'status'] = 'invalid command'
                print_wait_for_command = True
                continue
            
            # execute command
            print '----------------'
            print 'execute command: \n', cmd
            time_s = time.time()
            self.goto_joint(cmd['item_pred'])
            if cmd['item_pred'] in ['ready', 'home', 'deliver_up', 'deliver_down']:
                pass
            else:
                self.arduino.setMagnet(5)
                self.goto_pose(cmd['item_pred'], z_above = 0.03) 
                self.goto_joint("deliver_up")
                self.arduino.setMagnet(0)
                if cmd['item_pred'] not in ['tape', 'marker', 'left stile', 'right stile']:
                    self.goto_joint("deliver_down")
                    if cmd['item_pred'][:9] == 'thumbtack':
                        self.goto_joint("deliver_up") # works very well!!!
                self.goto_joint("ready")
            self.cmd_queue.at[cmd_index, 'status'] = 'done'
            print "executation finished after %.3f seconds" % (time.time()-time_s)
            print_wait_for_command = True

        print "----\ncmd queue:\n", self.cmd_queue
        
def main():
    motion_control = MOTION_CONTROL()
    motion_control.load_landmarks('/home/tzhou/Workspace/catkin_ws/src/ttp/data/joint_cart.csv')
    motion_control.run()

if __name__ == '__main__':
    main()
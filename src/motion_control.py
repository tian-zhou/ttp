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
        self.cmd = None
        self.last_cmd = {'item_id':-1, 'intent_pred':-1}
        self.cmd_history = []
        
        # init the publisher for sending to MoveWAM
        self.wam_command_pub = rospy.Publisher('wam_command', String, queue_size=10)
        self.wam_command_msg = String()
        self.abort_flag = False

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
        self.img_happy = cv2.imread('../data/robot.png')
        self.img_cry = cv2.imread('../data/robot_cry.jpg')
        cv2.imshow('WAM control', self.img_happy)
        cv2.waitKey(1)
        print "init finished, waiting for motion control command..."
        
    def load_landmarks(self, landmark_path):
        self.df_landmark = pd.read_csv(landmark_path)
        # print '-' * 30
        # print 'loaded landmark_path at: %s' % landmark_path   
        # print self.df_landmark['name'].to_string(index=False)

    def cmd_msg_callback(self, msg):
        self.msg_str = msg.data
        msg_split = msg.data.split(',')
        self.cmd = {}
        self.cmd['item_id'] = int(msg_split[0])
        self.cmd['item_pred'] = msg_split[1]
        self.cmd['intent_pred'] = float(msg_split[2])
        self.cmd['timestamp'] = msg_split[3]
        self.cmd['comment'] = msg_split[4] 
        self.cmd_history.append(self.msg_str)

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
            if self.check_abort():
                return
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
            if self.check_abort():
                return
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
        no_deliver = True
        # rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # rate.sleep()
            if not self.cmd:
                continue
            if self.cmd['item_pred'] not in self.df_landmark['name'].values:
                print "invalid command: [%s]" % self.cmd['item_pred']
                continue
            if self.cmd['item_id'] == self.last_cmd['item_id'] and \
                self.cmd['intent_pred'] == self.last_cmd['intent_pred']:
                # print "same command as last ??? => continue"
                continue

            self.last_cmd = self.cmd
            print 'execute command: [%s]' % self.msg_str

            if self.cmd['item_pred'] in ['ready', 'home', 'deliver_up', 'deliver_down']:
                self.goto_joint(self.cmd['item_pred'])
            else:
                # pick it up
                self.goto_joint(self.cmd['item_pred'])
                if no_deliver:
                    time.sleep(0.5)
                    self.goto_joint("ready")   
                else:
                    self.arduino.setMagnet(5)
                    time.sleep(0.3)
                    if 1:
                        self.goto_joint("ready")   
                    else:
                        self.goto_pose(self.cmd['item_pred'], z_above=0.05)
                    
                    # wait until intent becomes 1
                    # wrong, if there comes a new command with intent_pred is 1
                    # it won't be passed here since it is still in inf loop here
                    # we need to skip the rest, or just wait for 
                    if self.cmd['intent_pred'] == 1:
                        # move to exchange pad area, do one of the three
                        # 1) just drop it
                        # 2) wait for 2 seconds for human to pick it, then drop it if human does not respond
                        # 3) when force sensor reading changes a lot, turn off
                        # do 1) frist, easy, just drop it
                        self.goto_joint("deliver_up")
                        self.arduino.setMagnet(0)
                        if self.cmd['item_pred'] not in ['tape', 'marker']:
                            self.goto_joint("deliver_down")
                        time.sleep(0.3)
                        self.goto_joint("ready")
                    
        print "cmd history:"
        for c in self.cmd_history:
            print c
        
def main():
    motion_control = MOTION_CONTROL()
    motion_control.load_landmarks('/home/tzhou/Workspace/catkin_ws/src/ttp/data/joint_cart.csv')
    motion_control.run()

if __name__ == '__main__':
    main()
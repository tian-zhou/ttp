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
from datetime import datetime


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
        self.motion_queue = pd.DataFrame(columns = ['item_id', 'item_pred', 'step', 
                    'motion', 'status', 'timestamp'])
        
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
        cmd['status'] = 'command arrived'
        cmd = self.motion_plan(cmd)
        self.cmd_queue = self.cmd_queue.append(cmd, ignore_index=True)

    def motion_plan(self, cmd):
        # self.motion_queue = pd.DataFrame(columns = ['item_id', 'item_pred', 'step', 
            # 'motion', 'status', 'timestamp'])

        # check invalid command
        if cmd['item_pred'] not in self.df_landmark['name'].values:
            print "invalid command: [%s]" % cmd['item_pred']
            cmd['status'] = 'invalid command'
            return 
        
        # plan motions for command
        # print 'motion planning for command: \n', cmd
        motion = {}
        motion['item_id'] = cmd['item_id']
        motion['item_pred'] = cmd['item_pred']
        motion['status'] = 'to_do'
        motion['timestamp'] = datetime.now().strftime('%H:%M:%S:%f')[:-4]

        # go to this pose
        motion['step'] = 0
        motion['motion'] = 'goto_joint,%s' % motion['item_pred']
        self.motion_queue = self.motion_queue.append(motion, ignore_index=True)

        if motion['item_pred'] in ['ready', 'home', 'deliver_up', 'deliver_down']:
            pass
        else:
            motion['step'] += 1
            motion['motion'] = 'turn_on_magnet'
            self.motion_queue = self.motion_queue.append(motion, ignore_index=True)

            motion['step'] += 1
            motion['motion'] = 'goto_pose,%s,0.03' % motion['item_pred']
            self.motion_queue = self.motion_queue.append(motion, ignore_index=True)

            motion['step'] += 1
            motion['motion'] = 'goto_joint,deliver_up'
            self.motion_queue = self.motion_queue.append(motion, ignore_index=True)

            motion['step'] += 1
            motion['motion'] = 'turn_off_magnet'
            self.motion_queue = self.motion_queue.append(motion, ignore_index=True)
 
            if motion['item_pred'] not in ['tape', 'marker', 'left stile', 'right stile']:
                motion['step'] += 1
                motion['motion'] = 'goto_joint,deliver_down'
                self.motion_queue = self.motion_queue.append(motion, ignore_index=True)

                if motion['item_pred'][:9] == 'thumbtack':
                    motion['step'] += 1
                    motion['motion'] = 'goto_joint,deliver_up' # works very well!!!
                    self.motion_queue = self.motion_queue.append(motion, ignore_index=True)

            motion['step'] += 1
            motion['motion'] = 'goto_joint,ready'
            self.motion_queue = self.motion_queue.append(motion, ignore_index=True)

        cmd['status'] = 'motion planning finished'

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
        # print_wait_for_command = True
        print '| id |     item     | step |           motion           | status |    time     |'
        while not rospy.is_shutdown():
            # find planned motions whose "status" is "to_be_executed"
            to_do_motions = self.motion_queue.loc[self.motion_queue['status'] == 'to_do']
            
            # check to see if there are any to_do motion
            if len(to_do_motions) == 0:
                # if print_wait_for_command:
                #     print "waiting for new planned motions..."
                #     print_wait_for_command = False
                continue

            # find the first motion that is not finished
            motion = to_do_motions.iloc[0]
            motion_index = to_do_motions.iloc[[0]].index

            # execute motion
            # print '| %-2i | %-12s |  %-2i  | %-26s | %-6s | %10s |' % (motion['item_id'], motion['item_pred'], motion['step'], 
                    # motion['motion'], motion['status'], motion['timestamp'])            
            motion_cmd = motion['motion']
            if motion_cmd == 'turn_on_magnet':
                self.arduino.setMagnet(5)
            elif motion_cmd == 'turn_off_magnet':
                self.arduino.setMagnet(0)
            elif motion_cmd[:10] == 'goto_joint': 
                item = motion_cmd.split(',')[-1]
                self.goto_joint(item)
            elif motion_cmd[:9] == 'goto_pose':
                item = motion_cmd.split(',')[-2]
                z_above = float(motion_cmd.split(',')[-1])
                self.goto_pose(item, z_above) 
            else:
                print "unrecognized motion command %s" % motion_cmd

            # change status
            self.motion_queue.at[motion_index, 'status'] = 'done'
            self.motion_queue.at[motion_index, 'timestamp'] = datetime.now().strftime('%H:%M:%S:%f')[:-4]
            motion['status'] = 'done'
            motion['timestamp'] = datetime.now().strftime('%H:%M:%S:%f')[:-4]
            
            # print result motion
            print '| %-2i | %-12s |  %-2i  | %-26s | %-6s | %10s |' % (motion['item_id'], motion['item_pred'], motion['step'], 
                    motion['motion'], motion['status'], motion['timestamp'])
            
        print "----\ncmd queue:\n", self.cmd_queue
        print "----\nmotion queue:\n", self.motion_queue
        

def main():
    motion_control = MOTION_CONTROL()
    motion_control.load_landmarks('/home/tzhou/Workspace/catkin_ws/src/ttp/data/joint_cart.csv')
    motion_control.run()

if __name__ == '__main__':
    main()
#!/usr/bin/env python

"""
Description: 
    the path planning for WAM robot
    1) given the current robot position
    2) given the desired position to pick up an object
    3) given all real-time obstacles
    =>  plan a path and command WAM to move

Sample usage:
    path_plan = PATH_PLAN()
    path_plan.read_landmarks('../data/landmark_sample.csv')
    path_plan.run()


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
from std_msgs.msg import Float32, Float32MultiArray, String
import pandas as pd
from dsp import DSP
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import JointState
import time
from arduino import ARDUINO


def pprint(name, l):
    l_str = ''.join(['%.3f ' % item for item in l])
    print name + ': ' + l_str

class PATH_PLAN:
    def __init__(self):
        # init node for mm_bridge
        rospy.init_node('path_plan_node')

        # init the subscriber
        rospy.Subscriber("intent_pred", Float32, self.intent_msg_callback)
        self.intent_pred = None

        rospy.Subscriber("item_pred", String, self.item_msg_callback)
        self.item_pred = None
        self.last_item_id = -91235

        rospy.Subscriber("mm_bridge_output", String, self.mm_msg_callback)
        self.human_pose = None

        rospy.Subscriber("joint_states", JointState, self.wam_joint_states_msg_callback)
        self.wam_joint_states = None

        # init the publisher for sending to MoveWAM
        self.path_pub = rospy.Publisher('wam_bridge_planned_path', Float32MultiArray, queue_size=10)
        self.path_msg = Float32MultiArray()
        self.joint_2norm_tol = 0.1
       
        # init the signal processing unit
        self.dsp = DSP()
    
        # init moveit
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")

        # init arduino for magnet
        self.arduino = ARDUINO()
        self.arduino.connect()
        
    def load_landmarks(self, landmark_path):
        self.df_landmark = pd.read_csv(landmark_path)
        print '-' * 30
        print 'loaded landmark_path at: %s' % landmark_path   
        print self.df_landmark['name'].to_string(index=False)

    def intent_msg_callback(self, msg):
        self.intent_pred = msg.data

    def item_msg_callback(self, msg):
        msg_split = msg.data.split()
        self.item_id = int(msg_split[0])
        self.item_pred = msg_split[1]

    def wam_joint_states_msg_callback(self, msg):
        self.wam_joint_states = msg.position

    def mm_msg_callback(self, msg):
        # decode the msg into the format that we want
        # raw = self.dsp.decode_packet(msg.data)
        # self.dsp.df.fillHumanPose(raw)

        # now think about which joints of human should we worry about
        # and how to represent them
        # I feel that the following should be used:
        #   Head                    !!! important
        #   SpineMid, SpineBase
        #   ShoulderLeft, ShoulderRight,
        #   ElbowLeft, ElbowRight   !!! important
        #   HandLeft, HandRight     !!! important
        # self.human_pose = self.dsp.df.kinect

        # fake it
        self.human_pose = 'fake'
        pass 

    def reach_target(self, item_name, destination_only=True):
        # get the position for the target object
        df_select = self.df_landmark.loc[(self.df_landmark['name'] == item_name)]
        if len(df_select.index) != 1:
            print "path_plan: cannot find targetJp for item %s" % item_name
            return 

        targetJp = []
        for i in range(7):
            targetJp.append(df_select.iloc[0, i+1])
        # pprint('targetJp', targetJp)
        path = self.plan_a_path(targetJp)
        self.execute_path(path, destination_only)

    def execute_command(self):
        if not self.item_pred:
            print "invalid command: [%s]" % self.item_pred
            return 

        if self.item_id == self.last_item_id:
            # skip the repeated request, each command should only be done once
            return
        else:
            self.last_item_id = self.item_id

        if self.item_pred in ['ready', 'home', 'deliver', 'deliver_up']:
            # just execute it
            self.reach_target(self.item_pred)
            
        else:
            # plan a whole instrument delivery sequence
            
            # pick it up
            self.reach_target(self.item_pred)
            self.arduino.setMagnet(5)
            time.sleep(0.5)

            # move to ready
            self.reach_target("ready")
            
            # wait for intent to be 1, and drop
            while 1:
                # move to exchange pad area, do one of the three
                # 1) just drop it
                # 2) wait for 2 seconds for human to pick it, then drop it if human does not respond
                # 3) when force sensor reading changes a lot, turn off
                if self.intent_pred:
                    self.reach_target("deliver")
                    self.arduino.setMagnet(0)
                    #self.reach_target("deliver_up")
                    break
                    
            self.reach_target("ready")

    def plan_a_path(self, targetJp):
        # === input ===
        #   1) use the desired location -> self.item_pred
        #   2) use all obstacles -> self.human_pose -> ignore for now
        #   3) use the current robot location -> self.wam_joint_states

        # === output ===
        # path

        if not self.human_pose or not self.wam_joint_states:
            print "path planning requirement not met..."
            print "human_pose: ", self.human_pose
            print "wam_joint_states: ", self.wam_joint_states
            return ''

        self.group.clear_pose_targets()
        self.group.set_joint_value_target(targetJp)
            
        # set obstacles, do something with human_pose
        # skip for now.

        # plan a path
        plan = self.group.plan()
        return plan

    def execute_path(self, plan, destination_only):
        if plan == '':
            print "no plan found, skip..."
            return
        plan_points = plan.joint_trajectory.points
        if destination_only:
            # only use the target joint, don't use the planned path 
            # there are too many intermediate points along the path
            # getting to each one of them makes the movement non-smooth
            # so I suggest to just use the final target joint value
            plan_points = [plan_points[-1]]
        N = len(plan_points)
        # print '# of landmarks in planned path: %i' % N
        rate = rospy.Rate(10)
        for i in range(N):
            targetJp = plan_points[i].positions
            
            # publish once and then wait for it to get there
            self.path_msg.data = targetJp
            self.path_pub.publish(self.path_msg)
            # rate.sleep()
                
            while not rospy.is_shutdown():
                # wait for robot to reach there
                # pprint('targetJp', targetJp)
                # pprint('currentJp', self.wam_joint_states)

                offset = np.subtract(self.wam_joint_states, targetJp)
                dis = np.linalg.norm(offset)
                if  dis > self.joint_2norm_tol:
                    #print "new   distance to targetJp: %.3f > %.3f" % \
                    #    (dis, self.joint_2norm_tol)
                    # pprint('offset', offset)
                    pass
                else:
                    #print "final distance to targetJp: %.3f < %.3f" % \
                    #    (dis, self.joint_2norm_tol)
                    break
            print "reached landmark %i/%i" % (i+1, N)
            # print '='*30
                
    def run(self):
        while not rospy.is_shutdown():
            self.execute_command()
          
def main():
    path_plan = PATH_PLAN()
    path_plan.load_landmarks('/home/tzhou/Workspace/catkin_ws/src/ttp/data/chair_new.csv')
    path_plan.run()

if __name__ == '__main__':
    main()
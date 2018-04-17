#!/usr/bin/env python

"""
Description: 
    subscribe to the joint_states and based on user key input
    record the current robot position (and a string) into 
    a local csv file.
    It is for landmark generation purposes.

Sample usage:
    record_landmark = RECORD_LANDMARK()
    record_landmark.init_filepath()
    record_landmark.run()


Author:
    Tian Zhou (leochou1991@gmail.com)

Date: 
    Oct, 23, 2017

License: 
    GNU General Public License v3
"""

import rospy
import pandas as pd
from sensor_msgs.msg import JointState

def pprint(name, l):
    l_str = ''.join(['%.3f ' % item for item in l])
    print name + ': ' + l_str

class RECORD_LANDMARK:
    def __init__(self):
        # init node for mm_bridge
        rospy.init_node('record_landmark')

        rospy.Subscriber("joint_states", JointState, self.wam_joint_states_msg_callback)
        self.wam_joint_states = None

    def init_filepath(self):
        fn = raw_input('Please enter landmark filename: ')
        fn = '../data/'+fn+'.csv'
        print 'landmark file will be saved into %s' % fn
        self.fn = fn
    
    def wam_joint_states_msg_callback(self, msg):
        self.wam_joint_states = msg.position

    def run(self):
        df = pd.DataFrame(columns = ['name','space','x0','x1','x2','x3','x4','x5','x6'])
        while not rospy.is_shutdown():
            name = raw_input('Enter landmark name: ')
            if name == 'quit':
                break
            row = {'name':name, 'space':'joint'}
            for i in range(7):
                row['x'+str(i)] = self.wam_joint_states[i]
            print "adding row", row
            df = df.append(row, ignore_index=True)
        df.to_csv(self.fn, sep=',', index=False)
        print "final landmark:\n", df
        print "landmark file saved into %s" % self.fn
            
if __name__ == '__main__':
    record_landmark = RECORD_LANDMARK()
    record_landmark.init_filepath()
    record_landmark.run()

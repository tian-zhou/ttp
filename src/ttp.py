#!/usr/bin/env python

"""
Description: 
    the main function for turn-taking prediction (ttp) 
    1) subscribs to the mm_bridge_output topic
    2) decodes all the data into corresponding sensor buffers
    3) signal preprocessing (denoising, normalization)
    4) feature extraction
    5) feed into RNN and get output
    6) publish the output (when and what) into the topic robot_control

Sample usage:
    ttp = TTP(mode='early')
    ttp.run()

Author:
    Tian Zhou (leochou1991@gmail.com)

Date:
    Oct, 9, 2017

License:
    GNU General Public License v3
"""

import rospy
from std_msgs.msg import Float32, String, Float32MultiArray
from dsp import DSP
import keras 
import numpy as np
import readchar
import pandas as pd


class TTP:
    def __init__(self, mode):
        if mode not in ['early', 'speech', 'teleop']:
            rospy.logerr('unrecognized TTP mode %s' % mode)
        self.mode = mode
        
        # init node for mm_bridge
        rospy.init_node('ttp_node')

        # init the subscriber
        rospy.Subscriber("mm_bridge_output", String, self.mm_msg_callback)

        # init the publisher for intent
        self.intent_pub = rospy.Publisher('intent_pred', Float32, queue_size=10)
        self.f_buf_size = 20
        self.f_buf = []
        self.zprev = None

        # init the publisher for object
        self.item_pub = rospy.Publisher('item_pred', String, queue_size=10)
        self.item_id = 0

        # init the signal processing unit
        self.dsp = DSP()
        self.mu = None
        self.sigma = None

    def grace_exit(self, exit_msg):
        print "!" * 20
        print '(grace?) exit with message:'
        print '---\n', exit_msg, '\n---'
        self.intent_pub.publish(1)
        self.item_pub.publish('0 home')
        print "go home robot you are drunk..."
        exit(1)

    def load_keras_model(self, model_path):
        self.model = keras.models.load_model(model_path)
        print '-' * 30
        print "loaded LSTM model at %s" % model_path

    def load_database(self, landmark_path):
        self.df_landmark = pd.read_csv(landmark_path)
        print '-' * 30
        print 'loaded landmark_path at: %s' % landmark_path   
        print self.df_landmark['name'].to_string(index=False)     
        self.requested_order = 1
        
    def mm_msg_callback(self, msg):
        # decode the msg into the format that we want
        raw = self.dsp.decode_packet(msg.data)

        # fill data frame and get relevant data points
        self.dsp.df.fillDataFrame(raw)
        x, Name2Index, Index2Name = self.dsp.df.GetRelevantData()
        # for i in range(x.shape[1]):
            # print Index2Name[i], x[0, i]
        
        # all the preprocessing steps
        # smooth
        if self.zprev is None:
            z = x
        else:        
            z = self.dsp.expSmooth(x, self.zprev, alpha=0.2)
        self.zprev = z

        if 0:
            # to_do, load the means and stds for normalization
            print z.shape
            self.mu = [0 for _ in z.shape[1]]
            self.sigma = [1 for _ in z.shape[1]]
            z = self.dsp.normalize(z, self.mu, self.sigma)

        # get feature
        f, channelName = self.dsp.calcFeature(z, Index2Name)
        # print channelName
        self.feat_dim = len(channelName)
        # print f.shape

        # keep a buffer of length self.z_buf_size
        if len(self.f_buf) >= self.f_buf_size:
            del self.f_buf[0]
        self.f_buf.append(f)

    def intent_pred(self):
        # to_do
        # do something based on self.z_buf
        # it contains previous points ([0] is the oldest, [-1] is the latest)
        if len(self.f_buf) != self.f_buf_size:
            print "buffer not fully inited, return unknown (-1)"
            return -1, -1

        self.model.reset_states()
        # x_test = np.random.random((1, time_steps, data_dim))
        x_test = np.array(self.f_buf).reshape(1, self.f_buf_size, self.feat_dim)
        # fake it
        x_test = x_test[:, :, :16] # !!! fake it
        # print 'x_test.shape', x_test.shape
        y_test_pred_prob = self.model.predict(x_test, batch_size=1)
        # print 'y_test_pred_prob\n', y_test_pred_prob
        # print 'y_test_pred_prob.shape', y_test_pred_prob.shape
        y_test_pred = np.argmax(y_test_pred_prob, 2)[0]
        # print 'y_test_pred', y_test_pred
        intent = y_test_pred[-1] # get predict of last timestamp
        intent_confi = np.abs(y_test_pred_prob[0][-1][0]- y_test_pred_prob[0][-1][1])
        print "predicted intent is %i with confidence %.2f" %(intent, intent_confi)
        # intent_confi \in [0, 1), larger is better
        return intent, intent_confi

    def item_pred(self):
        # to_do
        # maybe keep an index for the current item to be used
        # and then update the index whenever the human has finished the current tool
        item = 'R'
        item_confi = 1
        return item, item_confi

    def teleop(self):
        print "\n=== task object status==="
        c = raw_input('Enter step for the next task object: ')
        if c == 'menu':
            print 'menu: ', self.df_landmark['name'].values
        if c.upper() in ['Q', 'QUIT']:
            print "done, exit program"
            self.grace_exit('done')
        return c

    def decode_item(self, item):
        if item not in self.df_landmark['name'].values:
            print "!!! cannot locate landmark for user input: %s" % item
            return 'none'
        item_msg = str(self.item_id) + ' ' + item
        print '#%i => you requested [%s]' % (self.item_id, item)
        print '========================='
        self.item_id += 1    
        return item_msg

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.mode == 'early':
                intent, intent_confi = self.intent_pred()
                item, item_confi = self.item_pred()
            elif self.mode == 'speech':
                # to_do
                # based on the sphinx output
                # get the intent and item
                intent = 1
                item = 'R'
            elif self.mode == 'teleop':
                item = self.teleop()
                intent = 1

            if 1:
                # always publish intent
                # to_do, fix later
                self.intent_pub.publish(intent)
            
            item_msg = self.decode_item(item)
            if item_msg != 'none':
                self.item_pub.publish(item_msg)

            rate.sleep()
            # print "predicted intent %.2f, item %s" % (intent, item)

if __name__ == '__main__':
    ttp = TTP(mode='teleop')
    ttp.load_keras_model(model_path = '/home/tzhou/Workspace/catkin_ws/src/ttp/model/my_model.h5')
    ttp.load_database(landmark_path = '/home/tzhou/Workspace/catkin_ws/src/ttp/data/landmark_chair.csv')
    ttp.run()
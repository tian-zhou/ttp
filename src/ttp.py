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
from datetime import datetime

class TTP:
    def __init__(self):
        # init node for mm_bridge
        rospy.init_node('ttp_node', anonymous=True)

        # init the subscriber
        rospy.Subscriber("mm_bridge_output", String, self.mm_msg_callback)

        # init the publisher for command
        self.cmd_pub = rospy.Publisher('cmd_pred', String, queue_size=10)
        self.item_id = 0
        self.rail_id = 1
        self.thumbtack_id = 1
        self.cmd_history = []
        
        # init the signal processing unit
        self.dsp = DSP()
        self.z_buf_size = 20
        self.z_buf = []
        
    def grace_exit(self, exit_msg):
        print "!" * 20
        print '(grace?) exit with message:'
        print '---\n', exit_msg, '\n---'
        # self.cmd_pub.publish('1,home,1.00,2018_04_30_19_49_32_983000, NA')
        # print "go home robot you are drunk..."
        print "cmd history:"
        for c in self.cmd_history:
            print c
        exit(1)

    def load_keras_model(self, model_path):
        self.model = keras.models.load_model(model_path)
        print '-' * 30
        print "loaded LSTM model at %s" % model_path

    def load_landmarks(self, landmark_path):
        self.df_landmark = pd.read_csv(landmark_path)
        print '-' * 30
        print 'loaded landmark_path at: %s' % landmark_path   
        print self.df_landmark['name'].to_string(index=False)     
        self.requested_order = 1
        
    def mm_msg_callback(self, msg):
        # decode the msg into the format that we want
        df, df0 = self.dsp.decode_packet(msg.data)
        
        x = df.loc[0].values.flatten() # shape (num_raw_columns,)

        # clip outlier
        self.dsp.clip_outlier(x) 

        # normalize
        self.dsp.scale(x, method='standard')

        # smooth
        z = self.dsp.expSmooth(x, self.z_buf[-1], alpha=0.2) if len(self.z_buf) else x

        # keep a buffer of length self.z_buf_size
        if len(self.z_buf) >= self.z_buf_size:
            del self.z_buf[0]
        self.z_buf.append(z)

        # encode features (LoG, Gabor etc)
        en_buf, encode_feat_names = self.dsp.encode_features(self.z_buf, 
                                                self.dsp.good_feat_names)

        # select features
        self.select_feat_names = list(pd.read_excel('../model/feature_info.xlsx', 
                                sheetname='fs_identity')['Name'])
        self.f_buf = self.dsp.select_features(en_buf, encode_feat_names, self.select_feat_names)
        self.feat_dim = len(self.select_feat_names)

    def intent_pred(self):
        # to_do
        # do something based on self.f_buf
        # it contains previous points ([0] is the oldest, [-1] is the latest)
        if len(self.z_buf) != self.z_buf_size:
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
        item = 'seat'
        item_confi = 1
        return item, item_confi

    def make_command_msg(self, item, intent, comment='NA'):
        # msg has the following format
        # step | task/object name | hand over ready? | timestamp | comment
        # e.g. 
        # 1,tape,0,2018_04_30_19_49_32_983,NA
        # 1,tape,1,2018_04_30_19_50_03_323,NA
        # 2,marker,1,2018_04_30_19_53_03_323,This is a comment 
        if item not in self.df_landmark['name'].values and item not in ['rail', 'thumbtack']:
            print "invalid command: [%s]" % item
            return ''

        msg = '%s,' % self.item_id
        if item == 'rail':
            msg += '%s%i,' % (item, self.rail_id)
            self.rail_id += (1 if intent == 1 else 0)
            if self.rail_id == 8:
                print "Run out of rails! Cannot pick!"
                return ''
        elif item == 'thumbtack':
            msg += '%s%i,' % (item, self.thumbtack_id)
            self.thumbtack_id += (1 if intent == 1 else 0)
            if self.thumbtack_id == 18:
                print "Run out of thumbtacks! Cannot pick!"
                return ''
        else:
            msg += '%s,' % item
        msg += '%.2f,' % intent
        msg += '%s,' % datetime.now().strftime('%Y_%m_%d_%H_%M_%S_%f')
        msg += '%s' % comment
        self.item_id += (1 if intent == 1 else 0)
        self.cmd_history.append(msg)
        return msg
        
    def run_tele(self):
        """
        in this case, for every entry of command, we publish one message
        and we are sure of the intent (always 1)
        """
        while not rospy.is_shutdown():
            item = raw_input('Enter name of next task object: ')
            if item == 'menu':
                print 'menu: ', self.df_landmark['name'].values
            if item == 'quit':
                self.grace_exit('quit program')
            msg = self.make_command_msg(item, intent=1, comment='teleop')
            if msg:
                self.cmd_pub.publish(msg)
        self.grace_exit('teleoperation finished...')
    
    def decode_word(self, word):
        """
        convert to lower case
        remove the beginning and ending spaces
        if the word is repeated, only return one
        e.g. 'seat seat' -> return 'seat'
        """
        item = word.lower().lstrip().rstrip()
        item_split = item.split()
        if len(set(item_split)) == 1:
            item = item_split[0]
        return item

    def run_speech(self):
        """
        in this case, we wait for speech command, and do necessary
        decoding to link it to our task. Everytime we got a command
        it is for sure that the human needs this object.
        We explicitly ask participant to utter the name of the object
        and not more than that
        """
        from pocketsphinx import LiveSpeech
        speech = LiveSpeech(
            verbose=False,
            sampling_rate=96000, # 48000
            buffer_size=4096, # 4096
            no_search=False,
            full_utt=False,
            hmm='/home/tzhou/Workspace/catkin_ws/src/ttp/model/sphinx/en-us-Win10', # 'en-us-Win10', 'en-us-dist-packages'
            lm='/home/tzhou/Workspace/catkin_ws/src/ttp/model/sphinx/7600.lm',
            dic='/home/tzhou/Workspace/catkin_ws/src/ttp/model/sphinx/7600.dic'
        )

        print 'listening...'
        thres = 0.1
        low_thres = 0.05
        for phrase in speech:
            if rospy.is_shutdown():
                break
            confi = phrase.confidence()
            item = self.decode_word(phrase.hypothesis())
            if confi > low_thres and confi < thres:
                print 'low-confi  word [%s] with confi [%.2f]' % (item, confi)
            if confi < thres:
                continue
            print 'recognized word [%s] with confi [%.2f]' % (item, confi)
            msg = self.make_command_msg(item, intent=1, comment='speech')
            if msg:
                self.cmd_pub.publish(msg)
                print 'publish message: [%s]' % msg
                print '========================='
        
        self.grace_exit('speech control finished...')

    def run_early(self):
        """
        since the intent and the item prediction happens non-stop
        we need to think about when to publish this new msg
        we don't always publish them, but only publish when necessary
        need to think about it        
        """
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            item, item_confi = self.item_pred()
            intent, intent_confi = self.intent_pred()
            if "something happened here, we continue to predict":
                continue
            msg = self.make_command_msg(item, intent=intent, comment='early prediction')
            self.cmd_pub.publish(msg)
            rate.sleep()
        self.grace_exit('early prediction finished...')

    def run(self, mode):
        if mode not in ['early', 'speech', 'teleop']:
            rospy.logerr('unrecognized TTP mode %s' % mode)
        if mode == 'teleop':
            self.run_tele()
        elif mode == 'speech':
            self.run_speech()
        elif mode == 'early':
            self.run_early()

def main():
    ttp = TTP()
    ttp.load_keras_model(model_path = '/home/tzhou/Workspace/catkin_ws/src/ttp/model/my_model.h5')
    ttp.load_landmarks(landmark_path = '/home/tzhou/Workspace/catkin_ws/src/ttp/data/joint_cart.csv')
    ttp.run(mode='speech') # ['teleop', 'speech', 'early']

if __name__ == '__main__':
    main()
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
    ttp = TTP(mode='teleop')
    ttp.run()

Author:
    Tian Zhou (leochou1991@gmail.com)

Date:
    Oct, 9, 2017

License:
    GNU General Public License v3
"""

import sys
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float32, String, Float32MultiArray
from dsp import DSP
import keras 
from keras.utils import plot_model
import numpy as np
import readchar
import pandas as pd
from datetime import datetime
from time import time
from keras.preprocessing.sequence import pad_sequences

class TTP:
    def __init__(self, model_path, fs_path, landmark_path, task_path):
        self.load_keras_model(model_path, fs_path)
        self.load_landmarks(landmark_path)
        self.load_task_order(task_path)

        # init node for mm_bridge
        rospy.init_node('ttp_node', anonymous=True)

        # init the publisher for command
        self.cmd_pub = rospy.Publisher('cmd_pred', String, queue_size=10)
        self.item_id = 0
        self.rail_id = 1
        self.thumbtack_id = 1
        self.cmd_request_history = []
        
        # init the signal processing unit
        self.dsp = DSP()
        self.z_buf = []
        self.z_buf_size = 5 # min 5 timestamps for feature encoding
        self.f_buf = []
        self.f_buf_size = 120 # 20 Hz * 6 seconds

        # record firing time 
        self.last_fire_time = time()

        # init the subscriber at the end when all init is finished
        rospy.Subscriber("mm_bridge_output", String, self.mm_msg_callback)
                
        
    def grace_exit(self, exit_msg, gohome):
        print "!" * 20
        print '(grace?) exit with message:'
        print '---\n', exit_msg, '\n---'
        if gohome:
            self.cmd_pub.publish('0|home|1.00|15:57:42:87|keyboard')
            print "go home robot you are drunk..."
        print "cmd history:"
        for c in self.cmd_request_history:
            print c
        exit(1)

    def load_keras_model(self, model_path, fs_path):
        self.model = keras.models.load_model(model_path)
        print '-' * 30
        print "loaded LSTM model at %s" % model_path
        print (self.model.summary())
        # plot_model(self.model, show_shapes=True)
        self.select_feat_names = list(pd.read_csv(fs_path, header=None)[0].values)
        self.feat_dim = len(self.select_feat_names)
        print "loaded Feature Selection at %s" % fs_path
        print "Selected features\n", self.select_feat_names

    def load_landmarks(self, landmark_path):
        self.df_landmark = pd.read_csv(landmark_path)
        print '-' * 30
        print 'loaded landmark_path at: %s' % landmark_path   
        print self.df_landmark['name'].to_string(index=False)     
        
    def load_task_order(self, task_path):
        self.df_task = pd.read_csv(task_path)
        self.df_task['status'] = 'to_do'
        print '-' * 30
        print 'loaded task_path at: %s' % task_path
        # print self.df_task.info()
        # print self.df_task.head()
        # exit()
        
    def mm_msg_callback(self, msg):
        # decode the msg into the format that we want
        df = self.dsp.decode_packet(msg.data)
        if df is None:
            print 'invalid mm msg packet, unequal pac length'
            return

        x = df.loc[0].values.flatten() # shape (num_raw_columns,)

        self.dsp.clip_outlier(x) 
        self.dsp.scale(x, method='standard')
        z = self.dsp.expSmooth(x, self.z_buf[-1], alpha=0.2) if len(self.z_buf) != 0 else x
        self.z_buf.append(z)

        if len(self.z_buf) < self.z_buf_size:
            return 
        elif len(self.z_buf) > self.z_buf_size:
            del self.z_buf[0]
        assert len(self.z_buf) == self.z_buf_size

        # encode features (LoG, Gabor etc)
        en, encode_feat_names = self.dsp.encode_features_online(self.z_buf)        
        f = self.dsp.select_features(en, encode_feat_names, self.select_feat_names)
        self.f_buf.append(f)
        if len(self.f_buf) > self.f_buf_size:
            del self.f_buf[0]

    def intent_pred(self):
        if len(self.f_buf) < self.f_buf_size:
            return 0
        # the intent_pred is very fast (takes about 0.01 second)    
        x_test = np.array(self.f_buf).reshape(1, self.f_buf_size, self.feat_dim)
        y_test_pred_prob = self.model.predict(x_test, batch_size=1)[0]
        intent = y_test_pred_prob[1]
        return intent

    def item_pred(self):
        # first, let us fix the order to dictate the steps
        to_do_tasks = self.df_task.loc[self.df_task['status'] == 'to_do']
        if len(to_do_tasks) == 0:
            print "all tasks finished..."
            return 'done'
        item = to_do_tasks.iloc[0]['part_name']
        return item

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

        msg = '%s|' % self.item_id
        if item == 'rail':
            msg += '%s%i|' % (item, self.rail_id)
            self.rail_id += 1
            if self.rail_id == 8:
                print "Run out of rails! Cannot pick!"
                return ''
        elif item == 'thumbtack':
            msg += '%s%i|' % (item, self.thumbtack_id)
            self.thumbtack_id += 1
            if self.thumbtack_id == 18:
                print "Run out of thumbtacks! Cannot pick!"
                return ''
        else:
            msg += '%s|' % item
        msg += '%.2f|' % intent
        msg += '%s|' % datetime.now().strftime('%H:%M:%S:%f')[:-4] # %Y_%m_%d_%H_%M_%S_%f
        msg += '%s' % comment
        self.item_id += 1
        self.cmd_request_history.append(msg)
        return msg
        
    def run_keyboard(self):
        """
        in this case, for every entry of command, we publish one message
        and we are sure of the intent (always 1)
        """
        while not rospy.is_shutdown():
            item = raw_input('Enter name of next task object: ')
            if item == 'menu':
                print 'menu: ', self.df_landmark['name'].values
            if item == 'quit':
                self.grace_exit('quit program', gohome=False)
            
            # map some shortcut names to full name
            mapping = {}
            mapping['left'] = 'left stile'
            mapping['right'] = 'right stile'
            mapping['notes'] = 'sticky notes'
            mapping['cut'] = 'scissors'
            for i in range(1, 17):
                mapping['tack'+str(i)] = 'thumbtack'+str(i)
            if item in mapping:
                print "translate command [%s] => [%s]" % (item, mapping[item])
                item = mapping[item]
            msg = self.make_command_msg(item, intent=1, comment='keyboard')
            if msg:
                self.cmd_pub.publish(msg)
        self.grace_exit('keyboard control finished...', gohome=False)
    
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
        print "thres: %.2f" % thres
        print "low_thres: %.2f" % low_thres
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
                # self.cmd_pub.publish(msg)
                print '(no) publish message: [%s]' % msg
                print '========================='
        self.grace_exit('speech control finished...', gohome=False)

    def check_fire(self, intent_history, time_history, confi_thres, active_duration_thres, 
                    fire_interval_thres):
        """
        === used, looks good overall!!!! 
        method 1: find the duration where confi exceeds a confidence threshold, 
        if this continuous duration is longer than a window threshold, we fire.
            problem: have to work together with the silencing scheme, otherwise
            we will see multiple firing for the same turn request action

        === not used 
        method 2: find local maximum, and if the value at local maximum exceeds
        a confidence threshold, we fire, and start silencing.
            problem: if the confi keeps increasing and never reaches local maximum,
            we cannot fire.
        """
        N = len(intent_history)
        hz = 20.0
        for i in range(N-1, max(-1, N-100), -1):
            if intent_history[i] > confi_thres:
                if time_history[-1] - time_history[i] >= active_duration_thres:
                    if time_history[-1] - self.last_fire_time > fire_interval_thres:
                        if 0:
                            plt.plot(np.arange(N)/hz, confi_thres*np.ones(N), 'g', label='threshold')
                            plt.plot(np.arange(0, i)/hz, intent_history[:i], 'b', label = 'inactive')
                            plt.plot(np.arange(i, N)/hz, intent_history[i:], 'r', label = 'active')
                            plt.xlabel('time (in seconds)')
                            plt.ylabel('intent history')
                            plt.legend(loc='best')
                            plt.show()
                        self.last_fire_time = time()
                        # print "fired at time: ", datetime.now()
                        print "intent: [%.2f], fire!!!" % intent_history[-1]
                        return True
                    else:
                        remain_time = fire_interval_thres - (time() - self.last_fire_time)
                        print "intent: [%.2f], wait for silence period to pass, %.2f sec left" % (intent_history[-1], remain_time)
                        return False
            else:
                break
        print "intent: [%.2f], not high enough..." % intent_history[-1]
        return False

    def run_tt(self):
        """
        since the intent and the item prediction happens non-stop
        we need to think about when to publish this new msg
        we don't always publish them, but only publish when necessary
        need to think about it        
        """
        rate = rospy.Rate(20)
        intent_history = []
        time_history = []
        time_past = []
        print "waiting for feature buffer to be fully inited (need 120 steps)..."
        f = plt.figure()
        ax = f.gca()
        f.show()
        ax.set_xlabel('past time in seconds')
        ax.set_ylabel('intent history')
        ax.set_ylim([0, 1])
        t0 = time()

        # === some parameters ===
        fire_interval_thres = 10 # 12 to be conservative, make it 10 to be faster. For subject 1,2,3 it is 12
        confi_thres = 0.4 # 0.5 to be conservative, notice that 0.5 does not work well with some subjects. Use 0.4!
        active_duration_thres = 0.3 # 0.5 to be conservative
        while not rospy.is_shutdown():
            item = self.item_pred()
            if item == 'done':
                break
            intent = self.intent_pred()
            # rate.sleep()
            intent_history.append(intent)
            time_history.append(time())
            time_past.append(time() - t0)
            if len(intent_history) < 10:
                continue

            # do some plot
            ax.plot(time_past[-2:], intent_history[-2:], 'r-', linewidth=5)
            ax.plot(time_past[-2:], [confi_thres, confi_thres], 'b-', linewidth=2)
            xlim_min = 0 if len(time_past) < 100 else time_past[-1] - 10 # show 10 seconds
            ax.set_xlim([xlim_min, time_past[-1]])
            f.canvas.draw()
            if self.check_fire(intent_history, time_history, confi_thres, active_duration_thres, 
                        fire_interval_thres): 
                # 12 is good in practice, 6 is good in debug (no robot motion) 
                # confi_thres in range [0.5, 0.99], the larger, the more misses but more accurate
                # active_duration_thres = 0.5 # high value for > 0.5 seconds
                # fire_interval_thres = 12 # 12 seconds: 7 sec for pickup/delivery, and 5 sec for operation
                msg = self.make_command_msg(item, intent=1, comment='early tt prediction')
                self.df_task.loc[self.df_task['part_name'] == item, 'status'] = 'done'
                # print self.df_task
                self.cmd_pub.publish(msg)
                print 'publish message: [%s]' % msg
                print '========================='
        self.grace_exit('tt prediction finished...', gohome=False)
        
    def run(self, mode):
        if mode not in ['tt', 'speech', 'keyboard']:
            rospy.logerr('unrecognized TTP mode %s' % mode)
        if mode == 'keyboard':
            self.run_keyboard()
        elif mode == 'speech':
            self.run_speech()
        elif mode == 'tt':
            self.run_tt()

def main():
    ttp = TTP(model_path = '/home/tzhou/Workspace/turn_taking/chair_github/model/lstm/LSTM_50.h5', 
                 fs_path = '/home/tzhou/Workspace/turn_taking/chair_github/model/features/one_model_select_feat_names.csv',
           landmark_path = '/home/tzhou/Workspace/catkin_ws/src/ttp/data/joint_cart.csv',
               task_path = '/home/tzhou/Workspace/catkin_ws/src/ttp/data/fix_chair_order.csv')
    mode = sys.argv[1].lower()
    print "argument mode: [%s]" % mode
    assert(mode in ['keyboard', 'speech', 'tt'])
    ttp.run(mode=mode) # ['keyboard', 'speech', 'tt']

if __name__ == '__main__':
    main()
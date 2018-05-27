#!/usr/bin/env python

import numpy as np
import time
from datetime import datetime, timedelta
import matplotlib.pyplot as plt
import pandas as pd

def epoc2time(epoch):
    return datetime.fromtimestamp(epoch).strftime('%Y_%m_%d_%H:%M:%S.%f')[:-4]

def time2epoc(stime):
    return time.mktime(datetime.timetuple(stime))

def load_timestamp():
    timestamp = {}
    timestamp['1_1'] = {'begin_time': "2018_05_22_10_49_30", 'end_time': "2018_05_22_10_57_55", 'offset': -11}
    timestamp['1_2'] = {'begin_time': "2018_05_22_11_04_39", 'end_time': "2018_05_22_11_11_31", 'offset': -11}
    timestamp['1_3'] = {'begin_time': "2018_05_22_11_26_11", 'end_time': "2018_05_22_11_32_24", 'offset': -11}
    timestamp['1_4'] = {'begin_time': "2018_05_22_11_35_53", 'end_time': "2018_05_22_11_41_56", 'offset': -11}
    timestamp['2_1'] = {'begin_time': "2018_05_22_14_52_09", 'end_time': "2018_05_22_14_57_11", 'offset': -11}
    timestamp['2_2'] = {'begin_time': "2018_05_22_15_07_41", 'end_time': "2018_05_22_15_11_51", 'offset': -11}
    timestamp['2_3'] = {'begin_time': "2018_05_22_15_26_53", 'end_time': "2018_05_22_15_30_18", 'offset': -11}
    timestamp['2_4'] = {'begin_time': "2018_05_22_15_33_00", 'end_time': "2018_05_22_15_36_37", 'offset': -11}
    timestamp['3_1'] = {'begin_time': "2018_05_22_16_24_07", 'end_time': "2018_05_22_16_31_29", 'offset': -11}
    timestamp['3_2'] = {'begin_time': "2018_05_22_16_35_00", 'end_time': "2018_05_22_16_42_18", 'offset': -11}
    timestamp['3_3'] = {'begin_time': "2018_05_22_16_51_40", 'end_time': "2018_05_22_16_59_48", 'offset': -11}
    timestamp['3_4'] = {'begin_time': "2018_05_22_17_03_52", 'end_time': "2018_05_22_17_11_16", 'offset': -11}
    timestamp['4_1'] = {'begin_time': "2018_05_24_11_40_54", 'end_time': "2018_05_24_11_46_32", 'offset': 0}
    timestamp['4_2'] = {'begin_time': "2018_05_24_11_50_05", 'end_time': "2018_05_24_11_55_53", 'offset': 0}
    timestamp['4_3'] = {'begin_time': "2018_05_24_12_02_23", 'end_time': "2018_05_24_12_09_01", 'offset': 0}
    timestamp['4_4'] = {'begin_time': "2018_05_24_12_12_08", 'end_time': "2018_05_24_12_18_16", 'offset': 0}
    timestamp['5_1'] = {'begin_time': "2018_05_24_13_24_04", 'end_time': "2018_05_24_13_30_40", 'offset': 0}
    timestamp['5_2'] = {'begin_time': "2018_05_24_13_34_20", 'end_time': "2018_05_24_13_40_13", 'offset': 0}
    timestamp['5_3'] = {'begin_time': "2018_05_24_13_46_49", 'end_time': "2018_05_24_13_52_28", 'offset': 0}
    timestamp['5_4'] = {'begin_time': "2018_05_24_13_55_47", 'end_time': "2018_05_24_14_01_04", 'offset': 0}
    timestamp['6_1'] = {'begin_time': "2018_05_24_17_03_23", 'end_time': "2018_05_24_17_08_35", 'offset': 0}
    timestamp['6_2'] = {'begin_time': "2018_05_24_17_12_24", 'end_time': "2018_05_24_17_18_09", 'offset': 0}
    timestamp['6_3'] = {'begin_time': "2018_05_24_17_22_17", 'end_time': "2018_05_24_17_25_35", 'offset': 0} # special, since recording stopped in the middle. change 2018_05_24_17_28_44 (true from annotation) to 2018_05_24_17_25_35 (rosbag stop time)
    timestamp['6_4'] = {'begin_time': "2018_05_24_17_31_38", 'end_time': "2018_05_24_17_36_53", 'offset': 0}
    return timestamp

def calc_robot_idle_time(print_flag, plot_flag):
    timestamp = load_timestamp()
    for trial in sorted(timestamp.keys()):
        # trial = '1_1'
        # trial = raw_input('Please enter trial name (e.g., 1_1): ')
        # print "Working on trial: [%s]" % trial    
        df_joint_vel = pd.read_csv('../anno/%s_jvel.csv' % trial)
        # print "df_joint_vel loaded from ../anno/%s_jvel.csv" % trial
        UNIX_time = df_joint_vel['UNIX_time'].values
        joint_vel = df_joint_vel['joint_vel'].values
        
        # print "correct annotation time by [%.2f] seconds offset" % timestamp[trial]['offset']
        begin_time = datetime.strptime(timestamp[trial]['begin_time'], '%Y_%m_%d_%H_%M_%S')
        begin_time += timedelta(seconds=timestamp[trial]['offset'])
        begin_time_epoch = time2epoc(begin_time)
        end_time = datetime.strptime(timestamp[trial]['end_time'], '%Y_%m_%d_%H_%M_%S')
        end_time += timedelta(seconds=timestamp[trial]['offset'])
        end_time_epoch = time2epoc(end_time)

        if print_flag:
            print '---'
            print "begin_time from rosbag: ", epoc2time(UNIX_time[0])
            print 'begin_time from annotation: ', begin_time

            print '---'
            print "end_time from rosbag: ", epoc2time(UNIX_time[-1])
            print 'end_time from annotation: ', end_time
            
            print '---'
            print 'epoch from rosbag:', UNIX_time[0]
            print 'epoch from begin_time in annotation:', begin_time_epoch

            print '---'
            print 'epoch from rosbag:', UNIX_time[-1]    
            print 'epoch from end_time in annotation:', end_time_epoch

        # create the robot active bar
        # base freq = 50 Hz, so step = 0.02 
        time_step = np.arange(begin_time_epoch, end_time_epoch, 0.01) 
        joint_vel_interp = np.interp(time_step, UNIX_time, joint_vel)
        tol = 0.02
        robot_active = (joint_vel_interp > tol).astype(int)
        robot_idle_ratio = np.sum(robot_active == 0) / float(len(robot_active))
        print "trial [%s] => robot_idle_ratio: %.3f" % (trial, robot_idle_ratio)

        # do some plots
        if plot_flag:
            t0 = UNIX_time[0]
            plt.figure()
            plt.subplot(311)
            plt.plot(UNIX_time - t0, joint_vel, 'rd-', linewidth=1, label='joint_vel')
            plt.subplot(312)
            plt.plot(time_step - t0, joint_vel_interp, 'b-x', linewidth=1, label='joint_vel_interp')
            plt.subplot(313)
            plt.plot(time_step - t0, robot_active, 'g-s', linewidth=1, label='robot_active')
            plt.xlabel('elapsed time since start')
            plt.ylabel('joint velocity')
            plt.legend(loc='best')
            plt.show()


if __name__ == '__main__':
    calc_robot_idle_time(print_flag=0, plot_flag=0)
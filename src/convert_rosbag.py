#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import time
from datetime import datetime
import matplotlib.pyplot as plt
import pandas as pd
from dsp import DSP


class RECORD_ROBOT_JOINT_VEL:
    def __init__(self):
        rospy.init_node('record_robot_joint_vel')
        rospy.Subscriber("joint_states", JointState, self.joint_states_msgcb)
        self.joint_pose = None
        self.prev_joint = None
        self.joint_norm_tol = 0.05
        self.joint_vel_buf = []
        self.time_buf = []
        self.df_joint_vel = pd.DataFrame(columns = ['UNIX_time', 'joint_vel'])

    def joint_states_msgcb(self, msg):
        # print '---'
        self.joint_pose = msg.position        
        sec = msg.header.stamp.secs
        ns = msg.header.stamp.nsecs
        UNIX_time = sec + ns * 1e-9
        # full_time = UNIX_time # time.localtime(UNIX_time)
        # print full_time
        # print time.strftime('%Y-%m-%d %H:%M:%S', full_time)
        # print self.joint_pose
        
        if self.prev_joint is None:
            self.prev_joint = self.joint_pose

        offset = np.subtract(self.joint_pose, self.prev_joint)
        joint_vel = np.linalg.norm(offset)
        self.joint_vel_buf.append(joint_vel)
        self.time_buf.append(UNIX_time)
        self.prev_joint = self.joint_pose

        row = {}
        row['UNIX_time'] = UNIX_time
        row['joint_vel'] = joint_vel
        self.df_joint_vel = self.df_joint_vel.append(row, ignore_index=True)

    def run(self):
        name = raw_input('Please enter trial name (e.g., 1_1): ')
        print "Working on rosbag for %s" % name
        
        while not rospy.is_shutdown():
            pass
        self.df_joint_vel.to_csv('../anno/%s_jvel.csv' % name, index=False)
        print "df_joint_vel.csv saved ../anno/%s_jvel.csv" % name

        # f = plt.figure()
        # ax = f.gca()
        # f.show()
        # ax.set_xlabel('time')
        # ax.set_ylabel('joint_vel')
        # # ax.set_ylim([0, 1])
        # while not rospy.is_shutdown():
        #     if len(self.time_buf) < 3:
        #         continue

        #     # do some plot
        #     ax.plot(self.time_buf[-2:], self.joint_vel_buf[-2:], 'r-d', linewidth=2)
        #     ax.plot(self.time_buf[-2:], [self.joint_norm_tol, self.joint_norm_tol], 'b-.', linewidth=2)
        #     # ax.set_xlim([self.time_buf[-1] - 10, self.time_buf[-1]])
        #     f.canvas.draw()


def main():
    converter = RECORD_ROBOT_JOINT_VEL()
    converter.run()

if __name__ == '__main__':
    main()
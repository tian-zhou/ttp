#!/usr/bin/env python

"""
Description:
    the necessary signal processing code to smooth, scale and extract features
    from the raw socket packet

Sample usage:
    xxx

Author:
    Tian Zhou (leochou1991@gmail.com)

Date:
    Oct, 15, 2017 - v1, initial setup
    4/17/2018 - v2, removed the previous dataframe class, and used pandas instead

License:
    GNU General Public License v3
"""

import numpy as np
import pandas as pd 
import matplotlib.pylab as plt 
from FilterBank import FilterBank


class DSP:
    def __init__(self):
        self.folder = '/home/tzhou/Workspace/catkin_ws/src/ttp'
        feat_names = list(pd.read_excel(self.folder + '/model/feature_info.xlsx', sheetname='descriptor')['Name'])
        self.feat_names = feat_names

        self.meta = pd.read_excel(self.folder + '/model/feature_info.xlsx', sheetname='selected')
        # df_feature_meta has columns for clipping and normalization
        # Name | mean | std | min | max | lower_fence | upper_fence
        self.good_feat_names = list(self.meta['Name'])
        
    def decode_packet(self, pac):
        # for some reason, sometimes the pac will contain some weird chars at the end of the buffer
        # we should remove them
        # solution -> find first abcd1234, and trim all stuff after that
        pac = pac[:pac.find('abcd1234')]
        pac = pac.split()
        pac[0] = pac[0][-23:] # just took the time out
        # print pac 
        # print '\n\n'
        # print "len(pac)", len(pac)
        # print "len(feat_names)", len(self.feat_names)
        df0 = pd.DataFrame([pac], columns = self.feat_names)
        # print df0
        # print df0['realtime_str']
        
        df0.insert(1, 'realtime_obj', pd.to_datetime(df0['realtime_str'], format='%Y_%m_%d_%H_%M_%S_%f'))
        df0 = df0.drop('realtime_str', 1)

        # only read the relevant columns
        df = df0[self.good_feat_names]
        df = df.apply(pd.to_numeric)
        return df, df0

    def clip_outlier(self, x):
        """
        x is of shape: (feature_dim,)
        use the lower_fence and upper_fence column of self.meta
        the value in the index column can be used to query element in x
        # meta: Name | mean | std | min | max | lower_fence | upper_fence
        """
        x[12] = max(-0.4, x[12]) # 'Kinect_JointType_HandRight_X'
        x[13] = max(-0.6, x[13]) # 'Kinect_JointType_HandRight_Y'
        x[14] = min( 1.5, x[14]) # 'Kinect_JointType_HandRight_Z'
        
    def scale(self, x, method):
        """
        method \in {'standard', 'minmax'}
        # meta: Name | mean | std | min | max | lower_fence | upper_fence
        """
        if method == 'standard':
            subtract = self.meta['mean'].values
            divider = self.meta['std'].values
        elif method == 'minmax':
            subtract = self.meta['min'].values
            divider = self.meta['max'].values - self.meta['min'].values
        x = (x - subtract)/divider
        return x

    def expSmooth(self, x, zprev, alpha):
        """
        Exponentially Weighted Moving Average (EWMA)
        z_t = alpha * x_t + (1-alpha) * z_{t-1}, z_0 = x_0
        alpha of 0.2 is good
        """
        assert(x.shape == zprev.shape)
        z = alpha * x + (1-alpha) * zprev
        return z

    def encode_features(self, z_buf, feat_names):
        """
        z.buf.shape: (T, num_features)
        the 1st dimension is the temporal dimension
        the 2nd dimension is the feature dimension
        apply filters on each column of z_buf to create encoded versions
        return en_buf, encode_feat_names
        en_buf has shape: (T, num_features * num_filters)
            ordered by [feature0_filter0, feature0_filter1, ...]
        encode_feat_names is the new name for the features
        """
        z_buf = np.array(z_buf)
        assert(z_buf.shape[1] == len(feat_names))
        fb = FilterBank(full = False)
        xf = fb.applyFilter(z_buf, filterName = 'all', GaussBlur = False)
        en_buf, encode_feat_names = fb.convertFilteredToMatrix(xf, feat_names)
        return en_buf, encode_feat_names

    def select_features(self, en_buf, encode_feat_names, select_feat_names):
        """
        perform feature selection, based on a given selected feature list
        """
        col_index = []
        encode_feat_names = np.array(encode_feat_names)
        for feat_name in select_feat_names:
            col_index.append(np.nonzero(encode_feat_names == feat_name)[0][0])
        f_buf = en_buf[:, col_index]
        return f_buf

def pprint(info, x, feat_names):
    print "\n=== %s ===" % info
    for i in range(len(x)):
        print "%s: %.3f" % (feat_names[i], x[i])

def main():
    dsp = DSP()
    pac = '2018_03_29_14_19_12_368 4.65738 0 0 0 0 -64 65535 0 1 -1 0 -1 0 -2 0 -0.523281 -1.2583 1.65986 0.928223 -0.160156 0.26709 -0.0458149 0.0479966 -0.0730857 4.8 0 0 0 0 0 0 0 0 0 0 1184 1298 182 294 0 20 -41 1 1 1 1 1 1 1 1 0.23554 -0.405567 1.35144 0.224475 -0.052628 1.35558 0.20957 0.28503 1.33762 0.244845 0.39729 1.36231 0.0891571 0.148112 1.21112 -0.0951321 0.0361299 1.15243 -0.19088 0.171554 1.02525 -0.162632 0.235798 1.04991 0.333316 0.0966397 1.35184 0.329768 -0.127406 1.23552 0.204463 -0.358911 1.14634 0.157254 -0.42666 1.12969 0.174056 -0.393821 1.2978 0.323878 -0.33994 0.84699 0.445406 -0.302237 0.505271 0.480275 -0.276557 0.37645 0.284239 -0.39485 1.33062 0.340514 -0.503678 0.991976 0.430776 -0.673479 0.542208 0.468143 -0.646726 0.420244 0.213763 0.203039 1.34493 -0.0943421 0.295496 1.09774 -0.125173 0.208125 1.01778 0.113704 -0.497916 1.10682 0.246498 -0.440854 1.15753 0 0 0.0322037 0.213941 0 0 128 2890.77 6362.05 2960.51 6337.44 7654.36 7041.03 4071.79 5099.49 6277.95 6703.59 1471.79 2925.64 3505.13 7306.67 1573 1963 4.902 0 0 0 0 0.652403 0 0.294184 0 1 abcd1234'

    # decode the msg into the format that we want
    df, df0 = dsp.decode_packet(pac)
    feat_names = dsp.good_feat_names
    x = df.loc[0].values.flatten() # shape (num_raw_columns,)
    
    # clip outlier
    # print 'x before clip outlier \n', x
    dsp.clip_outlier(x) 
    # print 'x after clip outlier \n', x
    
    # normalize
    # print 'x before scale \n', x
    x = dsp.scale(x, method='standard')
    # print 'x after scale \n', x
    
    # smooth
    z = dsp.expSmooth(x, 10*np.random.rand(x.shape[0]), alpha=0.2)
    
    # fake it, duplicate z for 20 rows
    z = z.reshape(1,-1)
    z_buf = np.repeat(z, 20, axis=0)
    
    # encode features (LoG, Gabor etc)
    en_buf, encode_feat_names = dsp.encode_features(z_buf, feat_names)
    
    # select features
    select_feat_names = list(pd.read_excel(dsp.folder + '/model/feature_info.xlsx', 
                                sheetname='fs_identity')['Name'])
    f_buf = dsp.select_features(en_buf, encode_feat_names, select_feat_names)
    
if __name__ == '__main__':
    main()
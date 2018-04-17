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
    Oct, 15, 2017

License:
    GNU General Public License v3
"""

import numpy as np
import matplotlib.pyplot as plt


class myoDataFrame():
    # store myo data
    def __init__(self):
        self.d = dict()
        self.X = []


class kinectDataFrame():
    # store kinect data
    def __init__(self):
        self.d = dict()
        self.X = []


class epocDataFrame():
    # store epoc data
    def __init__(self):
        self.d = dict()
        self.X = []


class dataFrame():
    # store all data into a frame
    def __init__(self):
        # Myo
        self.myo = myoDataFrame()

        # Kinect
        self.kinect = kinectDataFrame()

        # EPOC
        self.epoc = epocDataFrame()

    def fillHumanPose(self, X):
        """
        Description:
            fill the data frame with 1 frame of data

        Input:
            X: the input numpy array, of shape (1, numCols)

        """
        data = X

        # add a fake column in the beginning to make the index match
        data = np.hstack((np.zeros((len(data), 1)), data))

        # Kinect
        self.kinect.d['time'] = np.reshape(data[:, 25] - data[0, 25], (-1, 1)) # useless
        self.kinect.d['SpineBase'] = data[:, 51:54]
        self.kinect.d['SpineMid'] = data[:, 54:57]
        self.kinect.d['Neck'] = data[:, 57:60]
        self.kinect.d['Head'] = data[:, 60:63]
        self.kinect.d['ShoulderLeft'] = data[:, 63:66]
        self.kinect.d['ElbowLeft'] = data[:, 66:69]
        self.kinect.d['HandLeft'] = data[:, 72:75]
        self.kinect.d['ShoulderRight'] = data[:, 75:78]
        self.kinect.d['ElbowRight'] = data[:, 78:81]
        self.kinect.d['HandRight'] = data[:, 84:87]

    def fillDataFrame(self, X):
        """
        Description:
            fill the data frame with 1 frame of data

        Input:
            X: the input numpy array, of shape (1, numCols)

        """
        data = X

        # add a fake column in the beginning to make the index match
        data = np.hstack((np.zeros((len(data),1)),data))

        # Myo
        self.myo.d['timestamp'] = np.reshape((data[:,1] - data[0,1]), (-1,1)) # useless
        self.myo.d['onArm'] = np.reshape(data[:,2], (-1,1))       # useless
        self.myo.d['xdir'] = np.reshape(data[:,3], (-1,1))        # useless
        self.myo.d['whichArm'] = np.reshape(data[:,4], (-1,1))    # useless
        self.myo.d['isUnlocked'] = np.reshape(data[:, 5], (-1,1))  # useless
        self.myo.d['rssi'] = np.reshape(data[:,6], (-1,1))        # useless
        self.myo.d['currentPose'] = np.reshape(data[:,7], (-1,1)) # useless
        self.myo.d['emg'] = data[:, 8:16]
        self.myo.d['orientation'] = data[:, 16:19]
        self.myo.d['acceleration'] = data[:, 19:22]
        self.myo.d['gyroscope'] = data[:, 22:25]
        self.myo.X = np.hstack((self.myo.d['emg'], self.myo.d['orientation'],
                              self.myo.d['acceleration'], self.myo.d['gyroscope']))

        # Kinect
        self.kinect.d['time'] = np.reshape(data[:, 25] - data[0, 25], (-1,1)) # useless
        self.kinect.d['leftEye'] = data[:, 26:28]
        self.kinect.d['rightEye'] = data[:, 28:30]
        self.kinect.d['nose'] = data[:, 30:32]
        self.kinect.d['mouthLeft'] = data[:, 32:34]
        self.kinect.d['mouthRight'] = data[:, 34:36]
        self.kinect.d['faceBoundingBox'] = data[:, 36:40]
        self.kinect.d['faceOrientation'] = data[:, 40:43]
        self.kinect.d['faceRoll'] = data[:, 40:41]
        self.kinect.d['facePitch'] = data[:, 41:42]
        self.kinect.d['faceYaw'] = data[:, 42:43]
        self.kinect.d['faceProperty'] = data[:, 43:51]
        self.kinect.d['faceEngaged'] = data[:, 44:45]
        self.kinect.d['faceLookingAway'] = data[:, 50:51]
        self.kinect.d['jointUpperbody'] = np.hstack((data[:, 51:87], data[:, 111:114]))
        self.kinect.d['jointLowerbody'] = data[:, 87:111]
        self.kinect.d['spineMid'] = data[:, 54:57]
        self.kinect.d['leftHand'] = data[:, 72:75]
        self.kinect.d['rightHand'] = data[:, 84:87]
        self.kinect.d['leftHandTip'] = data[:, 114:120]
        self.kinect.d['rightHandTip'] = data[:, 120:126]
        self.kinect.d['leftHandState'] = np.reshape(data[:, 126], (-1,1))
        self.kinect.d['rightHandState'] = np.reshape(data[:, 127], (-1,1))
        self.kinect.d['lean'] = data[:, 128:130]
        self.kinect.d['audioAngle'] = np.reshape(data[:, 130], (-1,1))
        self.kinect.d['audioConfidence'] = np.reshape(data[:, 131], (-1,1))

        # EPOC
        self.epoc.d['counter'] = np.reshape(data[:, 132], (-1,1))     # useless
        self.epoc.d['eeg'] = data[:, 133:147]
        self.epoc.d['gyro'] = data[:,147:149]
        self.epoc.d['timestamp'] = np.reshape(data[:,149] - data[0,149], (-1,1))  # useless
        self.epoc.d['func_id'] = np.reshape(data[:,150], (-1,1))      # useless
        self.epoc.d['func_value'] = np.reshape(data[:,151], (-1,1))   # useless
        self.epoc.d['marker'] = np.reshape(data[:,152], (-1,1))       # useless
        self.epoc.d['sync_signal'] = np.reshape(data[:,153], (-1,1))  # useless
        self.epoc.d['emotion'] = data[:,154:159]                      # useless
        self.epoc.X = np.hstack((self.epoc.d['eeg'], self.epoc.d['gyro']))

        self.numSamples = data.shape[0]


    def GetRelevantData(self):
        """
        Description:
            Get the relevant data from self container

        Returns:
            X: 
                float np array with shape (numSamples, numRawFeatures)
            Name2Index: 
                dict with name as key and index as values
            Index2Name: 
                dict with column index as key and name as values
        """

        # container for all relevant info
        X = np.empty((self.numSamples,0))
        Name2Index = dict()
        Index2Name = dict()

        # get relevant mayo data
        myoHeader = ['emg', 'orientation', 'acceleration', 'gyroscope']
        for key in myoHeader:
            data = self.myo.d[key]
            Name2Index['myo_'+key] = (X.shape[1], X.shape[1] + data.shape[1])
            for columnIndex in range(X.shape[1], X.shape[1]+data.shape[1]):
                Index2Name[columnIndex] = 'myo_'+key+str(columnIndex-X.shape[1])
            X = np.hstack((X, data))

        # get relevant kinect data
        kinectHeader = ['faceBoundingBox', 'faceOrientation', 'faceProperty', 'faceLookingAway',
                        'spineMid', 'leftHand', 'rightHand', 'leftHandState', 'rightHandState',
                        'lean', 'audioAngle', 'audioConfidence']
        for key in kinectHeader:
            data = self.kinect.d[key]
            Name2Index['kinect_'+key] = (X.shape[1], X.shape[1] + data.shape[1])
            for columnIndex in range(X.shape[1], X.shape[1]+data.shape[1]):
                Index2Name[columnIndex] = 'kinect_'+key+str(columnIndex-X.shape[1])
            X = np.hstack((X, data))

        # get relevant epoc data
        epocHeader = ['eeg', 'gyro', 'emotion']
        for key in epocHeader:
            data = self.epoc.d[key]
            Name2Index['epoc_'+key] = (X.shape[1], X.shape[1] + data.shape[1])
            for columnIndex in range(X.shape[1], X.shape[1]+data.shape[1]):
                Index2Name[columnIndex] = 'epoc_'+key+str(columnIndex-X.shape[1])
            X = np.hstack((X, data))

        return X, Name2Index, Index2Name

class DSP:
    def __init__(self):
        self.df = dataFrame()

    def decode_packet(self, pac):
        pac = pac.split()

        # float point data
        raw = []
        for i, item in enumerate(pac):
            if i == 0:
                # it is time
                packet_time = item
                continue
            elif item == 'abcd1234':
                # end of packet string
                break
            raw.append(float(item))

        # fill the data frame
        raw = np.array(raw).reshape(1,-1)
        return raw

    def expSmooth(self, x, zprev, alpha):
        """
        Exponentially Weighted Moving Average (EWMA)
        z_t = alpha * x_t + (1-alpha) * z_{t-1}, z_0 = x_0
        alpha of 0.2 is good
        """
        assert(x.shape == zprev.shape)
        z = alpha * x + (1-alpha) * zprev
        if 0:
            plt.figure()
            plt.hold(True)
            plt.plot(x.flatten(), 'b:', label='raw signal')
            plt.plot(zprev.flatten(), 'g--', label='z prev')
            plt.plot(z.flatten(), 'r-', label='z')
            plt.legend()
            plt.show()
        return z

    def normalize(self, x, mu, sigma):
        """
        normalize each feature channel
        z = (x-mu)/sigma
        """
        assert(x.shape[1] == len(mu) == len(sigma))
        z = (x-mu)/sigma
        return z

    def calcFeature(self, X, Index2Name):
        """
        construct features manually, return an np array f
        f has the same rows as X, but has fewer columns than X
        also return channelName, which indicates the meaning of
        each column of f, should have 50 columns in total
        """

        # just select all relevant and use raw value
        f = np.hstack((X[:,0:17], X[:,21:24], X[:,36:39]-X[:,33:36], X[:,39:42]-X[:,33:36], X[:,44:46], X[:,47:48], X[:,48:]))

        # create channel names
        channelIdx = []
        channelIdx += range(0,17)
        channelIdx += range(21,24)
        channelIdx += range(36,39)
        channelIdx += range(39,42)
        channelIdx += range(44,46)
        channelIdx += range(47,48)
        channelIdx += range(48,X.shape[1])

        channelName = [Index2Name[i] for i in channelIdx]
        return f, channelName

def main():
    dsp = DSP()
    pac = ''.join([str(np.random.rand()) + ' ' for _ in range(160)])
    pac += ' abcd1234  '
    x = dsp.decode_packet(pac)
    N = x.shape[1]
    zprev = np.random.rand(1,N)
    mu = np.random.rand(N)
    sigma = np.random.rand(N)
    
    # smooth
    z = dsp.expSmooth(x, zprev, alpha=0.2)

    # normalize
    z2 = dsp.normalize(z, mu, sigma)

    # get feature
    f, channelName = dsp.calcFeature(z2)
    print channelName
    print f.shape

if __name__ == '__main__':
    main()
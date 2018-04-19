#!/usr/bin/env python

'''
-------------------------------------------------------------------------------
Name:        test module
Purpose:     test purpose
Idea:        how to solve it
Time:        N/A
Space:       N/A
Author:      Tian Zhou
Email:       zhou338 [at] purdue [dot] edu
Created:     01/12/2015
Copyright:   (c) Tian Zhou 2015
-------------------------------------------------------------------------------
'''

import numpy as np
import cv2
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
import collections


class FilterBank():
    """
    A filter class to apply all kinds of filtering, smoothing, pre-processing to the
    input 1D data
    """

    def __init__(self, full):
        self.full = full
        self.createFilterDict(full)

    def createFilterDict(self, full):
        """
        create the filter dictionary
        if full == True, creat all 18 filtuers
        if full == False, only 6 of them
        """
        if full:
            print "Create FULL version of filter dict (16 in total)"
        else:
            print "Create Simple version filter dict (only 6 filter)"

        self.kernelDict = collections.OrderedDict()

        # itself
        self.kernelDict['identity'] = [1]

        # box filter (moving average, smoothing, LPF)
        if full:
            self.kernelDict['box'] = [0.33, 0.33, 0.33]

        # differentiazion filter
        self.kernelDict['diff'] = [-1, 0, 1]

        # Gaussian filter
        g1 = cv2.getGaussianKernel(ksize=3,sigma=-1).flatten()
        g2 = cv2.getGaussianKernel(ksize=5,sigma=-1).flatten()
        g3 = cv2.getGaussianKernel(ksize=7,sigma=-1).flatten()

        if full:
            self.kernelDict['g1'] = g1
            self.kernelDict['g2'] = g2
            self.kernelDict['g3'] = g3

        # 1st order derivative of Gaussian
        self.kernelDict['dg1'] = np.convolve(g1, self.kernelDict['diff'], mode='full')
        if full:
            self.kernelDict['dg2'] = np.convolve(g2, self.kernelDict['diff'], mode='full')
            self.kernelDict['dg3'] = np.convolve(g3, self.kernelDict['diff'], mode='full')

        # 2nd order derivative of Gaussian (LoG)
        self.kernelDict['log1'] = np.convolve(g1, [-1, 2, -1], mode='full')
        if full:
            self.kernelDict['log2'] = np.convolve(g2, [-1, 2, -1], mode='full')
            self.kernelDict['log3'] = np.convolve(g3, [-1, 2, -1], mode='full')

        # gabor filter
        self.kernelDict['gabor1'] = self.gabor(ksize=5, freq=100, theta=0)
        self.kernelDict['gabor2'] = self.gabor(ksize=5, freq=100, theta=np.pi/2)
        if full:
            self.kernelDict['gabor3'] = self.gabor(ksize=7, freq=200, theta=0)
            self.kernelDict['gabor4'] = self.gabor(ksize=7, freq=200, theta=np.pi/2)

    # def add_delay(self, w, location):
    #     assert (w >= 0)
    #     for key in self.kernelDict:
    #         if location == 'start':
    #             self.kernelDict[key] = np.concatenate((np.zeros((w)), self.kernelDict[key]))
    #         elif location == 'end':
    #             self.kernelDict[key] = np.concatenate((self.kernelDict[key], np.zeros((w))))
    #         else:
    #             print "Unrecognized location %s for zero padding" % location

    def gabor(self, ksize, freq, theta):
        """
        define the 1D gabor filter
        sigma is the spread of the Gaussian window, small sigma means small spread
        freq is the central frequency in Hz
        theta is the phase, usually 0, pi/2, pi, 3pi/2
        """
        sigma = 0.3*((ksize-1) * 0.5 -1) + 0.8 # following opencv
        #gab = 1/(np.sqrt(2*np.pi) * sigma) * np.exp(-x**2 / (2*sigma**2)) * np.exp(1j*2*np.pi*freq*x + theta)
        x = np.arange((-ksize/2)+1,(ksize/2)+1)
        gab = 1/(np.sqrt(2*np.pi) * sigma) * np.exp(-x**2 / (2*sigma**2)) * np.cos(2*np.pi*freq*x + theta)
        gab = gab/np.sum(gab) # added by Tian, 3/16/2017. since gabor2 is almost all zero.
        return gab

    def normalize(self, x):
        """
        if x is a numpy array, x.shape = (N,M) normalize x for each one of the M columns
        else if x is a dictionary, then apply normalize to each entry of x
        """
        if type(x) is dict:
            for key in x.keys():
                x[key] = self.normalize(x[key])
            return x
        elif type(x) is np.ndarray:
            if len(x.shape) == 2:
                N,M = x.shape
                for col in range(M):
                    x[:,col] = self.normalize(x[:,col])
                return x
            elif len(x.shape) == 1:
                mu = np.mean(x)
                sigma = np.std(x)
                x -= mu
                if sigma != 0:
                    x /= sigma
                return x


    def showFilterBank(self):
        """
        display all filters, and then randomly generate a signal, and then show the result
        of applying each filter to the signal
        """

        Nfilters = len(self.kernelDict)
        width = int(np.sqrt(Nfilters)) + 1
        index = 1
        plt.figure()
        for key in self.kernelDict:
            kernel = self.kernelDict[key]
            figr = plt.subplot(width, width, index)
            index += 1
            plt.plot(range(len(kernel)), kernel, 'bx-')
            plt.ylabel(key, fontsize = 20)
            plt.tick_params(axis='both', which='major', labelsize=15)

        # generate some random data and filter it with all such kernel
        if 1:
            N = 100
            x = np.concatenate((np.zeros(40), np.ones(10), -np.ones(10), np.zeros((40)))) + \
                    0.0*np.random.randn(N)
            XF = self.applyFilter(x, filterName = 'all')
            index = 1
            plt.figure()
            plt.plot(range(N), x, 'b')
            plt.figure()
            for key in self.kernelDict:
                figr = plt.subplot(width, width, index)
                index += 1
                plt.plot(range(N), XF[key], 'b')
                plt.ylabel(key)
        plt.show()

    def properConv(self, x, h, show=False):
        """
        Description:
            properly convolve x with h
            pad x borders with x[0] and x[-1] of length (len(h)-1)/2, to make sure that
            all values are meaningful. len(h) has to be odd

            if len(x) < len(h):
                do something stupid, it is very rare
            I tried 'valid', 'full', 'same'. None of them is good.
            'valid' requires you to manually add points after conv, adding 0 is not good
                repeat same value is also no good
            'full' is not good either
            'same' will involve lots of 0

        Returns:
            f: the convolved version, should have same length as x, no matter in what condition
        """
        nx, nh = len(x), len(h)
        assert (nh % 2 == 1)
        half_nh = int((nh-1)/2)

        if show:
            print "x before padding: ", x

        # repat x with x[0] and x[-1] for (nh-1)/2 times on both sides
        for _ in range(half_nh):
            x = np.insert(x, 0, x[0])
            x = np.insert(x, len(x), x[-1])

        if show:
            print "x after padding: ", x

        # now convolve
        f = np.convolve(x, h, 'valid')

        assert (len(f) == nx)
        return f

    def applyFilter(self, X, filterName, GaussBlur = True):
        """
        Description:
            apply the filter (specified by filterName) to the given input X
        Args:
            X: either a 1D array of shape (N,) and the filter will directly be applied
                or a 2D matrix of shape (N,M) and the filter will be applied to each column

            filterName: a list which subset of self.filterDict.keys(), if filterName == 'all'
                use all the filters.

            GaussBlur: default is true, all the output are further smoothed by Gaus [1,2,1]

        Returns:
            xf: a dict to store output, xf[key] has the same shape of input X, and is the
                filtered version of X using kernel self.kernelDict(key)
        """

        if len(filterName) == 0 or filterName is None:
            return []

        if filterName == 'all':
            filterName = self.kernelDict.keys()

        # store output
        xf = collections.OrderedDict()
        X_backup = X.copy()

        # 1D array, just convolve it
        if len(X.shape) == 1:
            N = len(X)
            for key in filterName:
                X = np.copy(X_backup)
                F = self.properConv(X, self.kernelDict[key])
                if GaussBlur:
                    F = np.convolve(F, [0.25,0.5,0.25], 'same')
                xf[key] = F
            return xf

        # 2D array, convolve per column
        if len(X.shape) == 2:
            N, M = X.shape
            for key in filterName:
                X = np.copy(X_backup)
                for col in range(M):
                    F = self.properConv(X[:,col], self.kernelDict[key])
                    if GaussBlur:
                        F = np.convolve(F, [0.25,0.5,0.25], 'same')
                    X[:,col] = F
                xf[key] = X
            return xf

    def convertFilteredToMatrix(self, xf, feaNames):
        """
        Description:
            Extract a big X = (N, numFeature x numFilter) matrix from xf
            and return the new names for each column of X

        @arg xf: a dict to store output, xf[key] has the same shape of input X, and is the
                    filtered version of X using kernel self.kernelDict(key)
        @arg feaNames: a list containing names of each column of X

        @ret X: the flattened version of xf

        @ret feaNamesWfilter: the composed new feature names (meaning+filtertype)
        """

        # print "="*60
        # print feaNames
        # print "len(feaNames)", len(feaNames)
        N, numFeature = xf['identity'].shape
        # print "N", N
        # print "numFeature: ", numFeature

        numFilter = len(xf.keys())
        # print "numFilter: ", numFilter
        X = np.zeros((N, 0))
        feaNamesWfilter = []
        for i in range(numFeature):
            for j in xf.keys():
                x = xf[j][:,i].reshape((-1,1))
                X = np.concatenate((X, x), axis=1)
                feaNamesWfilter.append(feaNames[i] + '_' + j)
        # print "feaNamesWfilter: ", feaNamesWfilter
        # print "X.shape: ", X.shape
        # exit()
        return X, feaNamesWfilter


    def BinarizeFilterKmeans(self, X, featureIndex, show = False):
        """
        Create template convolved features from an array of input
        Each input sequence was convolved with a dictionary of templates
        the returned value was a dictionary of filtered signals
        after binarization (using Kmeans, return versions of pos polarity)
        each xbw in XBWs is a dictionary which is the result of binarization of the convolution
        of the original signal x with all the templates T, resulting in T sequences
        In total XBWs should contain 1 \times T \times N_features binary sequences
        """
        XBWs = []
        XBWsNames = []
        print 'Starts binarizing %i features...' % (X.shape[1])
        for i in range(X.shape[1]):
            featureName = featureIndex[i]
            # print "Binarizing %ith feature: %s" % (i, featureName)

            x = X[:, i]
            x = self.Normalize(x)
            #x = self.Smooth(x, method = 'average', winsize = 5)
            #x = filt.Smooth(x, method = 'gaussian', winsize = 3, sigma = 3)
            xtilts, xtiltsName = self.ApplyFilter(x, featureName, show = show)
            xbw = dict()
            for i in range(len(xtilts)):
                xbw[str(i)] = KMeans(n_clusters=2).fit_predict(xtilts[i])
                #xbw[str(i)+'_inv'] = 1-xbw[str(i)]
                #XBWs[featureName+'_'+i] = xbw[i]
                #XBWs[featureName+'_'+i+'_inv'] = 1-xbw[i]
                XBWs.append(xbw[str(i)])
                XBWsNames.append(featureName+'_'+xtiltsName[i])
                #XBWs.append(1-xbw[str(i)])
                #XBWsNames.append(featureName+'_'+xtiltsName[i]+'_inv')

            if show == True:
                Nfilters = len(XBWs)
                width = int(np.sqrt(Nfilters)) + 1
                plt.figure()
                for i in range(len(XBWs)):
                    #figr = plt.subplot(width, width, i+1)
                    figr = plt.subplot(4, 5, i+1)
                    plt.step(range(len(x)), XBWs[i], 'bo')
                    figr.set_xticklabels([])
                    figr.set_yticklabels([])
                    #figr.axes.get_xaxis().set_visible(False)
                    #figr.axes.get_yaxis().set_visible(False)
                    plt.ylabel(XBWsNames[i])
                plt.show()

        # XBWs \in R{T \times NumFilters * NumFeature}
        return np.array(XBWs).T, np.array(XBWsNames)

def main():
    filt = FilterBank(full=True)
    filt.showFilterBank()


if __name__ == '__main__':
    main()
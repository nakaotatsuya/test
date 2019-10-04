#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import rospy
import numpy as np
from audio_common_msgs.msg import AudioData
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
from fft.msg import SpectrumData

class FFTPlotNode():

    def __init__(self):
        self.fig = plt.figure(figsize = (15, 6))
        self.fig.suptitle('Fast Fourier Transform', size=12)
        self.fig.subplots_adjust(left=0.05, bottom=0.1, right=0.95, top=0.90, wspace=0.2, hspace=0.6)

        self.ax0 = plt.subplot2grid((1,2),(0,0))
        self.ax0.grid(True)
        self.ax0.set_title("Input", fontsize=12)
        self.ax0.set_xlabel("Time", fontsize=12)
        self.ax0.set_ylabel("Signal", fontsize=12)
        self.lines0, = self.ax0.plot([-1,-1],[1,1],label='f(n)')

        self.ax1 = plt.subplot2grid((1,2),(0,1))
        self.ax1.grid(True)
        self.ax1.set_title("Spectrum", fontsize=12)
        self.ax1.set_xlabel('Frequency', fontsize=12)
        self.ax1.set_ylabel('Amplitude', fontsize=12)
        self.lines1, = self.ax1.plot([-1,-1],[1,1],label='|F(k)|')

        self.subscribe()

    def subscribe(self):
        self.sub_spectrum = rospy.Subscriber(
            '~spectrum', SpectrumData, self._cb, queue_size=1000, buff_size=2**24)

    def _cb(self, msg):
        self.t = np.array(msg.tm)
        self.f = np.array(msg.audio, dtype='int16')
        self.freq = np.array(msg.frequency)
        self.Amp = np.array(msg.spectrum)
        self.plot()

    def plot(self):
        # plot graph
        self.lines0.set_data(self.t, self.f)
        self.ax0.set_xlim((self.t.min(), self.t.max()))
        self.ax0.set_ylim((-256.0 ** 2 / 2, 256.0 ** 2 / 2))
        self.ax0.legend(loc='upper right')

        self.lines1.set_data(self.freq, self.Amp)
        self.ax1.set_xlim((self.freq.min(), self.freq.max()/2))
        # self.ax1.set_ylim((0.0, Amp.max()))
        self.ax1.set_ylim((0.0, 5e+5))
        self.ax1.legend(loc='upper right')

if __name__=='__main__':
    rospy.init_node('fft_plot_node')
    FFTPlotNode()
    while not rospy.is_shutdown():
        plt.pause(.01)

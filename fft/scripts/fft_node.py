#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import rospy
import numpy as np
from audio_common_msgs.msg import AudioData
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
from fft.msg import SpectrumData

class AudioToSpeechAudioNode():

    def __init__(self):
        self.audio_prefetch = rospy.get_param("~audio_prefetch", 0.5)
        self.rate = rospy.get_param("~rate", 16000)
        self.bitwidth = rospy.get_param("~bitwidth", 2)
        self.bitdepth = rospy.get_param("~bitdepth", 16)

        self.sampling_rate = rospy.get_param("~sampling_rate", 1600)
        self.audio_prefetch_bytes = int(
            self.audio_prefetch * self.rate * self.bitdepth / 8.0)
        self.audio_prefetch_buffer = str()
        self.audio_prefetch_buffer_sparse = np.array([], dtype='int16')
        self.audio_prefetch_sparse_bytes = int(
            self.audio_prefetch_bytes * self.sampling_rate // self.rate // 2)

        self.freq = np.linspace(0, self.sampling_rate, self.audio_prefetch_sparse_bytes)
        self.t = np.arange(0, self.audio_prefetch, 1.0/self.sampling_rate)
        self.window_function = 0.54 - 0.46 * np.cos(2 * np.pi * np.arange(0.0, 1.0, 1.0 / (self.audio_prefetch * self.sampling_rate)))

        self.plot_graph = rospy.get_param("~plot", False)
        if self.plot_graph:
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

        rospy.Rate(100)
        # advertise
        self.pub = rospy.Publisher('~output', SpectrumData, queue_size=2**24)
        self.subscribe()

    def subscribe(self):
        self.sub_audio = rospy.Subscriber(
            '~audio', AudioData, self._cb, queue_size=1000, buff_size=2**24)

    def _cb(self, msg):
        data = msg.data
        data16 = np.frombuffer(data, dtype='int16')

        self.audio_prefetch_buffer += data
        self.audio_prefetch_buffer = self.audio_prefetch_buffer[-self.audio_prefetch_bytes:]
        self.audio_prefetch_buffer_sparse = np.append(
            self.audio_prefetch_buffer_sparse,
            np.frombuffer(data, dtype='int16')[np.where(np.arange(len(data) // 2) % (self.rate // self.sampling_rate) == 0)])
        self.audio_prefetch_buffer_sparse = self.audio_prefetch_buffer_sparse[-self.audio_prefetch_sparse_bytes:]

        if len(self.audio_prefetch_buffer_sparse) != self.audio_prefetch * self.sampling_rate:
            return

        # window function
        self.f = self.audio_prefetch_buffer_sparse * self.window_function

        # fft
        self.F = np.fft.fft(self.f)

        # amplitude spectrum
        self.Amp = np.abs(self.F)

        # plot graph
        if self.plot_graph:
            self.plot()

        # publish
        self.publish()

    def plot(self):
        # plot graph
        self.lines0.set_data(self.t, self.f)
        self.ax0.set_xlim((self.t.min(), self.t.max()))
        self.ax0.set_ylim((-256.0 ** 2 / 2, 256.0 ** 2 / 2))
        self.ax0.legend(loc='upper right')

        self.lines1.set_data(self.freq, self.Amp)
        self.ax1.set_xlim((self.freq.min(), self.freq.max()/2))
        # self.ax1.set_ylim((0.0, Amp.max()))
        self.ax1.set_ylim((0.0, 2e+5))
        self.ax1.legend(loc='upper right')

    def publish(self):
        msg = SpectrumData(tm=self.t,
                           audio=self.f,
                           frequency=self.freq,
                           spectrum=self.Amp)
        self.pub.publish(msg)


if __name__=='__main__':
    rospy.init_node('audio_to_speech_audio_node')
    AudioToSpeechAudioNode()
    while not rospy.is_shutdown():
        plt.pause(.0001)


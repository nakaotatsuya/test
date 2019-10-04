#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import rospy
import numpy as np
from audio_common_msgs.msg import AudioData
from std_msgs.msg import Bool
from fft.msg import SpectrumData
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui,QtCore

class FFTPlotNode():

    def __init__(self):
        self.UPDATE_SECOND = 10
        self.freq = []
        self.Amp = []

        self.subscribe()
        # Qt graph configuration
        self.app = QtGui.QApplication([])
        self.app.quitOnLastWindowClosed()
        # Window
        self.win = QtGui.QMainWindow()
        self.win.setWindowTitle("SpectrumVisualizer")
        self.win.resize(750,400)
        self.centralwid = QtGui.QWidget()
        self.win.setCentralWidget(self.centralwid)
        # Layout
        self.lay = QtGui.QVBoxLayout()
        self.centralwid.setLayout(self.lay)

        self.plotwid1 = pg.PlotWidget(name="spectrum")
        self.plotitem1 = self.plotwid1.getPlotItem()
        self.plotitem1.setMouseEnabled(x=False,y=False)
        self.plotitem1.setXRange(0,4000,padding=0)
        self.plotitem1.setYRange(0,2e+5)
        self.specAxis1 = self.plotitem1.getAxis("bottom")
        self.specAxis1.setLabel("Frequency[Hz]")
        self.curve1 = self.plotitem1.plot()
        self.lay.addWidget(self.plotwid1)
        # Show plot window
        self.win.show()
        # Update timer setting
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(self.UPDATE_SECOND)

    def subscribe(self):
        self.sub_spectrum = rospy.Subscriber('/fft/output', SpectrumData, self._cb, queue_size=1000, buff_size=2**24)
    def _cb(self, msg):
        self.freq = np.array(msg.frequency)
        self.Amp = np.array(msg.spectrum)
    def update(self):
        self.curve1.setData(self.freq,self.Amp)

if __name__=='__main__':
    rospy.init_node('fft_plot_node')
    FFTPlotNode()
    try:
        QtGui.QApplication.instance().exec_()
    except:
        print('terminate')

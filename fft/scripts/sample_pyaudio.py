#!/usr/bin/env python
# encoding: utf-8

## Module infomation ###
# Python (3.4.4)
# numpy (1.10.2)
# PyAudio (0.2.9)
# pyqtgraph (0.9.10)
# PyQt4 (4.11.4)
# All 32bit edition
########################

import pyaudio
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore

class SpectrumAnalyzer:
    FORMAT = pyaudio.paFloat32
    CHANNELS = 1
    RATE = 16000
    CHUNK = 1024
    START = 0
    N = 1024
    WAVE_RANGE = 1
    SPECTRUM_RANGE = 50
    UPDATE_SECOND = 10

    def __init__(self):
        #Pyaudio Configuration
        self.pa = pyaudio.PyAudio()
        self.stream = self.pa.open(format = self.FORMAT,
                channels = self.CHANNELS,
                rate = self.RATE,
                input = True,
                output = False,
                frames_per_buffer = self.CHUNK)
       
        # Graph configuration
        # Application
        self.app = QtGui.QApplication([])
        self.app.quitOnLastWindowClosed()
        # Window
        self.win = QtGui.QMainWindow()
        self.win.setWindowTitle("SpectrumAnalyzer")
        self.win.resize(800, 600)
        self.centralwid = QtGui.QWidget()
        self.win.setCentralWidget(self.centralwid) 
        # Layout
        self.lay = QtGui.QVBoxLayout()
        self.centralwid.setLayout(self.lay)
        # Wave figure window setting
        self.plotwid1 = pg.PlotWidget(name="wave")
        self.plotitem1 = self.plotwid1.getPlotItem()
        self.plotitem1.setMouseEnabled(x = False, y = False) 
        self.plotitem1.setYRange(self.WAVE_RANGE * -1, self.WAVE_RANGE * 1)
        self.plotitem1.setXRange(self.START, self.START + self.N, padding = 0)
        # Spectrum windows setting
        self.plotwid2 = pg.PlotWidget(name="spectrum")
        self.plotitem2 = self.plotwid2.getPlotItem()
        self.plotitem2.setMouseEnabled(x = False, y = False) 
        self.plotitem2.setYRange(0, self.SPECTRUM_RANGE)
        self.plotitem2.setXRange(0, self.RATE / 2, padding = 0)
        # Wave figure Axis
        self.specAxis1 = self.plotitem1.getAxis("bottom")
        self.specAxis1.setLabel("Time [sample]")
        # Spectrum Axis
        self.specAxis2 = self.plotitem2.getAxis("bottom")
        self.specAxis2.setLabel("Frequency [Hz]")
        #Plot data
        self.curve_wave = self.plotitem1.plot()
        self.curve_spectrum = self.plotitem2.plot()
        #Widget
        self.lay.addWidget(self.plotwid1)
        self.lay.addWidget(self.plotwid2)
        #Show plot window
        self.win.show()
        #Update timer setting
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(self.UPDATE_SECOND)

    def update(self):
        #Get audio input
        data = self.audioinput()
        
        # Wave figure
        wave_figure = data[self.START:self.START + self.N]
        # Wave time
        wave_time = range(self.START, self.START + self.N)
        # Frequency
        freqlist = np.fft.fftfreq(self.N, d = 1.0 / self.RATE) 
        # Spectrum power
        x = np.fft.fft(data[self.START:self.START + self.N])
        amplitudeSpectrum = [np.sqrt(c.real ** 2 + c.imag ** 2) for c in x]

        # Plot setdata
        self.curve_wave.setData(wave_time, wave_figure)
        self.curve_spectrum.setData(freqlist, amplitudeSpectrum)

    def audioinput(self):
        ret = self.stream.read(self.CHUNK)
        ret = np.fromstring(ret, np.float32)

        return ret


if __name__ == "__main__":
    spec = SpectrumAnalyzer()
    QtGui.QApplication.instance().exec_()

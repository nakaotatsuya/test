#!/usr/bin/env python
# -*- coding: utf-8 -*-

import wave
import numpy as np
from pylab import *
import scipy.signal
import scipy.fftpack
import scipy.fftpack.realtransforms
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC
from sklearn.metrics import accuracy_score
import matplotlib.pyplot as plt
from mlxtend.plotting import plot_decision_regions

def wavread(filename):
    wf = wave.open(filename, "r")
    fs = wf.getframerate()
    x = wf.readframes(wf.getnframes())
    x = np.frombuffer(x, dtype="int16") / 32768.0  # (-1, 1)に正規化
    wf.close()
    return x, float(fs)

def hz2mel(f):
    """Hzをmelに変換"""
    return 1127.01048 * np.log(f / 700.0 + 1.0)

def mel2hz(m):
    """melをhzに変換"""
    return 700.0 * (np.exp(m / 1127.01048) - 1.0)

def melFilterBank(fs, nfft, numChannels):
    """メルフィルタバンクを作成"""
    # ナイキスト周波数（Hz）
    fmax = fs / 2
    # ナイキスト周波数（mel）
    melmax = hz2mel(fmax)
    # 周波数インデックスの最大数
    nmax = nfft / 2
    # 周波数解像度（周波数インデックス1あたりのHz幅）
    df = fs / nfft
    # メル尺度における各フィルタの中心周波数を求める
    dmel = melmax / (numChannels + 1)
    melcenters = np.arange(1, numChannels + 1) * dmel
    # 各フィルタの中心周波数をHzに変換
    fcenters = mel2hz(melcenters)
    # 各フィルタの中心周波数を周波数インデックスに変換
    indexcenter = np.round(fcenters / df)
    # 各フィルタの開始位置のインデックス
    indexstart = np.hstack(([0], indexcenter[0:numChannels - 1]))
    # 各フィルタの終了位置のインデックス
    indexstop = np.hstack((indexcenter[1:numChannels], [nmax]))

    filterbank = np.zeros((numChannels, nmax))
    for c in np.arange(0, numChannels):
        # 三角フィルタの左の直線の傾きから点を求める
        increment= 1.0 / (indexcenter[c] - indexstart[c])
        for i in np.arange(indexstart[c], indexcenter[c]):
            i = int(i)
            filterbank[c, i] = (i - indexstart[c]) * increment
        # 三角フィルタの右の直線の傾きから点を求める
        decrement = 1.0 / (indexstop[c] - indexcenter[c])
        for i in np.arange(indexcenter[c], indexstop[c]):
            i = int(i)
            filterbank[c, i] = 1.0 - ((i - indexcenter[c]) * decrement)

    return filterbank, fcenters

def preEmphasis(signal,p):
    return scipy.signal.lfilter([1.0,-p], 1, signal)

def mfcc(signal, nfft, fs, nceps):
    """信号のMFCCパラメータを求める
    signal: 音声信号
    nfft  : FFTのサンプル数
    nceps : MFCCの次元"""
    # プリエンファシスフィルタをかける
    p = 0.97         # プリエンファシス係数
    signal = preEmphasis(signal, p)
    # ハミング窓をかける
    hammingWindow = np.hamming(len(signal))
    signal = signal * hammingWindow

    spec = np.abs(np.fft.fft(signal, nfft))[:nfft/2]
    #print(len(spec)) #1024
    fscale = np.fft.fftfreq(nfft, d = 1.0/fs)[:nfft/2]

    # プロット
    #plot(fscale, spec)
    #xlabel("frequency [Hz]")
    #ylabel("amplitude spectrum")
    #savefig("spectrum.png")
    #show()

    # メルフィルタバンクを作成
    numChannels = 20  # メルフィルタバンクのチャネル数
    df = fs / nfft   # 周波数解像度（周波数インデックス1あたりのHz幅）
    filterbank, fcenters = melFilterBank(fs, nfft, numChannels)

    # メルフィルタバンクのプロット
    #for c in np.arange(0, numChannels):
    #    plot(np.arange(0, nfft / 2) * df, filterbank[c])
    #savefig("melfilterbank.png")
    #show()

    # 振幅スペクトルに対してフィルタバンクの各フィルタをかけ、
    # 振幅の和の対数をとる
    mspec = []
    for c in np.arange(0, numChannels):
        mspec.append(np.log10(sum(spec * filterbank[c])))
    mspec = np.array(mspec)

    # 元の振幅スペクトルとフィルタバンクをかけて圧縮したスペクトルを表示
    # subplot(211)
    # plot(fscale, np.log10(spec))
    # xlabel("frequency")
    # xlim(0, 25000)

    # subplot(212)
    # plot(fcenters, mspec, "o-")
    # xlabel("frequency")
    # xlim(0, 25000)
    # show()

    ceps = scipy.fftpack.realtransforms.dct(mspec, type=2, norm="ortho", axis=-1)
    return ceps[:nceps]

class SVM():
    def __init__(self):
        pass

    def make_data(self, n, label):
        X = np.empty((0,2), float)
        y = []
        for i in range(n):
            wav, fs = wavread("/home/nakaotatsuya/Downloads/non%d.wav"%(i+1))
            t = np.arange(0.0, len(wav) / fs, 1/fs)
            center = len(wav)/2
            #print(center)
            cuttime = 0.5
            wavdata = wav[int(center - cuttime/2 * fs) : int(center + cuttime/2 * fs)]
            time = t[int(center - cuttime/2 * fs) : int(center + cuttime/2 * fs)]

            nfft = 2048
            nceps = 12
            ceps = mfcc(wavdata, nfft, fs, nceps)
            print("mfcc: ", ceps)

            if len(X) == 0:
                X = ceps
            else:
                X = np.vstack((X, ceps))
            y = np.append(y,label[0])

        for i in range(n):
            wav, fs = wavread("/home/nakaotatsuya/Downloads/test%d.wav"%(i+1))
            t = np.arange(0.0, len(wav) / fs, 1/fs)
            center = len(wav)/2
            #print(center)
            cuttime = 0.5
            wavdata = wav[int(center - cuttime/2 * fs) : int(center + cuttime/2 * fs)]
            time = t[int(center - cuttime/2 * fs) : int(center + cuttime/2 * fs)]

            nfft = 2048
            nceps = 12
            ceps = mfcc(wavdata, nfft, fs, nceps)
            print("mfcc: ", ceps)

            if len(X) == 0:
                X = ceps
            else:
                X = np.vstack((X, ceps))
            y = np.append(y,label[1])
        return X, y

    def fit(self, X, y):
        self.X_train , self.X_test, self.y_train, self.y_test = train_test_split(X, y, test_size=0.3, random_state=None)
        sc = StandardScaler()
        sc.fit(self.X_train)
        self.X_train_std = sc.transform(self.X_train)
        self.X_test_std = sc.transform(self.X_test)
        
        self.model = SVC(kernel="linear", random_state=None)
        self.model.fit(self.X_train_std, self.y_train)

    def predict(self, X, y):
        pred = self.model.predict(X)
        accuracy_test = accuracy_score(y, pred)
        print("accuracy: %.2f"%accuracy_test)

    def svm(self):
        self.X, self.y = self.make_data(5, 0)
        self.fit(self.X, self.y)
        self.predict(self.X_test_std, self.y_test)
        
if __name__ == "__main__":
    svm = SVM()
    X, y = svm.make_data(30,[0,1])
    print(X)
    print(y)

    svm.fit(X,y)
    

    print(svm.model.predict(svm.X_test_std))
    print(svm.y_test)
    svm.predict(svm.X_test_std, svm.y_test)

    plt.style.use("ggplot")
    X_combined_std = np.vstack((svm.X_train_std, svm.X_test_std))
    y_combined = np.hstack((svm.y_train, svm.y_test))

    y_combined = y_combined.astype(int)
    #fig = plt.figure(figsize=(13,8))
    #plot_decision_regions(X_combined_std, y_combined, clf=svm.model, res=0.02)
    #plt.show()
    # wav, fs = wavread("/home/nakaotatsuya/Downloads/hoge.wav")
    # t = np.arange(0.0, len(wav) / fs, 1/fs)
    # center = len(wav)/2
    # #print(center)
    # cuttime = 0.06
    # wavdata = wav[int(center - cuttime/2 * fs) : int(center + cuttime/2 * fs)]
    # time = t[int(center - cuttime/2 * fs) : int(center + cuttime/2 * fs)]

    # nfft = 2048
    # nceps = 12
    # ceps = mfcc(wavdata, nfft, fs, nceps)
    # print("mfcc: ", ceps)

    # plot(time, wavdata)
    # xlabel("time [ms]")
    # ylabel("amp")
    # show()

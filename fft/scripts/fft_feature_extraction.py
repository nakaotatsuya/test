#!/usr/bin/env python

import rospy
from fft.msg import SpectrumData
from std_msgs.msg import Float32, Float32MultiArray
import numpy as np

class FFTFeatureExtraction():

    def __init__(self):
        self.threshold = rospy.get_param('~threshold', 10000)

        self.pub = rospy.Publisher('~output', Float32, queue_size=1)
        self.subscribe()

    def subscribe(self):
        sub_spectrum = rospy.Subscriber(
            '~input', SpectrumData, self._cb, queue_size=1, buff_size=2**24)

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def _cb(self, msg):
        half = max(msg.frequency) / 2.0
        orig_freq = np.array(msg.frequency)
        orig_Amp = np.array(msg.spectrum)
        cut_off_ind = np.where((100.0 < orig_freq) & (orig_freq < half))

        freq = orig_freq[cut_off_ind]
        Amp = orig_Amp[cut_off_ind]
        ind = np.arange(len(freq))

        high_ind = Amp > self.threshold
        feature_freq = freq[high_ind]
        feature_amp = Amp[high_ind]
        # feature_ind = ind[high_ind]

        if len(feature_freq) > 0:
            max_freq = feature_freq[np.argmax(feature_amp)]
            pub_msg = Float32(data=max_freq)
            self.pub.publish(pub_msg)

if __name__ == "__main__":
    rospy.init_node('fft_feature_extraction_node')
    FFTFeatureExtraction()
    rospy.spin()


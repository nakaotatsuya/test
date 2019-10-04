#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pyaudio
import wave

def main():
    rec_time=5
    file_path="output.wav"
    fmt = pyaudio.paInt16
    ch = 1
    sampling_rate = 44100
    chunk = 2**11
    audio = pyaudio.PyAudio()
    index = 9

    stream = audio.open(format=fmt, channels=ch, rate=sampling_rate, input=True, input_device_index=index, frames_per_buffer=chunk)
    print("recording start ... ")

    frames = []
    for i in range(0, int(sampling_rate / chunk * rec_time)):
        data = stream.read(chunk)
        frames.append(data)

    print("recording end ...")

    stream.stop_stream()
    stream.close()
    audio.terminate()

    wav = wave.open(file_path, 'wb')
    wav.setnchannels(ch)
    wav.setsampwidth(audio.get_sample_size(fmt))
    wav.setframerate(sampling_rate)
    wav.writeframes(b''.join(frames))
    wav.close()

if __name__ == "__main__":
    main()

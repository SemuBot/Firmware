#!/usr/bin/env python3
import pyaudio

p = pyaudio.PyAudio()

print("Available audio input devices:")
print("-" * 80)

for i in range(p.get_device_count()):
    info = p.get_device_info_by_index(i)
    if info['maxInputChannels'] > 0:
        print(f"Index: {i}")
        print(f"  Name: {info['name']}")
        print(f"  Channels: {info['maxInputChannels']}")
        print(f"  Sample Rate: {info['defaultSampleRate']}")
        print()

p.terminate()

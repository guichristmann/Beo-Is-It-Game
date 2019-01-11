# -*- encoding: utf-8 -*-

import sys
sys.path.append('speech_detection/')
from speech_detector import SpeechDetection
from datetime import datetime

if len(sys.argv) != 2:
    print("Usage: python " + sys.argv[0] + " <speech_model>")
    sys.exit(0)

detect_count = 0
def callback():
    global detect_count
    
    detect_count += 1

def getTimestamp():
    return str(datetime.now()).split('.')[0]

model = SpeechDetection(sys.argv[1],
                        wake_callback=[callback],
                        audio_gain=0.95,
                        sensitivity=0.5)

print("Start time: " + getTimestamp())
while True:
    model.run(process=False)
    print("Detect count: " + str(detect_count))
    print("Current timestamp: " + getTimestamp())

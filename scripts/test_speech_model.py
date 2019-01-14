# -*- encoding: utf-8 -*-

import sys
sys.path.append('speech_detection/')
from speech_detector import SpeechDetection
from datetime import datetime

yes_count = 0
no_count = 0
isit_count = 0
def YesCallback():
    global yes_count
    
    yes_count += 1

def NoCallback():
    global no_count

    no_count += 1

def IsItCallback():
    global isit_count

    isit_count += 1

def getTimestamp():
    return str(datetime.now()).split('.')[0]

models = SpeechDetection(["speech_models/Yes.pmdl", "speech_models/No.pmdl"],
                            wake_callback=[YesCallback, NoCallback],
                            audio_gain=0.95,
                            sensitivity=0.4)

#isit_model = SpeechDetection("speech_models/Is_it.pmdl",
#                            wake_callback=[IsItCallback],
#                            audio_gain=0.95,
#                            sensitivity=0.45)

print("Start time: " + getTimestamp())
while True:
    models.run(process=False)
    #isit_model.run(process=False)
    #print('"Is It" count: ' + str(isit_count))
    print('"Yes" count: ' + str(yes_count))
    print('"No" count: ' + str(no_count))
    print("Current timestamp: " + getTimestamp())

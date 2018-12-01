# -*- encoding: utf-8 -*-
from snowboy.snowboydecoder import HotwordDetector
import speech_recognition as sr
import sys

GOOGLE_API_KEY='AIzaSyAeRQumWU2W-KhA_Uki0mnOqUcvwiKV1wE'

def dummy():
    pass

class SpeechDetection:
    def __init__(self, wake_word_model, record_callback=None, wake_callback=None, audio_gain=0.95, sensitivity=0.45):
        
        #print(record_callback, wake_callback)
        # At least one of these should be passed
        if record_callback == None and wake_callback == None:
            print("[SpeechDetection] meh.")
            sys.exit(1)

        # Google Speech Recognition
        self.SR = sr.Recognizer()

        # After running Speech to text, return the string to the given function
        if record_callback:
            self.record_callback = record_callback

        if wake_callback:
            def wcb(f): return lambda : self.wake_cb(f)

            self.wake_callback = [wcb(f) for f in wake_callback]

        # Create detector model
        self.detector = HotwordDetector(wake_word_model, sensitivity=sensitivity,
                                  audio_gain=audio_gain)

    def interrupt_check(self):
        return self.detected

    def run(self, process=True):
        print("[SpeechDetection] Running...")

        self.detected = False # Will be True when a detection has been performed

        # Process speech further than wake word?
        if process:
            self.detector.start(self.wake_cb_process, 
                                audio_recorder_callback=self.record_cb,
                                interrupt_check=self.interrupt_check,
                                recording_timeout=8)
        else:
            #print(self.wake_callback)
            self.detector.start(self.wake_callback,
                                interrupt_check=self.interrupt_check)


    def wake_cb_process(self):
        print("[SpeechDetection] Got wake word!")

    def wake_cb(self, function):
        self.detected = True
        function() # Behold ugly hack
        print("[SpeechDetection] Got wake word!")

    def record_cb(self, audio_file):
        print("[SpeechDetection] Running speech recognition...")
        with sr.AudioFile(audio_file) as source:
            audio = self.SR.record(source)

        try:
            recognized_string = self.SR.recognize_google(audio, 
                                    key=GOOGLE_API_KEY)

            self.record_callback(recognized_string)

        except sr.UnknownValueError:
            self.record_callback("")
            print("[SpeechDetection] Nothing detected!")
        except sr.RequestError:
            self.record_callback("")
            print("[SpeechDetection] Internet failed.:(")

        self.detected = True

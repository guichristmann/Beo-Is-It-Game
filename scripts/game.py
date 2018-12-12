#! /usr/bin/env python

# -*- encoding: utf-8 -*-

PACKAGE_PATH = '/home/linaro/catkin_ws/src/beo_isit_game/scripts/'

import sys
sys.path.append(PACKAGE_PATH + 'speech_detection/')
sys.path.append(PACKAGE_PATH + 'Beo_Audio/')
from speech_detector import SpeechDetection
from Audio import Audio
from Movements import Movements
from Eyes import Eyes
from game_utils import txtToDict
from time import sleep
import random
import numpy as np
from itertools import chain
import rospy
from std_msgs.msg import String
from pygame import mixer, time

from subprocess import Popen

DEFAULT_OBJS_FN = PACKAGE_PATH + "objects_description.txt"
MOTIONS_FOLDER = PACKAGE_PATH + "motions/"

class States:
    INTRODUCTION = 0
    EXPLANATION = 1
    CONFIRMATION = 2
    PRE_GAME = 3
    BEO_GUESSER0 = 4
    USER_GUESSER0 = 5
    USER_GUESSER1 = 6

# Helper class to handling the guessing game
class BeoGuesser:
    def __init__(self, objs_dict):
        self.objs_dict = objs_dict
        self.objs = self.objs_dict.keys()
        
        # Every single possible characteristic
        self.chars = list(np.unique(list(chain.from_iterable(self.objs_dict.values())))) 

        self.remaining_objs = self.objs

        self.positives = []
        self.negatives = []

    # Function to process if spoken string matches the characteristics. This is done
    # to remove unnecessary words ('a', 'the', etc.) and to match similar words like
    # when user says 'thin' but the detector catches 'Finn'
    def processIsIt(self, string):
        string = string.lower()

    def askQuestion(self):

        # Get random object from remaining and random characteristic
        obj_name = random.sample(self.remaining_objs, 1)[0] 
        obj_chars = self.objs_dict[obj_name] # Get characteristics of the chosen object
        print("[askQuestion]: Ask question about '{}'".format(obj_name))
        print("[askQuestion]: Object Characteristics '{}'".format(obj_chars))
        unasked_chars = list(set(obj_chars) - set(self.negatives + self.positives))
        print("[askQuestion]: Unasked chars: '{}'".format(unasked_chars))
        characteristic = random.sample(unasked_chars, 1)[0]
        print("[askQuestion]: Chosen: '{}'".format(characteristic))

        return characteristic

    def gotHint(self, hint, positive):
        # Keeps track of which objects to remove according to the hint
        del_objs = []

        if positive:
            self.positives.append(hint) # Append hint
            
            for obj in self.remaining_objs:
                if hint not in self.objs_dict[obj]: # If object's characteristics doesn't contain the hint
                    del_objs.append(obj)

        else:
            self.negatives.append(hint)

            for obj in self.remaining_objs:
                if hint in self.objs_dict[obj]: 
                    del_objs.append(obj)

        for d in del_objs:
            print("[gotHint] Removing '{}'".format(d))
            self.remaining_objs.remove(d)

    def possibleObjs(self):
        return self.remaining_objs

class BeoPicker:
    def __init__(self, objs_dict):
        self.objs_dict = objs_dict
        self.objs = self.objs_dict.keys()
        
        # Every single possible characteristic
        self.chars = list(np.unique(list(chain.from_iterable(self.objs_dict.values())))) 

        self.chosen_obj = None
        self.chosen_chars = None

    def pickRandomObject(self):
        self.chosen_obj = random.sample(self.objs, 1)[0]
        self.chosen_obj = 'journal'
        self.chosen_chars = self.objs_dict[self.chosen_obj]

        print("[pickRandomObject] Chosen Object: {}".format(self.chosen_obj))
        print("[pickRandomObject] Chosen Chars: {}".format(self.chosen_chars))

class IsItGame:

    def __init__(self, objs_file=DEFAULT_OBJS_FN):
        # Dictionary that holds each object name and a list of 
        # its caracteristics
        self.objs_features = txtToDict(objs_file) 
        self.n_objs = len(txtToDict(objs_file).keys())

        # Speech detection module
        self.isit_sd = SpeechDetection(PACKAGE_PATH + "speech_models/Is_it.pmdl",
                                       record_callback=self.processSpeech)

        self.yesno_sd = SpeechDetection([PACKAGE_PATH + "speech_models/Yes.pmdl",
                                         PACKAGE_PATH + "speech_models/No.pmdl"],
                                           wake_callback=[self.saidYes,
                                            self.saidNo], audio_gain=0.95,
                                          sensitivity=0.4)


        # Beo Audio module, used for speech generation
        self.beo_audio = Audio()

        # Controls joints movements
        self.beo_movs = Movements(no_print=True)

        # Controls eyes display
        self.beo_eyes = Eyes()



        self.player_said = ''

    def saidYes(self):
        self.player_said = 'yes'

    def saidNo(self):
        self.player_said = 'no'

    def beoSpeakMotion(self, sentence, motions=''):
        self.beo_audio.speak(sentence)
        
        if type(motions) == list:
            [self.beo_movs.play_motion_file(motion) for motion in motions]

        else:
            if motions != '':
                self.beo_movs.play_motion_file(motions)

        while mixer.music.get_busy():
            time.delay(100)
            print("Waiting end of utterance.")

    # Callback when calling run method for speech detection
    def processSpeech(self, string):
        self.player_said = string
        print("Detected Speech: {}".format(self.player_said))

    def IntroductionSequence(self):
       self.beo_eyes.set_expression("default")
       
       self.beoSpeakMotion("Hello", MOTIONS_FOLDER + "intro_0.mot")
       self.beoSpeakMotion("Wanna play a game?", MOTIONS_FOLDER + "intro_1.mot")
       self.beo_eyes.set_expression("wink")
       sleep(0.5)
       self.beo_eyes.set_expression("default")

    def ExplanationSequence(self):
       self.beoSpeakMotion("This will be a guessing game with these objects in front of us.", MOTIONS_FOLDER + "exp_0.mot")
       sleep(0.5)
       self.beoSpeakMotion("Each of us will take turns in choosing an object, and the other person has to guess what it is.", MOTIONS_FOLDER + "exp_1.mot")
       self.beoSpeakMotion("We will do that by asking 'Yes' and 'No' questions about the characteristics of these objects.", MOTIONS_FOLDER + "exp_2.mot")
       sleep(0.5)
       self.beoSpeakMotion('All our questions should start with Is it?. For example: Is it big?, Is it white? or Is it a cylinder?',
            [MOTIONS_FOLDER + "exp_3.mot"])
       sleep(1.2)

    def ConfirmationSequence(self):
        self.beoSpeakMotion("Did you understand everything so far?",
                            MOTIONS_FOLDER + 'conf_0.mot')

        # Wait player response
        self.yesno_sd.run(process=False)

        if self.player_said == 'no':
            self.beoSpeakMotion("Then I'll explain it one more time.",
                                 MOTIONS_FOLDER + "exp_4.mot")
            return False

        self.beoSpeakMotion("Great. Let's start.", MOTIONS_FOLDER + 'conf_1.mot')

        return True

    def PregameSequence(self):
        self.beoSpeakMotion("Okay, I will start as the guesser. Choose one of the objects in front of us. But don't tell me!", MOTIONS_FOLDER + 'pre_0.mot')
        sleep(4)
        self.beoSpeakMotion("When you are ready, say YES.", MOTIONS_FOLDER + 'pre_1.mot')
        self.yesno_sd.run(process=False)
        self.beoSpeakMotion("Okay, I'm going to try and guess which object you chose!", MOTIONS_FOLDER + 'pre_2.mot')
        self.beoSpeakMotion("Answer my questions!", MOTIONS_FOLDER + 'pre_3.mot')
        sleep(2)

    def BeoGuesser0(self):
        beo_guesser = BeoGuesser(self.objs_features)        

        motions = ['beo_0.mot', 'beo_1.mot', 'beo_2.mot']

        # Make 5 questions
        for i in range(5):
            characteristic = beo_guesser.askQuestion()

            # Get random motion
            m = random.sample(motions, 1)[0]

            # Ask question
            self.beoSpeakMotion("Is it " + characteristic + "?", MOTIONS_FOLDER + m)
            
            # Wait Player response
            self.yesno_sd.run(process=False)

            # Process hint
            if self.player_said == 'yes':
                beo_guesser.gotHint(characteristic, True)
            elif self.player_said == 'no':
                beo_guesser.gotHint(characteristic, False)

            # Get possible objects
            possible_objs = beo_guesser.possibleObjs()

            print("Possible objects: {}".format(possible_objs))

            if len(possible_objs) == 1:
                break
            elif len(possible_objs) == 0:
                self.beoSpeakMotion("I think you made a mistake...")

        obj = possible_objs[0].replace("_", " ")
        self.beoSpeakMotion("You're thinking of the " + obj + "?. Right?", MOTIONS_FOLDER + 'bbeo_0.mot')

        self.yesno_sd.run(process=False)

        if self.player_said == 'yes':
            self.beoSpeakMotion("Yesss. I knew it.", MOTIONS_FOLDER + 'bbeo_1.mot')
        elif self.player_said == 'no':
            self.beoSpeakMotion("Ahhh. I can't believe I was wrong.", MOTIONS_FOLDER + 'pre_2.mot')
            self.beoSpeakMotion("Congratulations.", MOTIONS_FOLDER + 'bbeo_0.mot')

        sleep(2)

    def UserGuesser0(self):
        self.beoSpeakMotion("Now it's your turn to be the guesser.", MOTIONS_FOLDER + 'user_1.mot')

        self.beoSpeakMotion("I'm gonna pick an object. Give me a second.", MOTIONS_FOLDER + 'user_0.mot')

        sleep(3)

        self.beoSpeakMotion("Okay, I'm ready. You can ask me 5 questions.", MOTIONS_FOLDER + 'user_2.mot')

        self.beo_picker = BeoPicker(self.objs_features)
        self.beo_picker.pickRandomObject()
        # Game loop
        for i in range(5): # User gets to ask 5 questions
            self.isit_sd.run(process=True)

            if self.player_said.lower() in self.beo_picker.chosen_chars:
                self.beoSpeakMotion("Yes.", MOTIONS_FOLDER + 'uuser_0.mot')
            else:
                self.beoSpeakMotion("No.", MOTIONS_FOLDER + 'uuser_1.mot')


    def darknet_cb(self, info):
        print(info.data)
        if info.data == self.beo_picker.chosen_obj:
            self.dn_detected = True
        else:
            self.dn_detected = False

    def UserGuesser1(self):
        self.beoSpeakMotion("You're out of questions. Show me the object you're thinking of.", MOTIONS_FOLDER + "exp_1.mot")

        rospy.init_node('detect_image')
        # Create a ros subscriber to the darknet node
        self.dn_detected = None
        rospy.Subscriber("/darknet_link/shown_object", String, self.darknet_cb)

        # Wait for object detection
        while self.dn_detected == None: pass

        if self.dn_detected == True:
            self.beoSpeakMotion("Yes, that's the correct object.", MOTIONS_FOLDER + 'uuser_0.mot')
        elif self.dn_detected == False:
            self.beoSpeakMotion("No, I was thinking of another object.", MOTIONS_FOLDER + 'uuser_1.mot')

    def FinalState(self):
        sleep(2)
        self.beoSpeakMotion("Thanks for playing with me.", MOTIONS_FOLDER + "exp_2.mot")

    def run(self):
        # Initial state of the game
        self.state = States.INTRODUCTION

        while True:
            if self.state == States.INTRODUCTION:
                self.IntroductionSequence()

                self.state = States.EXPLANATION

            elif self.state == States.EXPLANATION:
                self.ExplanationSequence()

                self.state = States.CONFIRMATION


            elif self.state == States.CONFIRMATION:
                # Check if Beo should explain one more time or proceed
                if self.ConfirmationSequence():
                   self.state = States.PRE_GAME
                else:
                    self.state = States.EXPLANATION

            elif self.state == States.PRE_GAME:
                self.PregameSequence()
                self.state = States.BEO_GUESSER0

            elif self.state == States.BEO_GUESSER0:
                self.BeoGuesser0()

                self.state = States.USER_GUESSER0

            elif self.state == States.USER_GUESSER0:
                self.UserGuesser0()

                self.state = States.USER_GUESSER1

            elif self.state == States.USER_GUESSER1:
                self.UserGuesser1()

                self.state = States.FINAL_STATE

            elif self.state == States.FINAL_STATE:
                self.FinalState()

                sys.exit(1)


if __name__ == "__main__":
    # Adjust audio
    Popen(['pacmd', 'set-card-profile', 'bluez_card.70_99_1C_02_85_B7', 'a2dp_sink'])
    Popen(['pacmd', 'set-default-source', 'alsa_input.usb-046d_0825_295C05D0-02.analog-mono'])
    sleep(2)
    Popen(['pacmd', 'set-default-sink', 'bluez_sink.70_99_1C_02_85_B7.a2dp_sink'])

    game = IsItGame()
    game.run()

#!/usr/bin/env python

# -*- encoding: UTF-8 -*-

##  @file speech.py
#   @brief File storing information about the speech services.

##  @package speech
#   @brief Package storing information about the speech services.

import sys
import time
import math
import speech_recognition as sr

from gtts import gTTS

from guessing_game_new.srv import * #IMPORT YOUR SERVICES

import rospy
import os

r = sr.Recognizer()
m = sr.Microphone()

## @brief Function handling the speech.
# @param   text             words that should be said
# @return True if the action succeeded.
#
def system_speech(req):
    tts = gTTS(text=req.text, lang='en')
    tts.save("text.mp3")
    os.system("mpg321 text.mp3")
    return text_to_speechResponse(True)

def system_speech_server():
    s = rospy.Service("text_to_speech", text_to_speech, system_speech)

## @brief Function handling the speech.
# @param   text             words that should be said
# @return True if the action succeeded.
#
def text_speech(req):
    flag = False
    while not flag:
        #r.adjust_for_ambiend_noise()
        text_client = rospy.ServiceProxy("text_to_speech", text_to_speech)
        rospy.wait_for_service('text_to_speech')
        text_client("Say something!")
        rospy.wait_for_service('text_to_speech')
        with m as source: audio = r.listen(source)
        #print("Got it! Now to recognize it...")
        try:
            # recognize speech using Google Speech Recognition
            value = r.recognize_google(audio)    
            # we need some special handling here to correctly print unicode characters to standard output
            if str is bytes: # this version of Python uses bytes for strings (Python 2)
                print(u"You said {}".format(value).encode("utf-8"))
            else: # this version of Python uses unicode for strings (Python 3+)
                print("You said {}".format(value))
            flag = True
        except sr.UnknownValueError:
            print("Oops! Didn't catch that")
        except sr.RequestError as e:
            print("Uh oh! Couldn't request results from Google Speech Recognition service; {0}".format(e))
    return speech_to_textResponse(value)

def text_speech_server():
    s = rospy.Service("speech_to_text", speech_to_text, text_speech)

## @brief Initialization function of the node.
#
if __name__ == "__main__":
    rospy.init_node('gg_speech')
    rospy.loginfo("Registered service gg_system_speech.")
    # Advertizing services
    system_speech_server()
    text_speech_server()
    # DO SOME STAFF
    rospy.spin()
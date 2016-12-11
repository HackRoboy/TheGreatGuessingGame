#!/usr/bin/env python

# -*- encoding: UTF-8 -*-

##  @file speech.py
#   @brief File storing information about the speech services.

##  @package speech
#   @brief Package storing information about the speech services.

import os
os.environ['PYTHONPATH'] += ':src/guessing_game_new/scripts'
#print (os.environ['PYTHONPATH'])
from roboy import Roboy

import rospy

from sys import stdin
import shlex
from guessing_game_new.srv import *
import ast

class GuessingGame:
    def __init__(self):
        os.environ['command'] = '/usr/lib/jvm/java-8-openjdk-amd64/bin/java -cp DialogSystem:/home/emilka/new_ws/src/DialogSystem/DialogSystem/resources:/home/emilka/new_ws/src/tocopy/DialogSystem.jar:/home/emilka/new_ws/src/tocopy/commons-codec-1.10.jar:/home/emilka/new_ws/src/tocopy/gson-2.7.jar:/home/emilka/new_ws/src/tocopy/opennlp-tools-1.6.0.jar de.roboy.dialog.DialogSystem'
        assert 'command' in os.environ
        self.roboy = Roboy(shlex.split(os.environ['command']))
        self.roboy.start()
        self.text_client = rospy.ServiceProxy("text_to_speech", text_to_speech)
        self.speech_client = rospy.ServiceProxy("speech_to_text", speech_to_text)
        self.fetch_image = rospy.ServiceProxy("image_request", image_request)

    def ask(self, sentence):
        rospy.loginfo("Starting asking")
        rospy.wait_for_service('text_to_speech')
        response = self.text_client('I heard you said {0}'.format(sentence))
        print ('I heard you said {0}'.format(sentence))
        if response.result:
            ## see something
            try:
                res = self.fetch_image()
                self.roboy.write(sentence, ast.literal_eval(res.response))
            except rospy.ServiceException as exc:
                rospy.loginfo("Service did not process request: " + str(exc))
        else:
            rospy.loginfo("Error")

    def read(self):
        rospy.loginfo("READING")
        ret = []
        for action in self.roboy.read():
            if 'speak' not in action:
                continue
            ret.append(action['speak'])
        return "\n".join(ret)
        

    def run(self):
        while(not rospy.is_shutdown()):
            rospy.loginfo("Start running")
            response = self.text_client(self.read())
            if not response.result:
                rospy.loginfo("ERROR. WARNING. NOOOOO")
            response = self.speech_client()
            #for line in stdin:
            #   line = line.strip("\n")
            rospy.loginfo(response.text)
            self.ask(response.text)
            #response = self.speech_client()
            
            rospy.loginfo("Stop running")
        

## @brief Initialization function of the node.
#
if __name__ == "__main__":
    rospy.init_node('guessing_game')

    rospy.loginfo('going to start roboy guessing game')

    game = GuessingGame()
    game.run()

    rospy.spin()
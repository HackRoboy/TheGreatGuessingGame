#!/usr/bin/env python3

from roboy import Roboy
from sys import stdin

class GuessingGame:
    def __init__(self):
        import os
        import shlex

        assert 'command' in os.environ
        self.roboy = Roboy(shlex.split(os.environ['command']))
        self.roboy.start()

    def ask(self, sentence):
        print('Asking: {0}'.format(sentence))
        self.roboy.write(sentence)

    def read(self):
        return "\n".join([action['speak'] for action in self.roboy.read()])

    def run(self):
        print(self.read())
        for line in stdin:
            line = line.strip("\n")
            print(line)
            self.ask(line)
            print(self.read())
        

if __name__ == '__main__':
    print('going to start roboy guessing game')
    game = GuessingGame()
    game.run()

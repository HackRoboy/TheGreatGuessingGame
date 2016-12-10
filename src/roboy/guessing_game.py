#!/usr/bin/env python3

from roboy import Roboy

class GuessingGame:
    def __init__(self):
        import os
        import shlex

        assert 'command' in os.environ
        self.roboy = Roboy(shlex.split(os.environ['command']))
        self.roboy.start()


if __name__ == '__main__':
    print('going to start roboy guessing game')
    game = GuessingGame()

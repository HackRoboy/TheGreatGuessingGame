#!/usr/bin/env python3

from roboy import Roboy
from sys import stdin

#!/usr/bin/env python3

from roboy import Roboy
from sys import stdin

from std_msgs.msg import String


import ast

class GuessingGame:
    def __init__(self):
        import os
        import shlex

        assert 'command' in os.environ
        self.roboy = Roboy(shlex.split(os.environ['command']))
        self.roboy.start()

    def ask(self, sentence):
        print('Asking: {0}'.format(sentence))
        ## see something
        rospy.wait_for_service('image_request')
        fetch_image = rospy.ServiceProxy("image_request", String)
        try:
            res = fetch_image()
            self.roboy.write(sentence, ast.literal_eval(res))
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: " + str(exc))

    def read(self):
        return "\n".join([action['speak'] for action in self.roboy.read()])

    def run(self):
        print(self.read())
        for line in stdin:
            line = line.strip("\n")
            rospy.loginfo(line)
            self.ask(line)
            rospy.loginfo(self.read())
        

## @brief Initialization function of the node.
#
if __name__ == "__main__":
    rospy.init_node('guessing_game')

    rospy.loginfo('going to start roboy guessing game')

    game = GuessingGame()
    game.run()

    rospy.spin()
    
class GuessingGame:
    def __init__(self):
        import os
        import shlex

        assert 'command' in os.environ
        self.roboy = Roboy(shlex.split(os.environ['command']))
        self.roboy.start()

    def ask(self, sentence):
        self.roboy.write(sentence)

    def read(self):
        return "\n".join([action['speak'] for action in self.roboy.read() if 'speak' in action])

    def run(self):
        print(self.read())
        for line in stdin:
            line = line.strip("\n")
            self.ask(line)
            print(self.read())

if __name__ == '__main__':
    print('going to start roboy guessing game')
    game = GuessingGame()
    game.run()

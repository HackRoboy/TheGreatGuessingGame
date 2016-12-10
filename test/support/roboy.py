class Roboy:
    def __init__(self, command):
        self.command = command
        self.process = None
    
    def start(self):
        from subprocess import Popen, PIPE
        self.process = Popen(self.command, stdout=PIPE, stdin=PIPE)

    def read(self):
        return self.process.stdout.readline().decode('utf-8').strip('\n')

    def write(self, sentence):
        return self.process.stdin.write("{0}\n".format(sentence))

    def ask(self, question):
        self.write(sentence)
        return self.read()

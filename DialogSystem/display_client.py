#!/usr/bin/env python2
import socket
import os

UDP_IP = "138.246.13.62"
UDP_PORT = 10000

lines = os.environ['message'].split('|')
color = None
if 'color' in os.environ:
  color = [int(a) for a in os.environ['color'].split(' ')]
print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT
print "message:", os.environ['message']

message = {}
message['lines'] = lines
if color is not None:
  r, g, b = color[0], color[1], color[2]
  message['color'] = {'red': r, 'green': g, 'blue': b}

import json

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(json.dumps(message), (UDP_IP, UDP_PORT))

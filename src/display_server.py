#!/usr/bin/env python2
import socket
import pyupm_i2clcd as lcd
import json

lcd2 = lcd.Jhd1313m1(0, 0x3E, 0x62)
UDP_IP = "0.0.0.0"
UDP_PORT = 10000
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
  data, addr = sock.recvfrom(10024) # buffer size is 1024 bytes
  data = json.loads(data)
  lines = [] if 'lines' not in data else data['lines']# if '\n' in data else data.split('\\n')
  #lines = data.split('\n') if '\n' in data else data.split('\\n')
  row = 0
  if 'color' in data:
    r, g, b = data['color']['red'], data['color']['green'], data['color']['blue']
    lcd2.setColor(r, g, b)
  for line in lines:
    lcd2.setCursor(row, 0)
    row += 1
    # lcd.write(line.decode('utf-8'))
    print(line)
    lcd2.write(str(line))
    # lcd2.write(bytes(line, encoding='utf-8')) # bytes(line, 'utf-8'))
  print "received message:", data

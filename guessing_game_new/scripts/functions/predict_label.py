#!/usr/bin/env python
# coding: utf8

import subprocess
import sys
import os
import re

path_to_darknet = '/Users/rootmac/Documents/workspace/darknet/'

def predict_label(path):

	# change working dir and run darknet
	os.chdir(path_to_darknet)
	command = './darknet classifier predict cfg/imagenet22k.dataset cfg/extraction.cfg extraction.weights ' + path
	val = subprocess.check_output(command.split(' '))

	# postprocess the results and return
	val = val.split('\n')
	ret = [d for d in val if re.search('[a-z]+: [\d.]+', d)]
	return {d.split(': ')[0] : float(d.split(': ')[1]) for d in ret}


if __name__ == "__main__":

	path = sys.argv[1]
	print(predict_label(path))
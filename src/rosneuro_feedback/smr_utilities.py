#!/usr/bin/python
import cv2
import rospy
import rospkg
import os
import random
import math
import numpy
from rosneuro_msgs.msg import NeuroEvent, NeuroOutput
from std_srvs.srv import Empty
from draw.GUI import SMRGUI


# TODO: Export to rosneuro_events.yaml
OFF = 32768
FIXATION = 786
CFEEDBACK = 781
CLASS_EVENTS = [773, 771]
TARGETHIT = 897
TARGETMISS = 898
EOG = 1024

# TODO: Export to smr_protocol.yaml
CLASSES = ['mi_both_hands', 'mi_both_feet']

def config_trials(n_classes,n_trials):
	sequence = []
	for i in range(n_classes):
		sequence.extend([i]*n_trials)
	random.shuffle(sequence)
	return sequence

def check_exit(key_pressed):
	if key_pressed is -1: return False
	if chr(key_pressed) is 'q': return True

def publish_neuro_event(pub,event):
	msg = NeuroEvent()
	msg.header.stamp = rospy.Time.now()
	msg.event = event
	pub.publish(msg)

def normalize_probabilities(value,max,min):
	nvalue = ((1.0 - 0.0) * (value - min))/(max - min) + 0.0
	nvalue = 1.0 if nvalue >= 1.0 else nvalue
	nvalue = 0.0 if nvalue <= 0.0 else nvalue
	return nvalue

def check_boom(values):
	if any(i >= 1.0 for i in values): return True
	else: return False

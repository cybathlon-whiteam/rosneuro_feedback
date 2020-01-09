#!/usr/bin/env python
from rosneuro_feedback import SmrOffline
import sys
import rospy
import rospkg
import os


def get_mode(idx):
	modality={
			0:'offline',
			1:'online'
		}
	return modality.get(idx,"Unexpected protocol index")

def main():
	sys.argv = rospy.myargv(sys.argv)
	
	rospy.init_node('rosneuro_feedback')
	mode = get_mode(rospy.get_param('~protocol_mode'))

	if mode is 'offline':
		o = SmrOffline()
	elif mode is 'online':
		print('[rosneuro_feedback] TODO')
		return
	else:
		print('[rosneuro_feedback] Unexpected protocol mode')
		return

	o.run()


if __name__ == '__main__':
	main()
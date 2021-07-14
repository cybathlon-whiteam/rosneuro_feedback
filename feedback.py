#!/usr/bin/env python
from rosneuro_feedback import SmrCalibration, SmrEvaluation, SmrControl, SmrControlContinuous
import sys
import rospy
import rospkg
import os


def get_mode(idx):
	modality={
			0:'calibration',
			1:'evaluation',
			2:'control'
			3:'continuous'
		}
	return modality.get(idx,"Unexpected protocol index")

def main():
	sys.argv = rospy.myargv(sys.argv)
	
	rospy.init_node('rosneuro_feedback')
	mode = get_mode(rospy.get_param('~protocol_mode'))

	if mode is 'calibration':
		o = SmrCalibration()
	elif mode is 'evaluation':
		o = SmrEvaluation()
	elif mode is 'control':
		o = SmrControl()
	elif mode is 'continuous':
		o = SmrControlContinuous()
	else:
		print('[rosneuro_feedback] Unexpected protocol mode')
		return

	o.run()


if __name__ == '__main__':
	main()
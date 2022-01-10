#!/usr/bin/env python
from smr_calibration import *
from smr_evaluation import *
from smr_control import *
from smr_control_continuous import *

import sys
import rospy
import rospkg
import os


def get_mode(idx):
	modality={
			0:'calibration',
			1:'evaluation',
			2:'control',
			3:'continuous'
		}
	return modality.get(idx,"Unexpected protocol index")

def main():
	sys.argv = rospy.myargv(sys.argv)
	
	rospy.init_node('rosneuro_feedback')
	mode = get_mode(rospy.get_param('~protocol_mode'))

	if mode == 'calibration':
		o = SmrCalibration()
	elif mode == 'evaluation':
		o = SmrEvaluation()
	elif mode == 'control':
		o = SmrControl()
	elif mode == 'continuous':
		o = SmrControlContinuous()
	else:
		print('[rosneuro_feedback] Unexpected protocol mode')
		return

	o.run()


if __name__ == '__main__':
	main()

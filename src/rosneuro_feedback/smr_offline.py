#!/usr/bin/python

from smr_utilities import *

class SmrOffline(object):
	def __init__(self):

		##### Configure publisher #####
		self.event_pub = rospy.Publisher("/events/bus", NeuroEvent, queue_size=1000)

		##### Configure protocol #####
		self.n_classes = rospy.get_param('~n_classes')
		self.n_trials = rospy.get_param('~n_trials')
		self.threshold = rospy.get_param('~threshold')
		self.values = numpy.zeros(self.n_classes)

		self.timings_begin = rospy.get_param('~timings_begin')
		self.timings_fixation = rospy.get_param('~timings_fixation')
		self.timings_cue = rospy.get_param('~timings_cue')
		self.timings_feedback_min = rospy.get_param('~timings_feedback_min')
		self.timings_feedback_max = rospy.get_param('~timings_feedback_max')
		self.timings_feedback_update = rospy.get_param('~timings_feedback_update')
		self.timings_boom = rospy.get_param('~timings_boom')
		self.timings_iti = rospy.get_param('~timings_iti')
		self.timings_end = rospy.get_param('~timings_end')

	def run(self):
		
		##### Configure protocol #####
		sequence = config_trials(self.n_classes,self.n_trials)

		##### Configure GUI engine #####
		gui = SMRGUI(rospy.get_param('~window_height'),rospy.get_param('~window_width'),rospy.get_param('~window_scale'))
		gui.init_bars(self.n_classes)
		gui.draw()

		print("[smr2class] Protocol starts")
		cv2.waitKey(self.timings_begin)

		exit = False
		for i,idx in enumerate(sequence):
			print("Trial " + str(i+1) + "/" + str(len(sequence)) + " [" + CLASSES[idx] + "]")

			##### Fixation #####
			publish_neuro_event(self.event_pub, FIXATION)
			gui.add_fixation()
			cv2.waitKey(self.timings_fixation)
			publish_neuro_event(self.event_pub, FIXATION+OFF)
			gui.remove_fixation()

			##### Cue #####
			publish_neuro_event(self.event_pub,CLASS_EVENTS[idx])
			gui.add_cue(idx)
			cv2.waitKey(self.timings_cue)
			publish_neuro_event(self.event_pub, CLASS_EVENTS[idx]+OFF)

			##### Continuous feedback #####
			Period = random.randrange(self.timings_feedback_min, self.timings_feedback_max)
			F = 1.0/(4.0 * Period)

			t = 0
			publish_neuro_event(self.event_pub, CFEEDBACK)
			while t < Period:
				value = math.sin(2.0*math.pi*t*F)
				gui.set_value_bars(value, idx)
				t = t + rospy.get_param('~timings_feedback_update')
				if check_exit(cv2.waitKey(self.timings_feedback_update)): exit=True
			publish_neuro_event(self.event_pub, CFEEDBACK+OFF)

			##### Boom #####
			gui.set_alpha_bars(0.8, idx)
			if check_exit(cv2.waitKey(self.timings_boom)): exit=True
			gui.reset_bars()
			gui.remove_cue()

			if check_exit(cv2.waitKey(self.timings_iti)): exit=True
			if exit:
				print("User asked to quit")
				break

		print("[smr2class] Protocol ends")
		cv2.waitKey(self.timings_end)

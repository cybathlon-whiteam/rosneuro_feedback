#!/usr/bin/python

from smr_utilities import *

class SmrOnline(object):
	def __init__(self):

		##### Configure publisher #####
		self.event_pub = rospy.Publisher("/events/bus", NeuroEvent, queue_size=1000)

		##### Configure subscriber #####
		rospy.Subscriber("/smrbci/integrated_neuroprediction", NeuroOutput, self.receive_probabilities)

		##### Initialize probabilities #####
		self.values = numpy.zeros(rospy.get_param('~n_classes'))

	def receive_probabilities(self, msg):
		self.values = msg.softpredict.data

	def reset_bci(self):
		rospy.wait_for_service('/smrbci/reset')
		resbci = rospy.ServiceProxy('/smrbci/reset', Empty)
    		try:
			resbci()
			return True
   		except rospy.ServiceException, e:
        		print "Service call failed: %s"
			return False


	def run(self):
		
		##### Configure protocol #####
		n_classes = rospy.get_param('~n_classes')
		sequence = config_trials(n_classes, rospy.get_param('~n_trials'))

		##### Configure GUI engine #####
		gui = SMRGUI(rospy.get_param('~window_height'),rospy.get_param('~window_width'),rospy.get_param('~window_scale'))
		gui.init_bars(n_classes)
		gui.draw()

		print("[smr2class] Protocol starts")
		cv2.waitKey(rospy.get_param('~timings_begin'))

		exit = False
		for i,idx in enumerate(sequence):
			print("Trial " + str(i+1) + "/" + str(len(sequence)) + " [" + CLASSES[idx] + "]")
			##### Fixation #####
			publish_neuro_event(self.event_pub, FIXATION)
			gui.add_fixation()
			cv2.waitKey(rospy.get_param('~timings_fixation'))
			publish_neuro_event(self.event_pub, FIXATION+OFF)
			gui.remove_fixation()

			##### Cue #####
			publish_neuro_event(self.event_pub,CLASS_EVENTS[idx])
			gui.add_cue(idx)
			cv2.waitKey(rospy.get_param('~timings_cue'))
			publish_neuro_event(self.event_pub, CLASS_EVENTS[idx]+OFF)

			##### Continuous feedback #####
			self.values = numpy.zeros(n_classes)
			hit = False
			self.reset_bci()
			publish_neuro_event(self.event_pub, CFEEDBACK)

			while not hit:
				#rospy.spin()
				
				for c in range(n_classes):
					value = normalize_probabilities(self.values[c], rospy.get_param('~threshold'), 1/n_classes)
					gui.set_value_bars(value, c)
					if value >= 1.0: 
						hit = True
						break
				if check_exit(cv2.waitKey(rospy.get_param('~timings_feedback_update'))): exit=True
			publish_neuro_event(self.event_pub, CFEEDBACK+OFF)

			##### Boom #####
			gui.set_alpha_bars(0.8, c)
			cv2.waitKey(100)
			if c == idx:
				publish_neuro_event(self.event_pub, TARGETHIT)
				if check_exit(cv2.waitKey(rospy.get_param('~timings_boom'))): exit=True
				publish_neuro_event(self.event_pub, TARGETHIT+OFF)
			else:
				publish_neuro_event(self.event_pub, TARGETMISS)
				if check_exit(cv2.waitKey(rospy.get_param('~timings_boom'))): exit=True
				publish_neuro_event(self.event_pub, TARGETMISS+OFF)
			gui.reset_bars()
			gui.remove_cue()

			if check_exit(cv2.waitKey(rospy.get_param('~timings_iti'))): exit=True
			if exit:
				print("User asked to quit")
				break

		print("[smr2class] Protocol ends")
		cv2.waitKey(rospy.get_param('~timings_end'))

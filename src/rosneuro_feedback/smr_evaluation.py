#!/usr/bin/python

from smr_utilities import *

class SmrEvaluation(object):
	def __init__(self):

		##### Configure publisher #####
		self.event_pub = rospy.Publisher("/events/bus", NeuroEvent, queue_size=1000)

		##### Configure subscriber #####
		rospy.Subscriber("/integrator/neuroprediction", NeuroOutput, self.receive_probabilities)

		##### Configure protocol #####
		self.n_classes = rospy.get_param('~n_classes')
		self.n_trials = rospy.get_param('~n_trials')
		str_thr = rospy.get_param('~threshold')
		list_thr = str_thr.split(",")
		self.threshold = []
		for i in list_thr:
			self.threshold.append(float(i))

		self.values = numpy.zeros(self.n_classes)
		self.rec_prob = numpy.zeros(self.n_classes)

		self.timings_begin = rospy.get_param('~timings_begin')
		self.timings_fixation = rospy.get_param('~timings_fixation')
		self.timings_cue = rospy.get_param('~timings_cue')
		self.timings_feedback_update = rospy.get_param('~timings_feedback_update')
		self.timings_boom = rospy.get_param('~timings_boom')
		self.timings_iti = rospy.get_param('~timings_iti')
		self.timings_end = rospy.get_param('~timings_end')

		self.time_checker = feedback_timecheck(self.timings_feedback_update)

	def receive_probabilities(self, msg):
		self.values = msg.softpredict.data

	def reset_bci(self):
		rospy.wait_for_service('/integrator/reset')
		resbci = rospy.ServiceProxy('/integrator/reset', Empty)
		try:
			resbci()
			return True
		except rospy.ServiceException as e:
			print("Service call failed: %s")
			return False

	def run(self):
		
		##### Configure protocol #####
		sequence = config_trials(self.n_classes, self.n_trials)

		##### Configure GUI engine #####
		gui = SMRGUI(rospy.get_param('~window_height'),rospy.get_param('~window_width'),rospy.get_param('~window_scale'))
		gui.init_bars(self.n_classes)
		gui.draw()

		print("[smrbci] Protocol starts")
		cv2.waitKey(self.timings_begin)

		exit = False
		for i,idx in enumerate(sequence):
			print("Trial " + str(i+1) + "/" + str(len(sequence)) + " [" + CLASSES[idx] + "]")
			##### Fixation #####
			publish_neuro_event(self.event_pub, FIXATION)
			gui.add_fixation()
			if check_exit(cv2.waitKey(self.timings_fixation)): exit=True
			publish_neuro_event(self.event_pub, FIXATION+OFF)
			gui.remove_fixation()

			##### Cue #####
			publish_neuro_event(self.event_pub,CLASS_EVENTS[idx])
			gui.add_cue(idx)
			if check_exit(cv2.waitKey(self.timings_cue)): exit=True
			publish_neuro_event(self.event_pub, CLASS_EVENTS[idx]+OFF)

			##### Continuous feedback #####
			self.rec_prob = numpy.zeros(self.n_classes)
			hit = False
			self.reset_bci()
			rospy.sleep(0.150)
			publish_neuro_event(self.event_pub, CFEEDBACK)

			while not hit:
				self.time_checker.make_tic()

				#rospy.spin()
				if abs(self.values[0] - self.rec_prob[0]) > 0.00001:
					self.rec_prob = self.values

					for c in range(self.n_classes):
						value = normalize_probabilities(self.rec_prob[c], self.threshold[c], 1/float(self.n_classes))
						gui.set_value_bars(value, c)
						if value >= 1.0: 
							hit = True
							break
				
				self.time_checker.make_toc()
				delay = self.time_checker.check_delay()
				if delay < 0:
					if check_exit(cv2.waitKey(-delay)): exit=True


			publish_neuro_event(self.event_pub, CFEEDBACK+OFF)

			##### Boom #####
			gui.set_alpha_bars(0.8, c)
			cv2.waitKey(100)
			if c == idx:
				publish_neuro_event(self.event_pub, TARGETHIT)
				if check_exit(cv2.waitKey(self.timings_boom)): exit=True
				publish_neuro_event(self.event_pub, TARGETHIT+OFF)
			else:
				publish_neuro_event(self.event_pub, TARGETMISS)
				if check_exit(cv2.waitKey(self.timings_boom)): exit=True
				publish_neuro_event(self.event_pub, TARGETMISS+OFF)
			gui.reset_bars()
			gui.remove_cue()

			if check_exit(cv2.waitKey(self.timings_iti)): exit=True
			if exit:
				print("User asked to quit")
				break

		print("[smrbci] Protocol ends")
		cv2.waitKey(self.timings_end)

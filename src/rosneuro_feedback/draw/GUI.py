#!/usr/bin/python
import cv2
import numpy 
from Window import Window
from Bar import Bar
from Cue import Cue
from Fixation import Fixation

# SMRGUI class
class SMRGUI:
	def __init__(self, win_height, win_width, win_scale):
		cv2.namedWindow('canvas', cv2.WINDOW_NORMAL)
		cv2.moveWindow('canvas', 0, 0)
		self.window = Window(win_height,win_width,win_scale)
		self.canvas = numpy.zeros((win_height, win_width, 3), numpy.uint8)
		self.bars = []
		self.cue = []
		self.fixation = []

	def init_canvas(self):
		self.canvas = numpy.zeros((self.window.height, self.window.width, 3), numpy.uint8)

	def init_bars(self, num_classes, class_names=None):
        	del self.bars[:] #TO CHECK Python 2 vs Python 3
       		if class_names is not None:
        		for i in range(num_classes):
                		self.bars.append(Bar(self.window, i, class_names[i]))
        	else:
            		for i in range(num_classes):
                		self.bars.append(Bar(self.window, i))

	def set_value_bars(self,value,idx):
		self.bars[idx].set_value(value)
		self.draw()

	def set_alpha_bars(self,alpha,idx):
		self.bars[idx].set_alpha(alpha)
		self.draw()

	def reset_bars(self):
		for bar in self.bars:
			bar.set_value(0.0)
			bar.set_alpha(0.5)
		self.draw()

	def add_cue(self,idx):
		self.cue.append(Cue(self.window,idx))
		self.draw()

	def add_fixation(self):
		self.fixation.append(Fixation(self.window))
		self.draw()

	def remove_cue(self):
		del self.cue[:]
		self.draw()

	def remove_fixation(self):
		del self.fixation[:]
		self.draw()

	def draw(self):
		self.init_canvas()
		for bar in self.bars:
			self.canvas = bar.draw(self.canvas)

		for cue in self.cue:
			self.canvas = cue.draw(self.canvas)

		for fixation in self.fixation:
			self.canvas = fixation.draw(self.canvas)

		canvas = cv2.resize(self.canvas, (self.window.width * self.window.scale, self.window.height * self.window.scale))
		
		cv2.imshow('canvas', canvas)

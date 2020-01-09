#!/usr/bin/python
import cv2
import numpy

FIX_THICK = 10

# Fixation class
class Fixation:
	def __init__(self,window):
		center = (numpy.int32(window.width/2), numpy.int32(window.height/2 + window.height/8))
		size = numpy.int32(window.height/12)
		# vertical line coordinates
		self.vertical_start = (center[0], center[1]-size)
		self.vertical_stop = (center[0], center[1]+size)
		# horizontal line coordinates
		self.horizontal_start = (center[0]-size, center[1])
		self.horizontal_stop = (center[0]+size, center[1])

	def draw(self,canvas):
		# Draw vertical line
		canvas = cv2.line(canvas,self.vertical_start,self.vertical_stop,(255,255,255),FIX_THICK)
		# Draw horizontal line
		canvas = cv2.line(canvas,self.horizontal_start,self.horizontal_stop,(255,255,255),FIX_THICK)
		return canvas
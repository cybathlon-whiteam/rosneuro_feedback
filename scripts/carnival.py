#!/usr/bin/python
import cv2
import math
import rospy
import numpy


# Player class draws just a red dot
class Player:
	def __init__(self, width, height):
		self.width = width
		self.height = height

		self.size = 20
		self.pos = numpy.int32((width / 2, height - 20))

	def draw(self, canvas):
		half_extent = (self.size / 2, self.size / 2)
		pt1 = tuple(self.pos - half_extent)
		pt2 = tuple(self.pos + half_extent)
		cv2.line(canvas, (self.width / 2, 0), tuple(self.pos), (0, 128, 64))
		cv2.rectangle(canvas, pt1, pt2, (0, 0, 255), -1)


# Block to be shooted
class Block:
	def __init__(self, pos):
		block_width = rospy.get_param('~block_width', 40)
		block_height = rospy.get_param('~block_height', 20)
		block_speed = rospy.get_param('~block_speed', 1)

		self.screen_width = rospy.get_param('~screen_width', 640)
		self.size = numpy.int32((block_width, block_height))
		self.pos = numpy.int32(pos)
		self.speed = block_speed

		self.is_missed = False
		self.blink_request = False

	def draw(self, canvas):
		pt1 = tuple(self.pos - self.size / 2)
		pt2 = tuple(self.pos + self.size / 2)
		cv2.rectangle(canvas, pt1, pt2, (255, 255, 255), -1)

	def update(self):
		self.pos[0] -= self.speed

		is_missed = self.pos[0] + self.size[0] < self.screen_width / 2
		self.blink_request = not self.is_missed and is_missed
		self.is_missed = is_missed


# Bullet or cannonball
class Bullet:
	def __init__(self, pos):
		self.size = numpy.int32((1, 4000))
		self.pos = numpy.int32((pos[0], pos[1]))
		self.speed = 500
		self.hit = False

	def draw(self, canvas):
		pt1 = tuple(self.pos - self.size / 2)
		pt2 = tuple(self.pos + self.size / 2)
		cv2.rectangle(canvas, pt1, pt2, (0, 255, 128), 2)

	def update(self):
		self.pos[1] -= self.speed

	def gone(self):
		return self.pos[1] < self.size[1]

	def hit_test(self, block):
		m1 = self.pos - self.size / 2
		m2 = self.pos + self.size / 2
		e1 = block.pos - block.size / 2
		e2 = block.pos + block.size / 2

		ret = m1[0] <= e2[0] and e1[0] <= m2[0] and m1[1] <= e2[1] and e1[1] <= m2[1]

		self.hit = self.hit or ret
		return ret


# Blinking effect for player's mistakes
class Blink:
	def __init__(self):
		self.init_time = rospy.Time.now()
		self.end_time = rospy.Time.now()

		self.blink_speed = rospy.get_param('~blink_speed', 20.0)
		self.blink_times = rospy.get_param('~blink_times', 1)
		self.blink_duration = self.blink_times * 2.0 * math.pi / self.blink_speed

	def draw(self, canvas):
		t = (rospy.Time.now() - self.init_time).to_sec() * self.blink_speed
		p = (math.cos(t) * 0.5 + 0.5) * 0.5

		if self.end_time < rospy.Time.now():
			p = 1

		red = numpy.zeros((canvas.shape), numpy.uint8)
		red[:, :, 2] = 255
		cv2.addWeighted(canvas, p, red, 1 - p, 0.0, canvas)

	def update(self):
		if self.end_time < rospy.Time.now():
			self.init_time = rospy.Time.now()
		self.end_time = rospy.Time.now() + rospy.Duration(self.blink_duration)


# Game class
class Game:
	def __init__(self):
		self.over = False
		self.width = rospy.get_param('~screen_width', 640)
		self.height = rospy.get_param('~screen_height', 480)
		self.scale = rospy.get_param('~screen_scale', 3)
		self.next_block_time = rospy.Time(0)
		self.last_hit_time = rospy.Time(0)

		self.player = Player(self.width, self.height)
		self.blocks = []
		self.bullets = []
		self.blink = Blink()

		self.block_interval_mean = rospy.get_param('~block_interval_mean', 3.5)
		self.block_interval_stddev = rospy.get_param('~block_interval_stddev', 1.0)
		self.block_interval_min = rospy.get_param('~block_interval_min', 2.0)
		self.block_interval_max = rospy.get_param('~block_interval_max', 5.0)

	def is_over(self):
		return self.over

	def time_until_next_block(self):
		time = numpy.random.normal(self.block_interval_mean, self.block_interval_stddev)
		time = min(self.block_interval_max, max(self.block_interval_min, time))
		return rospy.Duration(time)

	def generate_block_cluster(self, width, height):
		block_width = rospy.get_param('~block_width', 40)
		num_blocks = rospy.get_param('~cluster_size', 10)

		pos_x = [width + x * block_width for x in range(num_blocks)]
		blocks = [Block((x, 30)) for x in pos_x]
		return blocks

	def draw(self):
		canvas = numpy.zeros((self.height, self.width, 3), numpy.uint8)

		for block in self.blocks:
			block.draw(canvas)

		for bullet in self.bullets:
			bullet.draw(canvas)

		self.player.draw(canvas)
		self.blink.draw(canvas)

		canvas = cv2.resize(canvas, (canvas.shape[1] * self.scale, canvas.shape[0] * self.scale))
		cv2.imshow('canvas', canvas)

	def update(self):
		# spawn new block
		if rospy.Time.now() > self.next_block_time:
			self.next_block_time = rospy.Time.now() + self.time_until_next_block()
			cluster = self.generate_block_cluster(self.width, self.height)
			self.blocks.extend(cluster)

		# update game elements
		for block in self.blocks:
			block.update()

			if block.blink_request:
				self.blink.update()

		for bullet in self.bullets:
			bullet.update()

			# remove blocks colliding with a bullet
			hit = len([x for x in self.blocks if bullet.hit_test(x)]) > 0
			if hit:
				self.last_hit_time = rospy.Time.now()

			self.blocks = [x for x in self.blocks if not bullet.hit_test(x)]

			if bullet.gone() and not bullet.hit and (rospy.Time.now() - self.last_hit_time) > rospy.Duration(0.25):
				self.blink.update()
		self.bullets = [x for x in self.bullets if not x.gone()]

		# key input
		key = cv2.waitKey(5)
		if key == ord(' '):
			self.bullets.append(Bullet(self.player.pos))
		if key == 0x1b:
			self.over = True


# entry point
def main():
	rospy.init_node('carnival')

	game = Game()
	while not game.is_over() and not rospy.is_shutdown():
		game.draw()
		game.update()


if __name__ == '__main__':
	main()

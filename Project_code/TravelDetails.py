class Calculate_Travel(object):
	# Python3 Program to calculate speed, distance and time
	def __init__(self, dist, time, speed):  # , No_waypoints):
		self.dist = dist
		self.time = time
		self.speed = speed

	# self.No_waypoints = No_waypoints

	# Function to calculate speed
	def cal_speed(self):
		print(" Distance(meters) :", self.dist);
		print(" Time(min) :", self.time);
		return self.dist / self.time;

	# Function to calculate distance
	def cal_dis(self):
		print(" Time(min) :", self.time);
		print(" Speed(m / s) :", self.speed);
		return self.speed * self.time;

	# Function to estimate average flight time
	def cal_time(self):
		# print(" Distance(meters) :", self.dist);
		# print(" Speed(m / s) :", self.speed);
		# turnDelay = self.speed / 20 * (1 + self.speed);
		# print(" Time (min):", (self.dist / self.speed + self.No_waypoints * turnDelay) / 60.0)
		print("Estimated Flight Time (min):", (self.dist / self.speed) / 60, "\n")
		# (dist / speed + WGS84.size() * turnDelay) / 60.0;
		return (self.dist / self.speed) / 60  # divide time value by 60 for minutes
	# return (self.dist / self.speed + self.No_waypoints * turnDelay) / 60.0;

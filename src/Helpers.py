"""
# Created by emmarapt.
"""

import math


def rotate_point(point, angle, center_point):
	"""Rotates a point around center_point(origin by default)
	Angle is in degrees.
	Rotation is counter-clockwise
	"""
	angle_rad = math.radians(angle % 360)
	# Shift the point so that center_point becomes the origin
	new_point = (point[0] - center_point[0], point[1] - center_point[1])
	new_point = (new_point[0] * math.cos(angle_rad) - new_point[1] * math.sin(angle_rad),
				 new_point[0] * math.sin(angle_rad) + new_point[1] * math.cos(angle_rad))
	# Reverse the shifting we have done
	new_point = (new_point[0] + center_point[0], new_point[1] + center_point[1])
	return new_point


def centroid(vertexes):
	x_list = [vertex[0] for vertex in vertexes]
	y_list = [vertex[1] for vertex in vertexes]
	length = len(vertexes)
	x = sum(x_list) / length
	y = sum(y_list) / length
	return x, y


# Calculate bearing between GPS points
def calculate_initial_compass_bearing(pointAlat, pointAlon, pointBlat, pointBlon):
	lat1 = math.radians(pointAlat)
	lat2 = math.radians(pointBlat)

	diffLong = math.radians(pointBlon - pointAlon)

	x = math.sin(diffLong) * math.cos(lat2)
	y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)
										   * math.cos(lat2) * math.cos(diffLong))

	initial_bearing = math.atan2(x, y)

	# Now we have the initial bearing but math.atan2 return values

	# The solution is to normalize the initial bearing as shown below
	initial_bearing = math.degrees(initial_bearing)
	compass_bearing = (initial_bearing + 360) % 360

	return compass_bearing
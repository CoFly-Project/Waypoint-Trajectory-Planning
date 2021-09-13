"""
# Created by emmarapt.
"""

from Helpers import calculate_initial_compass_bearing

import numpy as np
import pyproj


class Turn_waypoints:
	def __init__(self, path, subcells):
		self.path = path
		self.subcells = subcells

	def waypoints_in_turns(self):
		# Projections
		p_ll = pyproj.Proj(init='epsg:4326')  # epsg code for Coordinate reference system  WGS84
		p_mt = pyproj.Proj(init='epsg:3857')  # Web-Mercator Projection for WGS84

		waypointsformission = [0] # starting point
		pathpoints = np.zeros(((len(self.path)), 2))
		bearingofpathpoints = np.zeros(((len(self.path)), 3))

		# --------------
		# choose waypoints in turns
		# --------------

		# Project the points in subcells into an array which follows the path
		j = 0
		for i in self.path:
			waypoints_WGS84_transformed = pyproj.transform(p_mt, p_ll, self.subcells[i][0], self.subcells[i][1])
			pathpoints[j][0] = waypoints_WGS84_transformed[0]
			pathpoints[j][1] = waypoints_WGS84_transformed[1]
			j = j + 1

		j = 0
		for i in range(0, len(pathpoints) - 1, 1):
			bearingofpathpoints[j][0] = i
			bearingofpathpoints[j][1] = i + 1
			bearingofpathpoints[j][2] = round(
				calculate_initial_compass_bearing((pathpoints[i][0]), (pathpoints[i][1]), (pathpoints[i + 1][0]),
												  (pathpoints[i + 1][1])))
			j += 1

		for i in range(len(bearingofpathpoints) - 1):
			# print(bearingofpathpoints[i][2] - bearingofpathpoints[i+1][2])
			if abs(bearingofpathpoints[i][2] - bearingofpathpoints[i + 1][
				2]) > 10:  # or bearingofpathpoints[i][2]!=bearingofpathpoints[i+1][2]:
				waypointsformission.append(int(bearingofpathpoints[i + 1][0]))
		# print(bearingofpathpoints)

		return pathpoints, waypointsformission

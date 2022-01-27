"""
# Created by emmarapt.
"""

from Helpers import rotate_point
from Graph import Graph_Kruskal
from Hamiltonian import HAM
from Pattern import Pattern
from Handle_errors import Handle_errors
from Turn_waypoints import Turn_waypoints

import shapely.geometry
from shapely.geometry.polygon import Polygon
import pyproj
import xml.etree.cElementTree as ET
import numpy as np
import math
import json
import time

"""
		# ------------------------------------------- **STAGE 1** ------------------------------------------------------
"""


# If you have an arbitrary polygon shape at hand, you need to test which of your generated points intersects with
# that poly and only return those coordinate pairs for which this condition holds true. [latitude first,
# longitude second]

class Coverage_Task:

	def __init__(self, stepsize, flight_direction, speed, time_start):
		self.stepsize = stepsize
		self.flight_direction = flight_direction
		self.speed = speed
		self.time_start = time_start

	def STC(self):

		# Projections
		p_ll = pyproj.Proj(init='epsg:4326')  # epsg code for Coordinate reference system  WGS84
		p_mt = pyproj.Proj(init='epsg:3857')  # Web-Mercator Projection for WGS84

		# READ data
		pointList = []  # pointList - Initialize the points inside an area of interest
		with open('map_data.geojson') as json_file:
			data = json.load(json_file)
			for p in data['features']:
				for i in range(len(p['geometry']['coordinates'][0])):
					point = 0
					point = (
						shapely.geometry.Point(
							(p['geometry']['coordinates'][0][i][1], p['geometry']['coordinates'][0][i][0])))
					pointList.append(point)
		# polygonlength = (len(p['geometry']['coordinates'][0]))  # calculate the length of the boundaries

		with open('disabled_paths.geojson') as json_file:
			try:
				data = json.load(json_file)
				Obstacle = [[] for _ in range(
					len(data['features']))]  # Obstacle List - Initialize the points that contain an obstacle
				iterate_obstacles = 0
				for p in data['features']:
					for i in range(len(p['geometry']['coordinates'][0])):
						point = 0
						point = (
							shapely.geometry.Point(
								(p['geometry']['coordinates'][0][i][1], p['geometry']['coordinates'][0][i][0])))
						Obstacle[iterate_obstacles].append(point)
					iterate_obstacles += 1
			except TypeError:
				pass

		# set up the polygon-area
		polygon = Polygon([[p.x, p.y] for p in pointList])  # p.x is lat

		try:
			polygonObst = [[] for _ in range(len(Obstacle))]

			for i in range(len(Obstacle)):
				polygonObst[i].append(Polygon([[p.x, p.y] for p in Obstacle[i]]))
		except:
			pass


		# Create corners of rectangle to be transformed to a grid
		minlonnw = min(p.y for p in pointList)
		minlatnw = min(p.x for p in pointList)
		maxlonsw = max(p.y for p in pointList)
		maxlatsw = max(p.x for p in pointList)

		# Create a rectagular outside the area of interest
		sw = shapely.geometry.Point((minlatnw, minlonnw))
		ne = shapely.geometry.Point((maxlatsw, maxlonsw))

		sw_transformed = pyproj.transform(p_ll, p_mt, sw.x, sw.y)  # Transform SW point to 3857
		ne_transformed = pyproj.transform(p_ll, p_mt, ne.x, ne.y)  # .. same for NE

		gridpoints = []  # points of rectagle area stored in this list
		Geoplaner = []  # points of the area of interest stored in this list

		# Iterate over 2D area
		x = sw_transformed[0]
		while x < ne_transformed[0]:
			y = round(sw_transformed[1])
			while y < ne_transformed[1]:
				# p = shapely.geometry.Point(pyproj.transform(p_mt, p_ll, x, y))
				gridpoints.append((x, y))
				y += self.stepsize  # * sqrt(2)
			x += self.stepsize

		# --------------------------------

		# When in flight direction mode

		# --------------------------------

		if self.flight_direction != 0:

			# Find the centroid of the polygon
			center = polygon.centroid
			center_transformed = pyproj.transform(p_ll, p_mt, center.x, center.y)  # Transform center point to 3857

			# Rotate Poly
			rotatepoly = []
			for p in gridpoints:
				rotatepoly.append(rotate_point((p[0], p[1]), self.flight_direction,
											   (center_transformed[0], center_transformed[1])))
			gridpoints.clear()
			for i in range(len(rotatepoly)):
				gridpoints.append((rotatepoly[i][0], rotatepoly[i][1]))

		numberofobstacles = 0
		numberofpoints = 0  # number of points in poly
		for p in gridpoints:
			p_transform = shapely.geometry.Point(pyproj.transform(p_mt, p_ll, p[0], p[1]))
			point = shapely.geometry.Point(p_transform.x, p_transform.y)  # from all points in rectagular area
			try:  # if obstacles
				if polygon.contains(point) and any([polygonObst[i][0].contains(point) for i in range(
						len(polygonObst))]):  # count megacells that are inside obstacles
					numberofobstacles += 1
				else:
					if polygon.contains(point):
						numberofpoints = numberofpoints + 1
						Geoplaner.append(p)  # list for megacells in EPSG:3857

			except:  # without obstacles
				if polygon.contains(point):
					numberofpoints = numberofpoints + 1
					Geoplaner.append(p)  # list for megacells in EPSG:3857

		i = 0
		messy_x = np.zeros((numberofpoints, 2))
		for p in Geoplaner:
			messy_x[i][0] = p[0]
			messy_x[i][1] = p[1]
			i += 1

		"""
		---------------------------------------
		Paths strictly in ROI = TRUE by default
		---------------------------------------
		Test if all(4) subcells are inside poly - outside obstacles and finalize megacells. STRICT RULE !!! 
		"""
		megacellsinpolygon = []
		messy_numberofwaypoints = 0
		numberofmegacells = 0
		messy_subcells = np.zeros(((numberofpoints * 4), 2))
		distance = (math.sqrt(2) * self.stepsize) / 4
		for i in range(len(messy_x)):
			countsubcells = 0
			for j in range(4):
				if j == 3:
					bearing = 45 + self.flight_direction
				elif j == 0:
					bearing = 135 + self.flight_direction
				elif j == 1:
					bearing = 225 + self.flight_direction
				elif j == 2:
					bearing = 315 + self.flight_direction

				bearing_radians = (bearing * math.pi) / 180
				result_x = messy_x[i][0] + distance * math.cos(bearing_radians)
				result_y = messy_x[i][1] + distance * math.sin(bearing_radians)

				messy_subcells[messy_numberofwaypoints][0] = result_x
				messy_subcells[messy_numberofwaypoints][1] = result_y
				messy_numberofwaypoints += 1
				messy_subcells_transformed = pyproj.transform(p_mt, p_ll, result_x, result_y)
				point = shapely.geometry.Point(messy_subcells_transformed[0], messy_subcells_transformed[1])
				"""CHECK if any subcell is inside obstacles - if yes remove the whole megacell"""
				try:
					if polygon.contains(point) and not any([polygonObst[obs][0].contains(point) for obs in range(
						len(polygonObst))]):
						countsubcells += 1
				except:
					if polygon.contains(point):
						countsubcells += 1

			if countsubcells == 4:
				megacellsinpolygon.append(i)
				numberofmegacells += 1

		megacells = np.zeros((numberofmegacells, 2))

		count = 0
		for i in megacellsinpolygon:
			megacells[count][0] = messy_x[i][0]
			megacells[count][1] = messy_x[i][1]
			count = count + 1


		print("The number of mega cells are %d" % numberofmegacells)
		print("The number of mega cells that contain obstacles are %d\n" % numberofobstacles)

		# Handle possible error
		Handle_errors(numberofmegacells=numberofmegacells, numberofobstacles=numberofobstacles,
					  waypointsformission=None, pathpoints=None,
					  time=None, speed=None).error_3()

		"""
		 --------------------------------
		# Kruskal algorithm
		 --------------------------------
		"""

		g = Graph_Kruskal(numberofmegacells)

		for i in range(len(megacells)):
			for j in range(len(megacells)):
				if i != j:
					if round(math.sqrt(((megacells[i][0] - megacells[j][0]) ** 2) + (
							(megacells[i][1] - megacells[j][1]) ** 2))) == self.stepsize:
						g.addEdgeinadjmatrix(i, j)
					else:
						g.removeEdgeinadjmatrix(i, j)

		# Aplly Kruskal algorithm in matrix megacells which contains the megacells of the area
		for i in range(len(megacells)):
			for j in range(len(megacells)):
				if g.containsEdge(i, j):
					g.addEdge(i, j, 0)
				else:
					g.addEdge(i, j, 1)

		MSTresult = g.KruskalMST()  # The results of Kruskal's algorithm

		"""
		Now calculate the sub-cells for each of the megacells
		"""
		# FOUND THE REAL SUBCELLS INSIDE THE POLY
		subcells = np.zeros((len(megacells) * 4, 2))

		numberofwaypoints = 0
		for i in range(len(megacells)):
			for j in range(4):
				if j == 3:
					bearing = 45 + self.flight_direction
				elif j == 0:
					bearing = 135 + self.flight_direction
				elif j == 1:
					bearing = 225 + self.flight_direction
				elif j == 2:
					bearing = 315 + self.flight_direction
				bearing_radians = (bearing * math.pi) / 180
				result_x = megacells[i][0] + distance * math.cos(bearing_radians)
				result_y = megacells[i][1] + distance * math.sin(bearing_radians)


				subcells[numberofwaypoints][0] = result_x
				subcells[numberofwaypoints][1] = result_y

				numberofwaypoints = numberofwaypoints + 1

		# Geoplaner.com
		root = ET.Element("gpx", version="1.1", creator="http://www.geoplaner.com",
						  xmlns="http://www.topografix.com/GPX/1/1",
						  schemaLocation="http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd")
		rte = ET.SubElement(root, "rte")

		ET.SubElement(rte, "name").text = "Route1"

		"""
		# Call Pattern.py
		"""

		g1 = Pattern(numberofwaypoints, MSTresult, megacells, subcells, self.stepsize)
		pattern = g1.patternforHamiltonianpath()

		g2 = HAM(numberofwaypoints)
		g2.graph = pattern
		path = g2.hamCycle()

		pathpoints, waypointsformission = Turn_waypoints(path=path, subcells=subcells).waypoints_in_turns()

		print("Waypoints for coverage mission: %s\n" % waypointsformission)  # mission ends at the starting point

		# Handle possible error
		Handle_errors(numberofmegacells=None, numberofobstacles=None, waypointsformission=waypointsformission,
					  pathpoints=pathpoints,
					  time=time, speed=self.speed).error_4()

		# For Geoplaner
		for i in waypointsformission:
			elem = ET.SubElement(rte, "rtept", lat=str(pathpoints[i][0]), lon=str(pathpoints[i][1]))
			ET.SubElement(elem, "ele").text = "WP" + str(i)
			ET.SubElement(elem, "name").text = "WP" + str(i)
		tree = ET.ElementTree(root)
		tree.write("pathFollowinginGeo.gpx", xml_declaration="1.0", encoding="UTF-8")

		# Path in .geojson for GUI in STAGE 1
		data = {"SenderID": "Conv-Cao",
				"DataObject": {
					"mission_id": 1,
					"Expected_time": "00.15.00",
					"path": {
						"waypoints": [

							# "latitude" : [],
							# "longitude" : [],
							#
						]

					}
				}
				}

		for i in waypointsformission:
			data["DataObject"]["path"]["waypoints"].append(
				"{latitude: %s" % pathpoints[i][0] + " , longitude : %s }" % pathpoints[i][1])

		# Computation time for calculating path for GUI and pass time in .geojson file
		time_expected = (time.process_time() - self.time_start)
		data["DataObject"]["Expected_time"] = time_expected

		# Write path in file.
		with open('Coverage_path.geojson', 'w') as outfile:
			json.dump(data, outfile)

		return data

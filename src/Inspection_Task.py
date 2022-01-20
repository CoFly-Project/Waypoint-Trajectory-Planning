"""
# Created by emmarapt.
"""

from TravelDetails import Calculate_Travel
from SiteSpecificMission import Site_Specific_mission
from geopy.distance import great_circle

import xml.etree.cElementTree as ET
import geopy as gp
import geopy.distance
import json
import time

"""
		# ------------------------------------------- **STAGE 2** ------------------------------------------------------
"""


class Inspection_Task:

	def __init__(self, time_start):
		self.time_start = time_start


	def TSP(self):
		pointsfor2visit = []  # points that we have to re-visit

		with open('hotpoint_data.geojson') as json_file:
			data = json.load(json_file)
		for p in data['features']:
			initial_drone_position = [p['InitialPosition'][1], p['InitialPosition'][0]]
			pointsfor2visit.append([p['InitialPosition'][1], p['InitialPosition'][0]])
			for i in range(len(p['geometry']['coordinates'][0])):
				pointsfor2visit.append([p['geometry']['coordinates'][0][i][1], p['geometry']['coordinates'][0][i][0]])

		radius = 5  # Radius ( in meters)

		"""Create the distance matrix"""
		distanceTSP = [[0 for column in range(len(pointsfor2visit))] \
					   for row in range(len(pointsfor2visit))]
		dist_list = []
		# distance between pointsfor2visit
		for i in range(len(pointsfor2visit)):
			for j in range(len(pointsfor2visit)):
				if i != j:
					distanceTSP[i][j] = great_circle((pointsfor2visit[i][0], pointsfor2visit[i][1]),
													 (pointsfor2visit[j][0], pointsfor2visit[j][1])).meters
					dist_list.append((i, j, distanceTSP[i][j]))

		# TRAVEL SALESMAN PROBLEM
		data, route = Site_Specific_mission(distanceTSP, pointsfor2visit, initial_drone_position).main()

		# --------------------------------------------------------------------------------------------------------------
		# 										Circular Flight for each point of interest
		# --------------------------------------------------------------------------------------------------------------

		hotpoint = []
		numberof2points = 0
		hotpoint.append((initial_drone_position[0], initial_drone_position[1]))  # Add it for path
		del route[0]  # Do not consider TakeOff/landing point as hotpoint (delete it from list)

		for i in route:

			hotpoint.append((pointsfor2visit[i][0], pointsfor2visit[i][1]))
			distance = geopy.distance.distance(meters=radius)  # Define the radius of the circle
			start = gp.Point(pointsfor2visit[i][0], pointsfor2visit[i][1])
			for j in range(9):
				if j == 0:
					bearing = 0
				elif j == 1:
					bearing = 45
				elif j == 2:
					bearing = 90
				elif j == 3:
					bearing = 135
				elif j == 4:
					bearing = 180
				elif j == 5:
					bearing = 225
				elif j == 6:
					bearing = 270
				elif j == 7:
					bearing = 315
				elif j == 8:
					bearing = 360

				result = distance.destination(start, bearing)  # no radians here

				hotpoint.append((result.latitude, result.longitude))

		# Return to Home Location (TakeOff/landing point)
		hotpoint.append((initial_drone_position[0], initial_drone_position[1]))

		# Calculate distance between Hot-Points (final route)
		distanceTSPhot = 0
		overalldistance = 0
		for i in range(len(hotpoint) - 1):
			distanceTSPhot = great_circle((hotpoint[i][0], hotpoint[i][1]),
										  (hotpoint[i + 1][0], hotpoint[i + 1][1])).meters

			overalldistance = overalldistance + distanceTSPhot

		print("Average Flight Time in minutes",
			  Calculate_Travel(dist=overalldistance, time=time, speed=3).cal_time())

		# Geoplaner.com
		root = ET.Element("gpx", version="1.1", creator="http://www.geoplaner.com",
						  xmlns="http://www.topografix.com/GPX/1/1",
						  schemaLocation="http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd")
		rte = ET.SubElement(root, "rte")

		ET.SubElement(rte, "name").text = "Route1"


		# Write Hotpoints for Geoplaner
		numberofWP = 0
		for Waypoint in hotpoint:
			elem = ET.SubElement(rte, "rtept", lat=str(Waypoint[0]), lon=str(Waypoint[1]))
			ET.SubElement(elem, "ele").text = "WP" + str(numberofWP)
			ET.SubElement(elem, "name").text = "WP" + str(numberofWP)
			numberofWP += 1
		tree = ET.ElementTree(root)
		tree.write("pathFollowinginGeo.gpx", xml_declaration="1.0", encoding="UTF-8")

		# Path in .geojson for GUI in STAGE 2
		data = {"SenderID": "Conv-Cao",
				"DataObject": {
					"mission_id": 2,
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

		for i in range(len(hotpoint)):
			data["DataObject"]["path"]["waypoints"].append(
				"{latitude: %s" % hotpoint[i][0] + " , longitude : %s }" % hotpoint[i][1])

		# Computation time for calculating path for GUI & Gazebo simulator and pass time in .geojson file
		time_expected = (time.process_time() - self.time_start)
		data["DataObject"]["Expected_time"] = time_expected

		# Write path in file.
		with open('Hotpoint_path.geojson', 'w') as outfile:
			json.dump(data, outfile)

		"""
		# These files contain all the data for the HOTPOINT-MISSION that simulator needs to run
		"""

		# with open('Stage2LAT', 'w') as filehandle:  # file that saves HOTPOINT's Latitude for simulator
		# 	for i in range(len(hotpoint)):
		# 		filehandle.write(('%s,' % hotpoint[i][0]))

		# with open('Stage2LON', 'w') as filehandle:  # file that saves HOTPOINT's Longitude for simulator
		# 	for i in range(len(hotpoint)):
		# 		filehandle.write(('%s,' % hotpoint[i][1]))
		#
		# HotpointHeight = np.zeros((len(hotpoint)))
		# HotpointHeading = np.zeros((len(hotpoint)))  # this is the heading of the drone in degrees
		# for i in range(len(HotpointHeight)):
		# 	HotpointHeight[i] = 5  # this is the height of the drone in meters
		#
		# with open('Stage2HEIGHT', 'w') as filehandle:
		# 	for i in range(len(HotpointHeight)):
		# 		filehandle.write(('%s,' % HotpointHeight[i]))
		#
		# with open('Stage2Heading', 'w') as filehandle:
		# 	for i in range(len(HotpointHeading)):
		# 		filehandle.write(('%s,' % HotpointHeading[i]))

		return data

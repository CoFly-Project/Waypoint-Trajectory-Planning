"""
# Created by emmarapt.
"""

from geopy.distance import great_circle
from requests.exceptions import HTTPError
from TravelDetails import Calculate_Travel

import requests
import sys


# Handle errors during the path planning procedure - Return them to GUI in order to be properly displayed by iknowhow
class Handle_errors:
	def __init__(self, numberofmegacells, Obstacle, waypointsformission, pathpoints, time, speed):
		self.numberofmegacells = numberofmegacells
		self.Obstacle = Obstacle
		self.waypointsformission = waypointsformission
		self.pathpoints = pathpoints
		self.time = time
		self.speed = speed

	def error2(self):
		data = {"Pair Scanning Distance - flight direction create conflict / Solution does not exist - Try again"}
		# Sending POST data to IKH with requests
		headers = {'Content-Type': 'application/json', }
		for url in ['http://127.0.0.1:8081/calculation_path_error?error_code=1']:  # HERE PUT THE URL FOR IKH
			try:
				responsepost = requests.post(url, json=data, headers=headers)
				responsepost.raise_for_status()
			except HTTPError as http_err:
				status_code = http_err.response.status_code
				print('HTTP error occurred: Status_code {', status_code, '}')
			except Exception as err:
				print('Other error occurred: {err = 1}')
			else:
				print('Communication - Success!')
				print('Error Code = 1')
			sys.exit(0)

	def error_3(self):
		if self.numberofmegacells - len(self.Obstacle) <= 3:
			data = {"Cannot find path because the area is very small - Minimize Scanning Distance and try again"}
			# Sending POST data to IKH with requests
			headers = {'Content-Type': 'application/json', }
			for url in ['http://127.0.0.1:8081/calculation_path_error?error_code=3']:  # HERE PUT THE URL FOR IKH
				try:
					responsepost = requests.post(url, json=data, headers=headers)
					responsepost.raise_for_status()
				except HTTPError as http_err:
					status_code = http_err.response.status_code
					print('HTTP error occurred: Status_code {', status_code, '}')
				except Exception as err:
					print('Other error occurred: {err = 3}')
				else:
					print('Communication Success!')
					print('Error Code = 3')
			sys.exit(0)

	def error_4(self):

		Distance_for_path = 0
		for i in range(len(self.waypointsformission) - 1):
			Distance_for_path += great_circle(
				(self.pathpoints[self.waypointsformission[i]][1], self.pathpoints[self.waypointsformission[i]][0]),
				(self.pathpoints[self.waypointsformission[i + 1]][1], self.pathpoints[self.waypointsformission[i + 1]][0])).meters

		if Calculate_Travel(dist=Distance_for_path, time=self.time, speed=self.speed).cal_time() > 20:
			data = {"Cannot execute mission due to time limitation (20 minutes)"}
			# Sending POST data to IKH with requests
			headers = {'Content-Type': 'application/json', }
			for url in ['http://127.0.0.1:8081/calculation_path_error?error_code=4']:  # HERE PUT THE URL FOR IKH
				try:
					responsepost = requests.post(url, json=data, headers=headers)
					responsepost.raise_for_status()
				except HTTPError as http_err:
					status_code = http_err.response.status_code
					print('HTTP error occurred: Status_code {', status_code, '}')
				except Exception as err:
					print('Other error occurred: {err = 4}')
				else:
					print('Communication Success!')
					print('Error Code = 4')
			sys.exit(0)

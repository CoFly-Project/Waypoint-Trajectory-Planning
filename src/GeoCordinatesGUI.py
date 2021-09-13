"""
# Created by emmarapt.
"""

from Coverage_Task import Coverage_Task
from Inspection_Task import Inspection_Task

import requests
from requests.exceptions import HTTPError
import time
import sys
import warnings

# Start processing time
time_start = time.process_time()

# Ignore Future & Deprecation Warnings
warnings.filterwarnings("ignore", category=FutureWarning)
warnings.filterwarnings("ignore", category=DeprecationWarning)

"""
Choose STAGE 1 or STAGE 2 (Compatible with Gazebo-Simulator)
"""
STAGE = int(sys.argv[5])  # Choose your STAGE: STAGE = 1 For Coverage_Task / STAGE = 2 For Inspection_Task

if STAGE == 1:

	flight_direction = int(sys.argv[3])  # rotation
	stepsize = int(sys.argv[4])  # Scanning Distance ( in meters)
	speed = int(sys.argv[6])

	data = Coverage_Task(stepsize=stepsize, flight_direction=flight_direction, speed=speed, time_start=time_start).STC()


elif STAGE == 2:

	data = Inspection_Task(time_start=time_start).TSP()


# Sending POST data to GUI with requests
headers = {'Content-Type': 'application/json', }
for url in ['http://localhost:8081/calculated_path']:  # HERE PUT THE URL FOR IKH
	try:
		responsepost = requests.post(url, json=data, headers=headers)
		# responseget = requests.get(url)
		# response2 = requests.post(url, data = payload)

		# If the response was successful, no Exception will be raised
		# responseget.raise_for_status()
		responsepost.raise_for_status()
	except HTTPError as http_err:
		status_code = http_err.response.status_code
		print('HTTP error occurred: Status_code {', status_code, '}')
	except Exception as err:
		print('Other error occurred: { Port is closed }')
	else:
		print('Communication - Success!')

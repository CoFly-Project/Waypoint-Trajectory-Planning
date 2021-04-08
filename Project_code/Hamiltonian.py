# Python program for solution of hamiltonian cycle problem

from requests.exceptions import HTTPError
import requests
import sys


class HAM():

	def __init__(self, vertices):
		self.graph = [[0 for column in range(vertices)] \
					  for row in range(vertices)]
		self.V = vertices

	''' Check if this vertex is an adjacent vertex
    of the previously added vertex and is not
    included in the path earlier '''

	def isSafe(self, v, pos, path):
		# Check if current vertex and last vertex
		# in path are adjacent
		if self.graph[path[pos - 1]][v] == 0:
			return False

		# Check if current vertex not already in path
		for vertex in path:
			if vertex == v:
				return False

		return True

	# A recursive utility function to solve
	# hamiltonian cycle problem
	def hamCycleUtil(self, path, pos):

		# base case: if all vertices are
		# included in the path
		if pos == self.V:
			# Last vertex must be adjacent to the
			# first vertex in path to make a cycle
			if self.graph[path[pos - 1]][path[0]] == 1:
				return True
			else:
				return False

		# Try different vertices as a next candidate
		# in Hamiltonian Cycle. We don't try for 0 as
		# we included 0 as starting point in in hamCycle()
		for v in range(1, self.V):

			if self.isSafe(v, pos, path) == True:

				path[pos] = v

				if self.hamCycleUtil(path, pos + 1) == True:
					return True

				# Remove current vertex if it doesn't
				# lead to a solution
				path[pos] = -1

		return False

	def hamCycle(self):
		path = [-1] * self.V

		''' Let us put vertex 0 as the first vertex
            in the path. If there is a Hamiltonian Cycle,
            then the path can be started from any point
            of the cycle as the graph is undirected '''
		path[0] = 0
		if self.hamCycleUtil(path, 1) == False:
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
					sys.exit(0)
				except Exception as err:
					print('Other error occurred: {err = 1}')
					sys.exit(0)
				else:
					print('Communication - Success!')
					print('Error Code = 1')
					sys.exit(0)

			return False

		self.printSolution(path)
		return path

	def printSolution(self, path):
		print("** Solution Exists: Now you can display path in map **\n")
		# for vertex in path:
		#   print (vertex,
		# path[0],"\n")

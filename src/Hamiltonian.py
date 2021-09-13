"""
# Created by emmarapt.
"""

# Python program for Hamiltonian Cycle problem

from requests.exceptions import HTTPError
from Handle_errors import Handle_errors
import requests
import sys


class HAM():

	def __init__(self, vertices):
		self.graph = [[0 for column in range(vertices)] \
					  for row in range(vertices)]
		#self.graph = Adj_subcells
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
			Handle_errors(numberofmegacells=None, Obstacle=None, waypointsformission=None,
						  pathpoints=None, time=None, speed=None).error_3()

		self.printSolution(path)
		return path

	def printSolution(self, path):
		print("** Solution Exists: Now you can display path in map **\n")
		# for vertex in path:
		#   print (vertex,
		# path[0],"\n")

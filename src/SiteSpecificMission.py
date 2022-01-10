"""
# Created by emmarapt.
"""

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


class Site_Specific_mission(object):
	def __init__(self, distanceTSP, pointsfor2visit, initial_drone_position):
		self.distanceTSP = distanceTSP
		self.pointsfor2visit = pointsfor2visit
		self.initial_drone_position = initial_drone_position
		self.No_initial_drone_position = [i for i in range(len(self.pointsfor2visit)) if self.pointsfor2visit[i] == self.initial_drone_position]

	def create_data_model(self):
		data = {'distance_matrix': self.distanceTSP, 'takeoff/landing': self.No_initial_drone_position[0]}
		return data

	def print_solution(self, manager, routing, solution):
		"""Prints solution on console."""
		route = []
		max_route_distance = 0
		total_route_distances = 0
		index = routing.Start(0)
		plan_output = 'Calculating route ...\n'
		plan_output += 'Optimal route for Site-Specific Mission:'
		route_distance = 0
		while not routing.IsEnd(index):
			route.append(manager.IndexToNode(index))
			plan_output += ' {} -> '.format(manager.IndexToNode(index))
			previous_index = index
			index = solution.Value(routing.NextVar(index))
			route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
		plan_output += '{}\n'.format(manager.IndexToNode(index))
		plan_output += 'Distance of the route: {}m\n'.format(route_distance)
		print(plan_output)
		max_route_distance = max(route_distance, max_route_distance)
		total_route_distances += max(route_distance, max_route_distance)
		return route

	def main(self):
		# create_data_model
		data = self.create_data_model()
		manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), 1, data['takeoff/landing'])

		# Create Routing Model.
		routing = pywrapcp.RoutingModel(manager)

		# Create and register a transit callback.
		def distance_callback(from_index, to_index):
			"""Returns the distance between the two nodes."""
			# Convert from routing variable Index to distance matrix NodeIndex.
			from_node = manager.IndexToNode(from_index)
			to_node = manager.IndexToNode(to_index)
			return data['distance_matrix'][from_node][to_node]

		transit_callback_index = routing.RegisterTransitCallback(distance_callback)

		# Define cost of each arc.
		routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

		# Add Distance constraint.
		dimension_name = 'Distance'
		routing.AddDimension(
			transit_callback_index,
			0,  # no slack
			3000,  # vehicle maximum travel distance
			True,  # start cumul to zero
			dimension_name)
		distance_dimension = routing.GetDimensionOrDie(dimension_name)
		distance_dimension.SetGlobalSpanCostCoefficient(100)

		# Setting first solution heuristic.
		search_parameters = pywrapcp.DefaultRoutingSearchParameters()
		search_parameters.first_solution_strategy = (
		    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

		# Setting more optimal solution.
		# search_parameters = pywrapcp.DefaultRoutingSearchParameters()
		# search_parameters.local_search_metaheuristic = (
		# 	routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
		# search_parameters.time_limit.seconds = 1
		# search_parameters.log_search = True

		# Solve the problem.
		solution = routing.SolveWithParameters(search_parameters)

		# Print solution on console.
		if solution:
			route = self.print_solution(manager, routing, solution)
		else:
			print("Solution does not exists - Possible error with map_data")
		return data, route


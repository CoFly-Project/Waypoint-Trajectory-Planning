from geopy.distance import great_circle
import math


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


class Pattern(object):
    # pattern to solve hamiltonian path problem in linear time
    def __init__(self, numberofwaypoints, MSTresult, x, subcells, flight_direction):
        self.numberofwaypoints = numberofwaypoints
        self.MSTresult = MSTresult
        self.x = x
        self.subcells = subcells
        self.flight_direction = flight_direction

    def patternforHamiltonianpath(self):

        # Nested list comprehension
        pattern = [[0 for column in range(self.numberofwaypoints)] \
                   for row in range(self.numberofwaypoints)]

        # create an Adj matrix self.x for self.subcells

        for i in range(self.numberofwaypoints):
            for j in range(self.numberofwaypoints):

                if i != j:
                    if (round(great_circle((self.subcells[i][0], self.subcells[i][1]),
                                           (self.subcells[j][0], self.subcells[j][1])).meters) == round(
                        great_circle((self.subcells[0][0], self.subcells[0][1]),
                                     (self.subcells[1][0], self.subcells[1][1])).meters)) or (round(
                        great_circle((self.subcells[i][0], self.subcells[i][1]),
                                     (self.subcells[j][0], self.subcells[j][1])).meters) == round(
                        great_circle((self.subcells[0][0], self.subcells[0][1]),
                                     (self.subcells[1][0], self.subcells[1][1])).meters) + 1) or (round(
                        great_circle((self.subcells[i][0], self.subcells[i][1]),
                                     (self.subcells[j][0], self.subcells[j][1])).meters) == round(
                        great_circle((self.subcells[0][0], self.subcells[0][1]),
                                     (self.subcells[1][0], self.subcells[1][1])).meters) - 1) or (round(
                        great_circle((self.subcells[i][0], self.subcells[i][1]),
                                     (self.subcells[j][0], self.subcells[j][1])).meters) == round(
                        great_circle((self.subcells[0][0], self.subcells[0][1]),
                                     (self.subcells[1][0], self.subcells[1][1])).meters) - 2) or (round(
                        great_circle((self.subcells[i][0], self.subcells[i][1]),
                                     (self.subcells[j][0], self.subcells[j][1])).meters) == round(
                        great_circle((self.subcells[0][0], self.subcells[0][1]),
                                     (self.subcells[1][0], self.subcells[1][1])).meters) + 2) or (round(
                        great_circle((self.subcells[i][0], self.subcells[i][1]),
                                     (self.subcells[j][0], self.subcells[j][1])).meters) == round(
                        great_circle((self.subcells[0][0], self.subcells[0][1]),
                                     (self.subcells[1][0], self.subcells[1][1])).meters) + 3) or (round(
                        great_circle((self.subcells[i][0], self.subcells[i][1]),
                                     (self.subcells[j][0], self.subcells[j][1])).meters) == round(
                        great_circle((self.subcells[0][0], self.subcells[0][1]),
                                     (self.subcells[1][0], self.subcells[1][1])).meters) - 3):
                        pattern[i][j] = 1
                        pattern[j][i] = 1

        for u, v, weight in self.MSTresult:
            if weight == 0 or weight == 1:

                if round(calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                           (self.x[v][1]))) == (
                        90 + self.flight_direction) % 360 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (
                        (90 + self.flight_direction) % 360) - 1 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (
                        (90 + self.flight_direction) % 360) + 1 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (
                        (90 + self.flight_direction) % 360) - 2 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (
                        (90 + self.flight_direction) % 360) + 2 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (
                        (90 + self.flight_direction) % 360) + 3 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (
                        (90 + self.flight_direction) % 360) - 3 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (270 + self.flight_direction) % 360 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (
                        (270 + self.flight_direction) % 360) - 1 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (
                        (270 + self.flight_direction) % 360) + 1 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (
                        (270 + self.flight_direction) % 360) - 2 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (
                        (270 + self.flight_direction) % 360) + 2 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (
                        (270 + self.flight_direction) % 360) + 3 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == ((270 + self.flight_direction) % 360) - 3:

                    for i in range(self.numberofwaypoints):
                        if round(
                                calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.subcells[i][0]),
                                                                  (self.subcells[i][1]))) == (
                                135 + self.flight_direction) % 360:
                            for j in range(self.numberofwaypoints):
                                if round(
                                        calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]),
                                                                          (self.subcells[j][0]),
                                                                          (self.subcells[j][
                                                                              1]))) == (
                                        45 + self.flight_direction) % 360:
                                    pattern[i][j] = 0
                                    pattern[j][i] = 0

                    for i in range(self.numberofwaypoints):
                        if round(
                                calculate_initial_compass_bearing((self.x[v][0]), (self.x[v][1]), (self.subcells[i][0]),
                                                                  (self.subcells[i][1]))) == (
                                225 + self.flight_direction) % 360:
                            for j in range(self.numberofwaypoints):
                                if round(
                                        calculate_initial_compass_bearing((self.x[v][0]), (self.x[v][1]),
                                                                          (self.subcells[j][0]),
                                                                          (self.subcells[j][
                                                                              1]))) == (
                                        315 + self.flight_direction) % 360:
                                    pattern[i][j] = 0
                                    pattern[j][i] = 0

                if round(calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                           (self.x[v][1]))) == (
                        0 + self.flight_direction) % 360 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (
                        (0 + self.flight_direction) % 360) + 1 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (
                        (0 + self.flight_direction) % 360) - 1 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (
                        (0 + self.flight_direction) % 360) - 2 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (
                        (0 + self.flight_direction) % 360) + 2 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (
                        (0 + self.flight_direction) % 360) + 3 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (
                        (0 + self.flight_direction) % 360) - 3 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (180 + self.flight_direction) % 360 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (
                        (180 + self.flight_direction) % 360) + 1 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (
                        (180 + self.flight_direction) % 360) - 1 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (
                        (180 + self.flight_direction) % 360) + 2 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (
                        (180 + self.flight_direction) % 360) - 2 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == (
                        (180 + self.flight_direction) % 360) - 3 or round(
                    calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.x[v][0]),
                                                      (self.x[v][1]))) == ((180 + self.flight_direction) % 360) + 3:

                    for i in range(self.numberofwaypoints):
                        if round(
                                calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]), (self.subcells[i][0]),
                                                                  (self.subcells[i][1]))) == (
                                45 + self.flight_direction) % 360:
                            for j in range(self.numberofwaypoints):
                                if round(
                                        calculate_initial_compass_bearing((self.x[u][0]), (self.x[u][1]),
                                                                          (self.subcells[j][0]),
                                                                          (self.subcells[j][
                                                                              1]))) == (
                                        315 + self.flight_direction) % 360:
                                    pattern[i][j] = 0
                                    pattern[j][i] = 0

                    for i in range(self.numberofwaypoints):
                        if round(
                                calculate_initial_compass_bearing((self.x[v][0]), (self.x[v][1]), (self.subcells[i][0]),
                                                                  (self.subcells[i][1]))) == (
                                135 + self.flight_direction) % 360:
                            for j in range(self.numberofwaypoints):
                                if round(
                                        calculate_initial_compass_bearing((self.x[v][0]), (self.x[v][1]),
                                                                          (self.subcells[j][0]),
                                                                          (self.subcells[j][
                                                                              1]))) == (
                                        225 + self.flight_direction) % 360:
                                    pattern[i][j] = 0
                                    pattern[j][i] = 0

        return pattern

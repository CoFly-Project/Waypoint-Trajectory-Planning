"""
# Created by emmarapt.
"""

from Intersection import Pointpattern, doIntersect

import math
import numpy as np


# Circumnavigation clockwise --------------------------------------------------
class Pattern(object):
    # pattern to solve hamiltonian path problem in linear time
    def __init__(self, numberofwaypoints, MSTresult, megacells, subcells, stepsize):
        self.numberofwaypoints = numberofwaypoints
        self.MSTresult = MSTresult
        self.megacells = megacells
        self.subcells = subcells
        self.stepsize = stepsize


    def patternforHamiltonianpath(self):

        Adj_subcells = np.zeros((self.numberofwaypoints, self.numberofwaypoints))
        for i in range(len(self.subcells)):
            p1 = Pointpattern(self.subcells[i][0], self.subcells[i][1])
            for j in range(len(self.subcells)):
                if round(math.sqrt(((self.subcells[i][0] - self.subcells[j][0]) ** 2) + (
                        (self.subcells[i][1] - self.subcells[j][1]) ** 2))) == round(self.stepsize / 2) or round(
                    math.sqrt(((self.subcells[i][0] - self.subcells[j][0]) ** 2) + (
                            (self.subcells[i][1] - self.subcells[j][1]) ** 2))) == round(self.stepsize / 2) - 1 or round(
                    math.sqrt(((self.subcells[i][0] - self.subcells[j][0]) ** 2) + (
                            (self.subcells[i][1] - self.subcells[j][1]) ** 2))) == round(self.stepsize / 2) + 1:

                    intersection = []

                    q1 = Pointpattern(self.subcells[j][0], self.subcells[j][1])
                    for u, v, weight in self.MSTresult:
                        p2 = Pointpattern(self.megacells[u][0], self.megacells[u][1])
                        q2 = Pointpattern(self.megacells[v][0], self.megacells[v][1])
                        if doIntersect(p1, q1, p2, q2) is False:
                            intersection.append(False)
                        else:
                            intersection.append(True)

                    if all(elem is False for elem in intersection):
                        # find where they belong
                        for k in range(len(self.megacells)):
                            if round(math.sqrt(((self.subcells[i][0] - self.megacells[k][0]) ** 2) + (
                                    (self.subcells[i][1] - self.megacells[k][1]) ** 2))) == round((math.sqrt(2) * self.stepsize) / 4):
                                megacell_1 = k
                            if round(math.sqrt(((self.subcells[j][0] - self.megacells[k][0]) ** 2) + (
                                    (self.subcells[j][1] - self.megacells[k][1]) ** 2))) == round((math.sqrt(2) * self.stepsize) / 4):
                                megacell_2 = k

                        if megacell_1 == megacell_2:
                            Adj_subcells[i][j] = 1
                            Adj_subcells[j][i] = 1
                        elif megacell_1 != megacell_2:
                            if [megacell_1, megacell_2, 0] or [megacell_2, megacell_1, 0] in self.MSTresult:
                                Adj_subcells[i][j] = 1
                                Adj_subcells[j][i] = 1
                else:
                    Adj_subcells[i][j] = 0
                    Adj_subcells[j][i] = 0

        return Adj_subcells

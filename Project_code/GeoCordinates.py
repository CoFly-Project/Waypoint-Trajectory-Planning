"""
 Created by emmarapt.
"""

from Graph import Graph
from Hamiltonian import HAM
from Pattern import Pattern
from TravelDetails import Calculate_Travel
from SiteSpecificMission import Site_Specific_mission
from Intersection import Pointpattern, doIntersect

from pyproj import _datadir, datadir
import shapely.geometry
from shapely.geometry.polygon import Polygon
import pyproj
import xml.etree.cElementTree as ET
import numpy as np
import geopy as gp
import geopy.distance
from geopy.distance import great_circle
import math
from math import sqrt
import json
import requests
from requests.exceptions import HTTPError
import time
import sys
import warnings
# Ignore Future & Deprecation Warnings
warnings.filterwarnings("ignore", category=FutureWarning)
warnings.filterwarnings("ignore", category=DeprecationWarning)

# Start time
time_start = time.process_time()


def rotate_point(point, angle, center_point=(0, 0)):
    """Rotates a point around center_point(origin by default)
    Angle is in degrees.
    Rotation is counter-clockwise
    """
    angle_rad = math.radians(angle % 360)
    # Shift the point so that center_point becomes the origin
    new_point = (point[0] - center_point[0], point[1] - center_point[1])
    new_point = (new_point[0] * math.cos(angle_rad) - new_point[1] * math.sin(angle_rad),
                 new_point[0] * math.sin(angle_rad) + new_point[1] * math.cos(angle_rad))
    # Reverse the shifting we have done
    new_point = (new_point[0] + center_point[0], new_point[1] + center_point[1])
    return new_point


def centroid(vertexes):
    x_list = [vertex[0] for vertex in vertexes]
    y_list = [vertex[1] for vertex in vertexes]
    length = len(vertexes)
    x = sum(x_list) / length
    y = sum(y_list) / length
    return (x, y)


# Calculate bearing between GPS points
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


# If you have an arbitrary polygon shape at hand, you need to test which of your generated points intersects with
# that poly and only return those coordinate pairs for which this condition holds true. [latitude first,
# longitude second]

"""
Choose STAGE 1 (Coverage mission) or STAGE 2 (Site-Specific mission) for the Co-Fly Project 
"""
STAGE = int(sys.argv[5])  # Choose your STAGE

if STAGE == 1:

    # get argument list using sys module
    with open(sys.argv[1]) as json_data:
        try:
            mapdatafromIKH = json.load(json_data)
        except ValueError:
            mapdatafromIKH = None

    with open('map_data.geojson', 'w') as outfile:
        json.dump(mapdatafromIKH, outfile)

    with open(sys.argv[2]) as json_data:
        try:
            obstacledatafromIKH = json.load(json_data)
        except ValueError:
            obstacledatafromIKH = None

    with open('disabled_paths.geojson', 'w') as outfile:
        json.dump(obstacledatafromIKH, outfile)

    flight_direction = int(sys.argv[3])  # node rotation
    stepsize = int(sys.argv[4])  # Scanning Distance (in meters)
    speed = int(sys.argv[6])

    # Projections
    p_ll = pyproj.Proj(init='epsg:4326')  # epsg code for Coordinate reference system  WGS84
    p_mt = pyproj.Proj(init='epsg:3857')  # Web-Mercator Projection for WGS84

    pointList = []  # Point List - Initialize the points inside an area of interest
    with open('map_data.geojson') as json_file:
        data = json.load(json_file)
        for p in data['features']:
            for i in range(len(p['geometry']['coordinates'][0])):
                point = (
                    shapely.geometry.Point(
                        (p['geometry']['coordinates'][0][i][1], p['geometry']['coordinates'][0][i][0])))
                pointList.append(point)
                # polygonlength = (len(p['geometry']['coordinates'][0]))  # calculate the length of the boundaries

    Obstacle = []  # Obstacle List - Initialize the points that contain an obstacle
    with open('disabled_paths.geojson') as json_file:
        try:
            data = json.load(json_file)
            for p in data['features']:
                for i in range(len(p['geometry']['coordinates'][0])):
                    point = (
                        shapely.geometry.Point(
                            (p['geometry']['coordinates'][0][i][1], p['geometry']['coordinates'][0][i][0])))
                    Obstacle.append(point)
        except TypeError:
            pass

    # set up the polygon-area
    polygon = Polygon([[p.y, p.x] for p in pointList])  # p.y is lat / p.x is lon
    polygonObst = Polygon([[p.y, p.x] for p in Obstacle])

    # Create corners of rectangle to be transformed to a grid
    minlonnw = 100
    minlatnw = 100
    maxlonsw = 0
    maxlatsw = 0
    for p in pointList:
        if p.x < minlonnw:
            minlonnw = p.x
        if p.y < minlatnw:
            minlatnw = p.y
        if p.x > maxlonsw:
            maxlonsw = p.x
        if p.y > maxlatsw:
            maxlatsw = p.y

    # Create a rectagular outside the area of interest
    sw = shapely.geometry.Point((minlatnw, minlonnw))
    ne = shapely.geometry.Point((maxlatsw, maxlonsw))

    s = pyproj.transform(p_ll, p_mt, sw.x, sw.y)  # Transform NW point to 3857
    e = pyproj.transform(p_ll, p_mt, ne.x, ne.y)  # .. same for SE

    gridpoints = []
    Geoplaner = []
    # Iterate over 2D area
    x = s[0]
    while x < e[0]:
        y = round(s[1])
        while y < e[1]:
            p = shapely.geometry.Point(pyproj.transform(p_mt, p_ll, x, y))
            gridpoints.append(p)
            y += stepsize * sqrt(2)
        x += stepsize

    numberofpoints = 0  # number of points in poly

    for p in gridpoints:
        point = shapely.geometry.Point(p.x, p.y)  # from all points in rectagular area
        if polygon.contains(point) and not polygonObst.contains(
                point):  # choose those which are in intersects with that polygon
            numberofpoints = numberofpoints + 1
            Geoplaner.append(p)  # list for megacells

    # Take into account only megacells which have all of their subcells inside the poly
    i = 0
    messy_x = np.zeros((numberofpoints, 2))
    for p in Geoplaner:
        messy_x[i][0] = p.x
        messy_x[i][1] = p.y
        i = i + 1

    # When in flight direction mode
    center = centroid(messy_x)

    if flight_direction != 0:
        # Rotate Poly
        rotatepoly = []
        for i in range(len(messy_x)):
            rotatepoly.append(rotate_point((messy_x[i][0], messy_x[i][1]), flight_direction, (center[0], center[1])))

        for i in range(len(rotatepoly)):
            messy_x[i][0] = rotatepoly[i][0]
            messy_x[i][1] = rotatepoly[i][1]

    megacellsinpolygon = []
    messy_numberofwaypoints = 0
    numberofmegacells = 0
    messy_subcells = np.zeros(((numberofpoints * 4), 2))
    distance = geopy.distance.great_circle(
        meters=((math.sqrt(2) * stepsize) / 4))  # Distance between megacell and subcell
    for i in range(len(messy_x)):
        countsubcells = 0
        start = gp.Point(messy_x[i][0], messy_x[i][1])
        for j in range(4):
            if j == 3:
                bearing = 45 + flight_direction
            elif j == 0:
                bearing = 135 + flight_direction
            elif j == 1:
                bearing = 225 + flight_direction
            elif j == 2:
                bearing = 315 + flight_direction
            result = distance.destination(start, bearing)  # no radians here

            messy_subcells[messy_numberofwaypoints][0] = result.latitude
            messy_subcells[messy_numberofwaypoints][1] = result.longitude
            messy_numberofwaypoints += 1

            point = shapely.geometry.Point(result.latitude, result.longitude)
            if polygon.contains(point):
                countsubcells += 1
        if countsubcells == 4:
            megacellsinpolygon.append(i)
            numberofmegacells += 1

    x = np.zeros((numberofmegacells, 2))
    count = 0
    for i in megacellsinpolygon:
        x[count][0] = messy_x[i][0]
        x[count][1] = messy_x[i][1]
        count = count + 1

    print("The number of mega cells are %d" % (numberofmegacells - len(Obstacle)))

    # Handle errors during the path planning procedure - Return them to GUI in order to be properly displayed
    if numberofmegacells - len(Obstacle) <= 3:
        data = {"Cannot find path because the area is very small - Minimize scanning_distance and try again"}
        # Sending POST data to GUI with requests
        headers = {'Content-Type': 'application/json', }
        for url in ['http://127.0.0.1:8081/calculation_path_error?error_code=3']:
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

    print("The number of mega cells that contain obstacles are %d\n" % len(Obstacle))

    """
    #Kruskal algorithm & Adj Matrix implementation inside the area of interest -----------------------------------------
    """

    g = Graph(numberofmegacells)

    # For the Adjacency Matrix of megacells
    for i in range(len(x)):
        for j in range(len(x)):
            if i != j:
                if round(great_circle((x[i][0], x[i][1]), (x[j][0], x[j][1])).meters) == stepsize or round(
                        great_circle((x[i][0], x[i][1]), (x[j][0], x[j][1])).meters) == stepsize + 1 or round(
                    great_circle((x[i][0], x[i][1]), (x[j][0], x[j][1])).meters) == stepsize - 1 or round(
                    great_circle((x[i][0], x[i][1]), (x[j][0], x[j][1])).meters) == stepsize - 2 or round(
                    great_circle((x[i][0], x[i][1]), (x[j][0], x[j][1])).meters) == stepsize + 2 or round(
                    great_circle((x[i][0], x[i][1]), (x[j][0], x[j][1])).meters) == stepsize + 3 or round(
                    great_circle((x[i][0], x[i][1]), (x[j][0], x[j][1])).meters) == stepsize - 3:
                    g.addEdgeinadjmatrix(i, j)
                else:
                    g.removeEdgeinadjmatrix(i, j)

    # g.toString(); # for printing the Adjacency matrix

    # Aplly Kruskal algorithm in matrix x which contains the megacells of the area
    for i in range(len(x)):
        for j in range(len(x)):
            if g.containsEdge(i, j):
                g.addEdge(i, j, 0)
            else:
                g.addEdge(i, j, 1)

    MSTresult = g.KruskalMST()  # The results of Kruskal's algorithm

    # Now calculate the sub-cells for each of the megacells

    # FOUND THE REAL SUBCELLS INSIDE THE POLY
    subcells = np.zeros((len(x) * 4, 2))
    # distance = geopy.distance.great_circle(meters=(math.sqrt(2) * stepsize / 4) )
    numberofwaypoints = 0
    for i in range(len(x)):
        start = gp.Point(x[i][0], x[i][1])  # lat first , lon second
        for j in range(4):
            if j == 3:
                bearing = 45 + flight_direction
            elif j == 0:
                bearing = 135 + flight_direction
            elif j == 1:
                bearing = 225 + flight_direction
            elif j == 2:
                bearing = 315 + flight_direction
            result = distance.destination(start, bearing)  # no radians here
            subcells[numberofwaypoints][0] = result.latitude
            subcells[numberofwaypoints][1] = result.longitude
            numberofwaypoints = numberofwaypoints + 1

    # Adjacency matrix for the MST tree
    AdjMSTtree = np.zeros((numberofpoints, numberofpoints))
    for u, v, weight in MSTresult:
        if weight == 0:
            AdjMSTtree[u][v] = 1
            AdjMSTtree[v][u] = 1

    # ---------------------------------------------
    # Circumnavigation clockwise
    # ---------------------------------------------
    # Geoplaner.com
    root = ET.Element("gpx", version="1.1", creator="http://www.geoplaner.com",
                      xmlns="http://www.topografix.com/GPX/1/1",
                      schemaLocation="http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd")
    rte = ET.SubElement(root, "rte")

    ET.SubElement(rte, "name").text = "Route1"

    """
    # ------- **STAGE 1** -------------------------------------------------------------------------------------------------------
    """

    # Call Pattern.py & Intersection.py - Try to solve Hamiltonian path problem in linear time
    g2 = Pattern(numberofwaypoints, MSTresult, x, subcells, flight_direction)
    pattern = g2.patternforHamiltonianpath()
    for u, v, weight in MSTresult:
        p1 = Pointpattern(x[u][1], x[u][0])
        q1 = Pointpattern(x[v][1], x[v][0])
        for i in range(len(subcells)):
            for j in range(len(subcells)):
                if pattern[i][j] == 1:
                    p2 = Pointpattern(subcells[i][1], subcells[i][0])
                    q2 = Pointpattern(subcells[j][1], subcells[j][0])
                    if doIntersect(p1, q1, p2, q2):
                        pattern[i][j] = 0

    g1 = HAM(numberofwaypoints)
    g1.graph = pattern
    path = g1.hamCycle()
    # print("HamCycle path", path)
    waypointsformission = [0]
    pathpoints = np.zeros(((len(path)), 2))
    bearingofpathpoints = np.zeros(((len(path)), 3))

    # --------------
    # choose waypoints in turns
    # --------------
    # Store the points in subcells into an array which follows the path
    j = 0
    for i in path:
        pathpoints[j][0] = subcells[i][0]
        pathpoints[j][1] = subcells[i][1]
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

        if abs(bearingofpathpoints[i][2] - bearingofpathpoints[i + 1][
            2]) > 10:
            waypointsformission.append(int(bearingofpathpoints[i + 1][0]))

    print("Waypoints for path %s" % waypointsformission)

    Distance_for_path = 0
    for i in range(len(waypointsformission) - 1):
        Distance_for_path += great_circle(
            (pathpoints[waypointsformission[i]][1], pathpoints[waypointsformission[i]][0]),
            (pathpoints[waypointsformission[i + 1]][1], pathpoints[waypointsformission[i + 1]][0])).meters

    if Calculate_Travel(dist=Distance_for_path, time=time, speed=speed).cal_time() > 20:
        data = {"Cannot execute mission due to time limitation (20 minutes)"}
        # Sending POST data to GUI with requests
        headers = {'Content-Type': 'application/json', }
        for url in ['http://127.0.0.1:8081/calculation_path_error?error_code=4']:
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

    for i in waypointsformission:
        Distance_path = great_circle((), ())
        elem = ET.SubElement(rte, "rtept", lat=str(pathpoints[i][1]), lon=str(pathpoints[i][0]))
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

    # Average calculation time
    time_expected = (time.process_time() - time_start)
    data["DataObject"]["Expected_time"] = time_expected

    with open('path.geojson', 'w') as outfile:
        json.dump(data, outfile)

    print("Mission saved at ~PROJECT_PATH/path.geojson\n")


elif STAGE == 2:
    """
        # ------- **STAGE 2** ------------------------------------------------------------------------------------------
    """

    pointsfor2visit = []  # Points of interest that we have to re-visit

    with open(sys.argv[1]) as json_data:
        try:
            hot_spots = json.load(json_data)
        except ValueError:
            hot_spots = None

    with open('hotspots_data.geojson', 'w') as outfile:
        json.dump(hot_spots, outfile)

    radius = 5  # Radius ( in meters)
    speed = int(sys.argv[6])

    with open('hotspots_data.geojson') as json_file:
        data = json.load(json_file)
    for p in data['features']:
        for i in range(len(p['geometry']['coordinates'][0])):
            pointsfor2visit.append([p['geometry']['coordinates'][0][i][0], p['geometry']['coordinates'][0][i][1]])

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

    data, route = Site_Specific_mission(distanceTSP).main()

    # ------------------------------------------------------------------------------------------------------------------
    # 										Circular Flight for each point of interest
    # ------------------------------------------------------------------------------------------------------------------

    # Hot-Spot Mission for each point of interested
    hotpoint = []
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

    # Return to Home Location
    hotpoint.append((pointsfor2visit[route[0]][0], pointsfor2visit[route[0]][1]))

    # Calculate distance between Hot-Points (final route)
    distanceTSPhot = 0
    overalldistance = 0
    for i in range(len(hotpoint) - 1):
        distanceTSPhot = great_circle((hotpoint[i][0], hotpoint[i][1]),
                                      (hotpoint[i + 1][0], hotpoint[i + 1][1])).meters

        overalldistance = overalldistance + distanceTSPhot

    # Geoplaner.com
    root = ET.Element("gpx", version="1.1", creator="http://www.geoplaner.com",
                      xmlns="http://www.topografix.com/GPX/1/1",
                      schemaLocation="http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd")
    rte = ET.SubElement(root, "rte")

    ET.SubElement(rte, "name").text = "Route1"

    with open('Stage2LON', 'w') as filehandle:  # file that saves HOTPOINT's Longitude for simulator

        # Write Hotpoint for Geoplaner
        numberofWP = 0
        for Waypoint in hotpoint:
            elem = ET.SubElement(rte, "rtept", lat=str(Waypoint[0]), lon=str(Waypoint[1]))
            ET.SubElement(elem, "ele").text = "WP" + str(numberofWP)
            ET.SubElement(elem, "name").text = "WP" + str(numberofWP)
            numberofWP += 1
        tree = ET.ElementTree(root)
        tree.write("pathFollowinginGeo.gpx", xml_declaration="1.0", encoding="UTF-8")

    # Path in .geojson for IKH in STAGE 1
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
    time_expected = (time.process_time() - time_start)
    data["DataObject"]["Expected_time"] = time_expected

    with open('Hotpoint_path.geojson', 'w') as outfile:
        json.dump(data, outfile)

    print("Mission saved at ~PROJECT_PATH/Hotpoint_path.geojson\n")

# Sending POST data to GUI
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
        print('Other error occurred: {Port is closed}')
    else:
        print('Communication - Success!')

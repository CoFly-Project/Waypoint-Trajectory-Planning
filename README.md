# Waypoint-Trajectory-Planning
A waypoint-based mission planner for coverage and inspection tasks. 

This package deals with the path planning problem of a mobile robot, in order to calculate coverage patterns supporting No-Fly zones/Obstacles and optimal routes in any arbitrary polygon shape. 

Example of Coverage:

![CoverageImage](https://user-images.githubusercontent.com/48250484/114038719-c43a5c80-988a-11eb-8684-40f71522e4c5.png)

Example of Inspection:

<img src="https://user-images.githubusercontent.com/48250484/114039072-19766e00-988b-11eb-930c-a8ef1bc1ad9d.png" width="250"> <img src="https://user-images.githubusercontent.com/48250484/114040299-28a9eb80-988c-11eb-898d-d738e48a42ec.png" width="337">

## Dependencies

Install all the neccecary dependencies using pip3 install <package name<package name>>
  
## General use
> /~PROJECT_PATH/GeoCordinates polygon_area obstacle_area flight_direction scanning_distance mission_type speed

## Details
- Polygon & Obstacle areas must follow the original type of .geojson
- Flight direction is in degrees
- Scanning distance is in meters
- Mission type = 1 is for coverage task
- Mission type = 2 is for inspection task
- Speed is in m/s

> Waypoints for coverage and inspection tasks are saved at path.geojson and at Hotpoint_path.geojson, respectively.

## Run

1. Clone this repo.
2. Open terminal on ~PROJECT_PATH
3. Run 
    -  For coverage e.g. GeoCordinates map_data.geojson disabled_paths.geojson 0 40 1 3 
    -  For inspection e.g. GeoCordinates hotspot_data.geojson disabled_paths.geojson 0 5 2 3 

> Note: Visualization of the results can be shown through CoFly GUI at http://localhost:8081/calculated_path. 

## References
cite as:
> Not published yet

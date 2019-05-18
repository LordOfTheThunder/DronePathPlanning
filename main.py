from Waypoint import file, loadData
from PathPlanning import pathPlanning
import SimulationGenerator
import FlowHelper
from MainConfig import logger, global_config, Flows, PointFormats
from PathPlanning.pathPlanning import TravelingSalesmanTypes

from PathPlanning.pathPlanningHelpers import metersToLongLat

def pathPlanningCall(start_point, sensor_coords_with_radius, obstacle_bboxes):
    # If we work in meters, our points are all relative to 0,0 so start_point should be updated
    if global_config["Point Format"] == PointFormats.Meters:
        start_point = [0, 0]

    # Calculate path with traveling salesman
    # Regular traveling salesman
    # path = list(pathPlanning.travelingSalesman(start_point, sensor_coords))
    # Adapted traveling salesman
    # path = list(pathPlanning.advancedTravelingSalesman(start_point, sensor_coords_with_radius))
    # Obstacle traveling salesman
    # path, dist = pathPlanning.obstacleTravelingSalesman(start_point, sensor_coords_with_radius, obstacle_bboxes)
    # Obstacle traveling salesman with dynamic algorithm
    # path, dist = pathPlanning.obstacleTravelingSalesman(start_point, sensor_coords_with_radius, obstacle_bboxes, TravelingSalesmanTypes.HeuristicDynamic)
    # path, dist = pathPlanning.obstacleTravelingSalesman(start_point, sensor_coords_with_radius, obstacle_bboxes, TravelingSalesmanTypes.BruteForceDynamic)
    # Dynamic optimized traveling salesman : take straight line if no obstacles. Otherwise calculate route using grid and dijkstra
    # path, dist = pathPlanning.obstacleTravelingSalesman(start_point, sensor_coords_with_radius, obstacle_bboxes, TravelingSalesmanTypes.HeuristicOptimizedDynamic)
    path, dist = pathPlanning.obstacleTravelingSalesman(start_point, sensor_coords_with_radius, obstacle_bboxes,
                                                        TravelingSalesmanTypes.BruteForceOptimizedDynamic)
    logger.info("Total dist is: " + str(dist))
    return path, dist

def SimulationFlow():
    # Parse sensor points
    sensor_coords, sensor_coords_with_radius = FlowHelper.getSensorPointsFromCsv()
    # Parse start point
    start_point, start_point_map_coords = FlowHelper.getStartPointFromCsv()
    # Parse Obstacle bboxes
    obstacle_bboxes = FlowHelper.getObstacleBboxesFromCsv()
    # Generate HTML file using config
    SimulationGenerator.generateHTMLFile()
    # Generate Config JS file
    SimulationGenerator.generateConfigFile()
    # Generate files with start point, sensor points and path
    SimulationGenerator.generateStartPointFile(start_point)
    # Generate sensor points file
    SimulationGenerator.generateSensorPointsFile(sensor_coords_with_radius)
    # Generate sensor radii intersection file
    SimulationGenerator.generateIntersectionFile(sensor_coords_with_radius)
    # Generate Obstacle file
    SimulationGenerator.generateObstacleFile(obstacle_bboxes)
    # Generate grid file
    SimulationGenerator.generateGridFile(start_point, sensor_coords_with_radius)
    # Calculate path with traveling salesman
    path, dist = pathPlanningCall(start_point, sensor_coords_with_radius, obstacle_bboxes)
    # Generate Path File
    SimulationGenerator.generatePathFile(path)

def WaypointFlow():
    # Parse sensor points
    sensor_coords, sensor_coords_with_radius = FlowHelper.getSensorPointsFromCsv()
    # Parse start point
    start_point, start_point_map_coords = FlowHelper.getStartPointFromCsv()
    # Parse Obstacle bboxes
    obstacle_bboxes = FlowHelper.getObstacleBboxesFromCsv()
    # Parse config file
    loadData.readWaypointConfig()
    # Calculate path with traveling salesman
    path, dist = pathPlanningCall(start_point, sensor_coords_with_radius, obstacle_bboxes)
    # Create waypoint file
    file.createWaypointFile(start_point_map_coords, path)

def CombinedFlows():
    # Parse sensor points
    sensor_coords, sensor_coords_with_radius = FlowHelper.getSensorPointsFromCsv()
    # Parse start point
    start_point, start_point_map_coords = FlowHelper.getStartPointFromCsv()
    # Parse Obstacle bboxes
    obstacle_bboxes = FlowHelper.getObstacleBboxesFromCsv()
    # Parse config file
    loadData.readWaypointConfig()
    # Generate HTML file using config
    SimulationGenerator.generateHTMLFile()
    # Generate Config JS file
    SimulationGenerator.generateConfigFile()
    # Generate files with start point, sensor points and path
    SimulationGenerator.generateStartPointFile(start_point)
    # Generate sensor points file
    SimulationGenerator.generateSensorPointsFile(sensor_coords_with_radius)
    # Generate sensor radii intersection file
    SimulationGenerator.generateIntersectionFile(sensor_coords_with_radius)
    # Generate Obstacle file
    SimulationGenerator.generateObstacleFile(obstacle_bboxes)
    # Generate grid file
    SimulationGenerator.generateGridFile(start_point, sensor_coords_with_radius)
    # Calculate path with traveling salesman
    path, dist = pathPlanningCall(start_point, sensor_coords_with_radius, obstacle_bboxes)
    # Create waypoint file
    file.createWaypointFile(start_point_map_coords, path)
    # Generate Path File
    SimulationGenerator.generatePathFile(path)

if __name__ == "__main__":
    # Different flows we can execute
    if global_config["Current Flow"] == Flows.Waypoint:
        WaypointFlow()
    if global_config["Current Flow"] == Flows.Simulation:
        SimulationFlow()
    if global_config["Current Flow"] == Flows.Both:
        CombinedFlows()

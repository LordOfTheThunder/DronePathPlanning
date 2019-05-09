from Waypoint import file
from PathPlanning import pathPlanning
import SimulationGenerator
import FlowHelper
from MainConfig import logger
from PathPlanning.pathPlanning import TravelingSalesmanTypes

def SimulationFlow():
    # Parse sensor points
    sensor_coords, sensor_coords_with_radius = FlowHelper.getSensorPointsFromCsv()
    # Parse start point
    start_point = FlowHelper.getStartPointFromCsv()
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

    # Regular traveling salesman
    # path = list(pathPlanning.travelingSalesman(start_point, sensor_coords))
    # Adapted traveling salesman
    # path = list(pathPlanning.advancedTravelingSalesman(start_point, sensor_coords_with_radius))
    # Obstacle traveling salesman
    # path, dist = pathPlanning.obstacleTravelingSalesman(start_point, sensor_coords_with_radius, obstacle_bboxes)
    # Obstacle traveling salesman with dynamic algorithm
    path, dist = pathPlanning.obstacleTravelingSalesman(start_point, sensor_coords_with_radius, obstacle_bboxes, TravelingSalesmanTypes.HeuristicDynamic)
    logger.info("Total dist is: " + str(dist))
    # Generate Path File
    SimulationGenerator.generatePathFile(path)

def WaypointFlow():
    # Parse sensor points
    sensor_coords, sensor_coords_with_radius = FlowHelper.getSensorPointsFromCsv()
    # Parse start point
    start_point = FlowHelper.getStartPointFromCsv()
    # Parse Obstacle bboxes
    obstacle_bboxes = FlowHelper.getObstacleBboxesFromCsv()

    # Calculate path with traveling salesman
    # Regular traveling salesman
    # path = list(pathPlanning.travelingSalesman(start_point, sensor_coords))
    # Adapted traveling salesman
    # path = list(pathPlanning.advancedTravelingSalesman(start_point, sensor_coords_with_radius))
    # Obstacle traveling salesman
    # path, dist = pathPlanning.obstacleTravelingSalesman(start_point, sensor_coords_with_radius, obstacle_bboxes)
    # Obstacle traveling salesman with dynamic algorithm
    path, dist = pathPlanning.obstacleTravelingSalesman(start_point, sensor_coords_with_radius, obstacle_bboxes, TravelingSalesmanTypes.HeuristicDynamic)
    # Create waypoint file
    file.createWaypointFile(start_point, path)

if __name__ == "__main__":
    # Different flows we can execute
    # WaypointFlow()
    SimulationFlow()
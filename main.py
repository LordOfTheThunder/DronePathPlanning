from Waypoint import file
from PathPlanning import pathPlanning
import SimulationGenerator
import FlowHelper

def SimulationFlow():
    # Parse sensor points
    sensor_coords, sensor_coords_with_radius = FlowHelper.getSensorPointsFromCsv()
    # Parse start point
    start_point = FlowHelper.getStartPointFromCsv()
    # Generate HTML file using config
    SimulationGenerator.generateHTMLFile()
    # Generate files with start point, sensor points and path
    SimulationGenerator.generateStartPointFile(start_point)
    # Generate sensor points file
    SimulationGenerator.generateSensorPointsFile(sensor_coords_with_radius)
    # Calculate path with traveling salesman
    # For now use the regular non adaptive traveling salesman - we will later upgrade it to adapted traveling salesman with radius
    path = list(pathPlanning.travelingSalesman(start_point, sensor_coords))
    # Generate Path File
    SimulationGenerator.generatePathFile(path)

def WaypointFlow():
    # Parse sensor points
    sensor_coords, sensor_coords_with_radius = FlowHelper.getSensorPointsFromCsv()
    # Parse start point
    start_point = FlowHelper.getStartPointFromCsv()
    # Calculate path with traveling salesman
    # For now use the regular non adaptive traveling salesman - we will later upgrade it to adapted traveling salesman with radius
    path = list(pathPlanning.travelingSalesman(start_point, sensor_coords))
    # Create waypoint file
    file.createWaypointFile(start_point, path)

if __name__ == "__main__":
    # Different flows we can execute
    SimulationFlow()
    # WaypointFlow()
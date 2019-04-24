from Waypoint import file
from PathPlanning import pathPlanning
from MainConfig import global_config
import SimulationGenerator
import FlowHelper

def Flow():
    # Parse sensor points
    sensor_coords, sensor_coords_with_radius = FlowHelper.getSensorPointsFromCsv()
    # Generate HTML file using config
    SimulationGenerator.generateHTMLFile()
    # Generate files with start point, sensor points and path
    SimulationGenerator.generateStartPointFile()
    # Generate sensor points file
    SimulationGenerator.generateSensorPointsFile(sensor_coords_with_radius)
    # Calculate path with traveling salesman
    # For now use the regular non adaptive traveling salesman - we will later upgrade it to adapted traveling salesman with radius
    path = list(pathPlanning.travelingSalesman(global_config["Start Point"], sensor_coords))
    # Generate Path File
    SimulationGenerator.generatePathFile(path)

if __name__ == "__main__":
    # file.createWaypointFile([[10, 10]])
    # print(list(pathPlanning.travelingSalesman([3, 3], [[5, 4], [4, 4]])))
    # print(list(pathPlanning.travelingSalesman([3, 3], [[11, 3], [4, 4], [4.5, 7], [3, 5], [8, 7]])))
    Flow()
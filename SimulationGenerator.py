# Generate HTML file
# Generate js files with configs after getting input from path planning
from MainConfig import simulation_config, logger, path_planning_algorithm_config, js_config
from Geo import GeoHelpers
from shapely.geometry import Point, mapping

def generateHTMLFile():
    html_path = simulation_config["Relative Path"] + simulation_config["HTML File"]
    with open(html_path, 'w') as fh:
        fh.write("<!DOCTYPE html>\n")
        fh.write("<html>\n")
        fh.write("<body>\n\n")
        fh.write("<canvas id=\"myCanvas\" width=1000 height=500 style=\"border:1px solid #d3d3d3;\">\n")
        fh.write("Your browser does not support the HTML5 canvas tag.</canvas>\n\n")
        fh.write("<button onclick=\"gridOn()\" id=\"gridButton\">Grid On</button>\n\n")
        fh.write("<script src=\"" + simulation_config["Config File"] + "\"></script>\n")
        fh.write("<script src=\"" + simulation_config["Start Point File"] +"\"></script>\n")
        fh.write("<script src=\"" + simulation_config["Sensor File"] + "\"></script>\n")
        fh.write("<script src=\"" + simulation_config["Path File"] + "\"></script>\n")
        fh.write("<script src=\"" + simulation_config["Obstacle Path File"] + "\"></script>\n")
        fh.write("<script src=\"" + simulation_config["Intersection File"] + "\"></script>\n")
        fh.write("<script src=\"" + simulation_config["Grid File"] + "\"></script>\n")
        fh.write("<script src=\"" + simulation_config["JS Code"] + "\"></script>\n\n")
        fh.write("</body>\n")
        fh.write("</html>\n")
    fh.close()
    logger.info('Generated HTML File at ' + html_path)

def generateStartPointFile(start_point):
    start_point_path = simulation_config["Relative Path"] + simulation_config["Start Point File"]
    with open(start_point_path, 'w') as fh:
        fh.write("var start_point = " + "[" + str(start_point[0]) + "," + str(start_point[1]) + ",0" + "]")
    fh.close()
    logger.info('Generated Start Point File at ' + start_point_path)

def generateSensorPointsFile(sensor_positions):
    sensors_point_path = simulation_config["Relative Path"] + simulation_config["Sensor File"]
    with open(sensors_point_path, 'w') as fh:
        fh.write("var sensor_points = " + "[")
        cnt = len(sensor_positions)
        last_space = ", "
        for point in sensor_positions:
            if cnt == 1:
                last_space = ""
            fh.write("[" + str(point[0]) + "," + str(point[1]) + "," + str(point[2]) + "]" + last_space)
            cnt -= 1
        fh.write("]")
    fh.close()
    logger.info('Generated Sensor Point File at ' + sensors_point_path)

def generatePathFile(path):
    path_point_path = simulation_config["Relative Path"] + simulation_config["Path File"]
    with open(path_point_path, 'w') as fh:
        first_line = "var path_points = " + "["
        second_line = "var path_actions = " + "["
        cnt = len(path)
        last_space = ", "
        for action, point in path:
            if cnt == 1:
                last_space = ""
            first_line += "[" + str(point[0]) + "," + str(point[1]) + "]" + last_space
            second_line += "[\"" + action + "\"]" + last_space
            cnt -= 1
        fh.write(first_line + "]\n")
        fh.write(second_line + "]\n")
    fh.close()
    logger.info('Generated Path File at ' + path_point_path)

# Used for canvas drawing
def generateIntersectionFile(point_radius_list):
    pols = list(GeoHelpers.getCircleObjectsFromList(point_radius_list))
    # Calculate all intersections
    intersections = list(GeoHelpers.getIntersectionsFromCircles(pols))

    # write all intersections to js file as array of intersection strings
    intersection_point_path = simulation_config["Relative Path"] + simulation_config["Intersection File"]
    with open(intersection_point_path, 'w') as fh:
        fh.write("var intersection_point_path = []\n")
        for intersection in intersections:
            fh.write("intersection_point_path.push([")
            coords = mapping(intersection.boundary)['coordinates']
            cnt = len(coords)
            last_space = ", "
            for coord in coords:
                if cnt == 1:
                    last_space = ""
                point_string = ','.join(list(map(str, coord)))
                fh.write("[" + point_string + "]" + last_space)
                cnt -= 1
            fh.write("])\n")
    fh.close()

def generateObstacleFile(obstacle_bboxes):
    obstacle_bboxes_path = simulation_config["Relative Path"] + simulation_config["Obstacle Path File"]
    with open(obstacle_bboxes_path, 'w') as fh:
        fh.write("var obstacle_bboxes = " + "[")
        cnt = len(obstacle_bboxes)
        last_space = ", "
        for bbox in obstacle_bboxes:
            if cnt == 1:
                last_space = ""
            fh.write("[" + "[" + str(bbox[0]) + "," + str(bbox[1]) + "]" + "," + "[" + str(bbox[2]) + "," + str(bbox[3]) + "]" + "]" + last_space)
            cnt -= 1
        fh.write("]")
    fh.close()

def generateGridFile(start_point, point_radius_list):
    nodes = list(GeoHelpers.getIntersectionPointsForShapes(point_radius_list)) + [start_point]
    lx, ly, ux, uy = GeoHelpers.calcLimitsFromPoints(nodes, path_planning_algorithm_config["Grid Coeff"])
    grid_size = path_planning_algorithm_config["Grid Size"]
    x_step = (ux - lx) / grid_size
    y_step = (uy - ly) / grid_size
    grid_file = simulation_config["Relative Path"] + simulation_config["Grid File"]
    with open(grid_file, 'w') as fh:
        fh.write("var grid_lines = " + "[")
        cnt = 0
        last_space = ", "
        for i in range(grid_size + 1):
            if cnt == grid_size:
                last_space = ""
            # Horizontal Line
            horizontal_line_lx = lx
            horizontal_line_ly = ly + y_step * cnt
            horizontal_line_ux = ux
            horizontal_line_uy = horizontal_line_ly
            # Vertical Line
            vertical_line_lx = lx + x_step * cnt
            vertical_line_ly = ly
            vertical_line_ux = vertical_line_lx
            vertical_line_uy = uy
            fh.write("[" + "[" + str(horizontal_line_ly) + "," + str(horizontal_line_lx) + "]" + ", ["
                     + str(horizontal_line_uy) + "," + str(horizontal_line_ux) + "]" + "]" + ", ")
            fh.write("[" + "[" + str(vertical_line_ly) + "," + str(vertical_line_lx) + "]" + ", ["
                     + str(vertical_line_uy) + "," + str(vertical_line_ux) + "]" + "]" + last_space)
            cnt += 1
        fh.write("]")
    fh.close()

def generateConfigFile():
    config_file = simulation_config["Relative Path"] + simulation_config["Config File"]
    with open(config_file, 'w') as fh:
        for key in js_config:
            fh.write("var " + key + " = " + str(js_config[key]) + ";\n")
    fh.close()
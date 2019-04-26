# Generate HTML file
# Generate js files with configs after getting input from path planning
from MainConfig import simulation_config, logger
from shapely.geometry import Point, mapping

def generateHTMLFile():
    html_path = simulation_config["Relative Path"] + simulation_config["HTML File"]
    with open(html_path, 'w') as fh:
        fh.write("<!DOCTYPE html>\n")
        fh.write("<html>\n")
        fh.write("<body>\n\n")
        fh.write("<canvas id=\"myCanvas\" width=1000 height=500 style=\"border:1px solid #d3d3d3;\">\n")
        fh.write("Your browser does not support the HTML5 canvas tag.</canvas>\n\n")
        fh.write("<script src=\"" + simulation_config["Start Point File"] +"\"></script>\n")
        fh.write("<script src=\"" + simulation_config["Sensor File"] + "\"></script>\n")
        fh.write("<script src=\"" + simulation_config["Path File"] + "\"></script>\n")
        fh.write("<script src=\"" + simulation_config["Intersection File"] + "\"></script>\n")
        fh.write("<script src=\"" + simulation_config["JS Code"] + "\"></script>\n\n")
        fh.write("</body>\n")
        fh.write("</html>\n")
    fh.close()
    logger.info('Generated HTML File at ' + html_path)

def generateStartPointFile(start_point):
    start_point_path = simulation_config["Relative Path"] + simulation_config["Start Point File"]
    with open(start_point_path, 'w') as fh:
        fh.write("start_point = " + "[" + str(start_point[0]) + "," + str(start_point[1]) + ",0" + "]")
    fh.close()
    logger.info('Generated Start Point File at ' + start_point_path)

def generateSensorPointsFile(sensor_positions):
    sensors_point_path = simulation_config["Relative Path"] + simulation_config["Sensor File"]
    with open(sensors_point_path, 'w') as fh:
        fh.write("sensor_points = " + "[")
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
        fh.write("path_points = " + "[")
        cnt = len(path)
        last_space = ", "
        for point in path:
            if cnt == 1:
                last_space = ""
            fh.write("[" + str(point[0]) + "," + str(point[1]) + "]" + last_space)
            cnt -= 1
        fh.write("]")
    fh.close()
    logger.info('Generated Path File at ' + path_point_path)

# Used for canvas drawing
def generateIntersectionFile(point_radius_list):
    pols = []
    # Get all circle objects
    for x_y_radius in point_radius_list:
        p = Point(x_y_radius[0], x_y_radius[1])
        circle = p.buffer(x_y_radius[2])
        pols.append(circle)

    # Calculate all intersections
    intersections = []
    for i in range(len(pols)):
        for j in range(i + 1, len(pols)):
            pol_1 = pols[i]
            pol_2 = pols[j]
            if pol_1 is pol_2 or not pol_1.intersects(pol_2):
                continue
            intersections.append(pol_1.intersection(pol_2))

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
        pass
    fh.close()

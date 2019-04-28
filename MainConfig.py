import logging

"""
Different plans we might decide for our drone
Follow Points : follow points in order and then return to base
"""
path_plans_config = {
    "Follow Points" : 0
}

waypoint_config = {
    "Default Alt" : 100,
    "Columns" : "<INDEX> <CURRENT WP> <COORD FRAME> <COMMAND> <PARAM1> <PARAM2> <PARAM3> <PARAM4> <PARAM5/X/LONGITUDE> <PARAM6/Y/LATITUDE> <PARAM7/Z/ALTITUDE> <AUTOCONTINUE>",
    "Relative Path" : "./output/",
    "File Name" : "path_planning.waypoints",
    "First Row" : "QGC WPL",
    "Version" : "110",
    "Commands" : {
        "WAYPOINT" : 16,
        "RETURN_TO_LAUNCH" : 20
    },
    "Default Path Plan" : path_plans_config["Follow Points"],
    "Default Coord Frame" : 3,
}

global_config = {
    "Relative Path" : "./input/",
    "Start Point CSV" : "start_point.csv",
    "Sensor Point CSV" : "sensor_positions.csv",
    "Obstacles Bbox CSV" : "obstacle_bboxes.csv",
    "Path Name" : "path1",
}

simulation_config = {
    "Relative Path" : "./PathPlotter/",
    "HTML File" : "path_plotter.html",
    "JS Code" : "path_plotter.js",
    "Start Point File" : global_config["Path Name"] + ".start_point.js",
    "Sensor File" : global_config["Path Name"] + ".sensors.js",
    "Path File" : global_config["Path Name"] + ".path.js",
    "Obstacle Path File" : global_config["Path Name"] + ".obstacle_bbox_path.js",
    # These files are input files generated using only python code without CSV dependency. So it is not path specific
    # Therefore we don't add path name to the name of the file
    "Intersection File" : "intersection_path.js",
    "Grid File" : "grid_file.js",
    "Config File" : "config.js"
}

path_planning_algorithm_config = {
    "Grid Size" : 100,
    "Grid Coeff" : 0.2
}

js_config = {
    "animation_time" : 5,
    "draw_point_width" : 4,
    "text_size" : 12,
    "path_ordering_text_size" : 16,
    "grid_flag" : "true",
    "show_path_ordering" : "false"
}

# Logger for the debug logs
logger = logging.getLogger()
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
logger.setLevel(logging.DEBUG)

fh = logging.FileHandler('path_planning_log.txt')
fh.setLevel(logging.DEBUG)
fh.setFormatter(formatter)
logger.addHandler(fh)

ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
ch.setFormatter(formatter)
logger.addHandler(ch)
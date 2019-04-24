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

simulation_config = {
    "Relative Path" : "./PathPlotter/",
    "HTML File" : "path_plotter.html",
    "JS Code" : "path_plotter.js",
    "Start Point File" : "path1.start_point.js",
    "Sensor File" : "path1.sensors.js",
    "Path File" : "path1.path.js",
}

global_config = {
    "Start Point" : [31.323287, 34.299660],
    "Sensor Point CSV" : "sensor_positions.csv",
}
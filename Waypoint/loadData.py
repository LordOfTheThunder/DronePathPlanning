from MainConfig import waypoint_config

# Read waypoint.config and load everything from it to waypoint_config
def readWaypointConfig():
    config_path = waypoint_config["Config File"]
    with open(config_path, 'r') as fh:
        for line in fh:
            att, value = line.split(':')
            waypoint_config[att] = value
    fh.close()
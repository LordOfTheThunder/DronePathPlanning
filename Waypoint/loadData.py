from MainConfig import waypoint_config, global_config, PointFormats

# Read waypoint.config and load everything from it to waypoint_config
def readWaypointConfig():
    config_path = global_config["Config File"]
    with open(config_path, 'r') as fh:
        for line in fh:
            # ignore comments
            if line[0] == "#":
                continue
            config_type, line = line.split(',')
            att, value = line.split(':')
            # Special Check: if it's point format we need to append PointFormats enum
            if att == "Point Format":
                value = "PointFormats." + value
            exec(config_type.lower() + "_config[\"" + att.lstrip() + "\"] = " + value.rstrip())

    fh.close()
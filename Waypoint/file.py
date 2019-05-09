from MainConfig import waypoint_config, logger, path_plans_config
import pandas as pd
import csv

def createWaypointFile(start_point, point_list):
    column_names = waypoint_config["Columns"]
    columns = column_names.split(' ')
    first_row = waypoint_config["First Row"].split(' ')
    first_row.append(waypoint_config["Version"])

    file = waypoint_config["Relative Path"] + waypoint_config["File Name"]
    with open(file, 'w') as fh:
        writer = csv.writer(fh, delimiter='\t')
        writer.writerow([' '.join(first_row)])
    fh.close()

    def createWaypointRowMap(data_vec):
        data_map = {
            '<CURRENT WP>': [data_vec[0]],
            '<COORD FRAME>': [data_vec[1]],
            '<COMMAND>': [data_vec[2]],
            '<PARAM1>': [data_vec[3]],
            '<PARAM2>': [data_vec[4]],
            '<PARAM3>': [data_vec[5]],
            '<PARAM4>': [data_vec[6]],
            '<PARAM5/X/LONGITUDE>': [data_vec[7]],
            '<PARAM6/Y/LATITUDE>': [data_vec[8]],
            '<PARAM7/Z/ALTITUDE>': [data_vec[9]],
            '<AUTOCONTINUE>': [data_vec[10]],
        }
        return data_map

    def appendToWaypointRowMap(map, data_vec):
        map['<CURRENT WP>'].append(data_vec[0])
        map['<COORD FRAME>'].append(data_vec[1])
        map['<COMMAND>'].append(data_vec[2])
        map['<PARAM1>'].append(data_vec[3])
        map['<PARAM2>'].append(data_vec[4])
        map['<PARAM3>'].append(data_vec[5])
        map['<PARAM4>'].append(data_vec[6])
        map['<PARAM5/X/LONGITUDE>'].append(data_vec[7])
        map['<PARAM6/Y/LATITUDE>'].append(data_vec[8])
        map['<PARAM7/Z/ALTITUDE>'].append(data_vec[9])
        map['<AUTOCONTINUE>'].append(data_vec[10])
        return map

    # Add first point - point of HOME
    first_point = [1, 0, waypoint_config["Commands"]["WAYPOINT"], 0, 0, 0, 0,
                   start_point[0], start_point[1],
                   waypoint_config["Default Alt"], 1]
    data = createWaypointRowMap(first_point)

    # Build other points according to path plan
    if waypoint_config["Default Path Plan"] == path_plans_config["Follow Points"]:
        for action, point in point_list:
            action_cmd = waypoint_config["Commands"]["WAYPOINT"]
            first_arg = 0
            if action == "stop":
                action_cmd = waypoint_config["Commands"]["LOITER_TIME"]
                first_arg = waypoint_config["Default Loiter Time"]
            point_data = [0, waypoint_config["Default Coord Frame"], action_cmd,
                          first_arg, 0, 0, 0, point[0], point[1], waypoint_config["Default Alt"], 1]
            data = appendToWaypointRowMap(data, point_data)
        # Add last point - return to launch
        last_point = [0, waypoint_config["Default Coord Frame"],
                                           waypoint_config["Commands"]["RETURN_TO_LAUNCH"],
                                           0, 0, 0, 0, 0, 0, 0, 1]
        data = appendToWaypointRowMap(data, last_point)
    else:
        logger.error("No plan by name " + waypoint_config["Default Path Plan"] +" exists")
        exit(1)
    df = pd.DataFrame(data)
    # Write all to csv
    df.to_csv(file, sep='\t', header=False, mode='a')
    logger.info('Generated Waypoint File at ' + file)

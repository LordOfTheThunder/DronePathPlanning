from MainConfig import global_config
def getSensorPointsFromCsv():
    file = global_config["Relative Path"] + global_config["Sensor Point CSV"]
    sensor_coords = []
    sensor_coords_with_radius = []
    with open(file, 'r') as fh:
        for coord in fh:
            coord_list = [float(x) for x in coord.split(',')]
            sensor_coords_with_radius.append(coord_list)
            sensor_coords.append(coord_list[:len(coord_list) - 1])
    fh.close()

    return [sensor_coords, sensor_coords_with_radius]

def getStartPointFromCsv():
    file = global_config["Relative Path"] + global_config["Start Point CSV"]
    with open(file, 'r') as fh:
        point = [float(x) for x in fh.readline().split(',')]
    fh.close()
    return point

def getObstacleBboxesFromCsv():
    file = global_config["Relative Path"] + global_config["Obstacles Bbox CSV"]
    bbox_list = []
    with open(file, 'r') as fh:
        for bbox in fh:
            bbox_list.append([float(x) for x in bbox.split(',')])
    fh.close()
    return bbox_list

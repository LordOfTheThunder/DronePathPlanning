from MainConfig import global_config, PointFormats
def getSensorPointsFromCsv():
    file = global_config["Relative Path"] + global_config["Sensor Point CSV"]
    sensor_coords = []
    sensor_coords_with_radius = []

    with open(file, 'r') as fh:
        for coord in fh:
            coord_list = [float(x) for x in coord.split(',')]
            if global_config["Point Format"] == PointFormats.LongLat:
                # Need to convert meter radius format to long lat
                # Dirty estimate : 111,111m is one degree in latitude
                # The clean estimate will be to work with ellipsis instead of circles
                # By calculating vertical radius and horizontal radius based on https://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
                coord_list[-1] /= 111111
            sensor_coords_with_radius.append(coord_list)
            sensor_coords.append(coord_list[:len(coord_list) - 1])
    fh.close()

    return [sensor_coords, sensor_coords_with_radius]

def getStartPointFromCsv():
    file = global_config["Relative Path"] + global_config["Start Point CSV"]
    with open(file, 'r') as fh:
        point = [float(x) for x in fh.readline().split(',')]
    fh.close()
    if global_config["Point Format"] == PointFormats.Meters:
        return [0, 0], point
    return point, point

def getObstacleBboxesFromCsv():
    file = global_config["Relative Path"] + global_config["Obstacles Bbox CSV"]
    bbox_list = []
    with open(file, 'r') as fh:
        for bbox in fh:
            bbox_list.append([float(x) for x in bbox.split(',')])
    fh.close()
    return bbox_list

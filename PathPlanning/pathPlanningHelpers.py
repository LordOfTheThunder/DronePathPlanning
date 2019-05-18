import math
from geopy import Point
from geopy.distance import vincenty

def euclideanDist(first_point, second_point):
    return ((first_point[0] - second_point[0])**2 + (first_point[1] - second_point[1])**2)**0.5

def pointToString(point):
    return str(point[0]) + "," + str(point[1])

# Point in meters:
# point[0] is horizontal distance and point[1] is vertical distance
# orig_point is lat, long
def metersToLongLat(orig_point, point_in_meters):
    bearing = math.degrees(math.atan(point_in_meters[0] / point_in_meters[1]))
    dist_meters = euclideanDist([0, 0], point_in_meters)
    res = vincenty(meters=dist_meters).destination(Point(orig_point[0], orig_point[1]), bearing).format_decimal().split(',')
    return float(res[0]), float(res[1])

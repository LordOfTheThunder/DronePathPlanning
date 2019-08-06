from shapely.geometry import Point, LineString, mapping
import numpy as np
from operator import itemgetter
from MainConfig import geo_config, path_planning_algorithm_config

def euclideanDist(first_point, second_point):
    return ((first_point[0] - second_point[0])**2 + (first_point[1] - second_point[1])**2)**0.5

def getCircleObjectsFromList(point_radius_list):
    # Get all circle objects
    for x_y_radius in point_radius_list:
        p = Point(x_y_radius[0], x_y_radius[1])
        circle = p.buffer(x_y_radius[2])
        yield circle


def getIntersectionsFromCircles(pols):
    intersections = []
    for i in range(len(pols)):
        for j in range(i + 1, len(pols)):
            pol_1 = pols[i]
            pol_2 = pols[j]
            if pol_1 is pol_2 or not pol_1.intersects(pol_2):
                continue
            curr_int = pol_1.intersection(pol_2)
            # Check if we already have an intersection which takes this
            exists = False
            for s in range(len(intersections)):
                if curr_int.intersects(intersections[s]):
                    intersections[s] = curr_int.union(intersections[s])
                    exists = True
            if not exists:
                intersections.append(curr_int)

    return intersections

# Given a point and an intersection/sensor shape we would like to get a point inside the shape which is the closest
# to our original point
def getClosestPointFromPointToShape(point, shape):
    # Create line shape from point and center of shape
    center_of_shape = shape.representative_point()
    line = LineString([(point.x, point.y), (center_of_shape.x, center_of_shape.y)])

    intersection_coordinates = mapping(line.intersection(shape).boundary)['coordinates']
    closest_point = min([[euclideanDist([point.x, point.y], inter_point), inter_point] for inter_point in intersection_coordinates],
                        key=itemgetter(0))[1]
    # Get a little closer to the inside.
    delta_x = (center_of_shape.x - closest_point[0]) * geo_config["Relative point coeff"]
    delta_y = (center_of_shape.y - closest_point[1]) * geo_config["Relative point coeff"]
    closest_point = [closest_point[0] + delta_x, closest_point[1] + delta_y]
    return closest_point

def getShapeGroups(point_radius_list):

    def getShapeGroupsNaive(point_radius_list):
        pols = list(getCircleObjectsFromList(point_radius_list))
        intersections = []  # List of tuples
        for pol_1 in pols:
            for pol_2 in pols:
                if pol_1 is pol_2 or not pol_1.intersects(pol_2):
                    continue
                curr_int = pol_1.intersection(pol_2)
                # Check if we already have an intersection which takes this
                exists = False
                for s in range(len(intersections)):
                    if curr_int.intersects(intersections[s][0]):
                        intersections[s] = [curr_int.intersection(intersections[s][0]), intersections[s][1]]
                        try:
                            pols.remove(pol_1)
                            intersections[s][1] += [pol_1]
                        except ValueError:
                            pass
                        try:
                            pols.remove(pol_2)
                            intersections[s][1] += [pol_2]
                        except ValueError:
                            pass
                        exists = True
                if not exists:
                    intersections.append([curr_int, [pol_1, pol_2]])
                    try:
                        pols.remove(pol_1)
                    except ValueError:
                        pass
                    try:
                        pols.remove(pol_2)
                    except ValueError:
                        pass

        return intersections

    perms = []
    def getAllPointPermutations(point_list):
        def swap(list, a, b):
            list[a], list[b] = list[b], list[a]
            return list

        def permute(point_list, l, r):
            if r == l:
                perms.append(point_list.copy())
            else:
                for i in range(l, r + 1):
                    point_list = swap(point_list, l, i)
                    permute(point_list, l + 1, r)
                    point_list = swap(point_list, l, i)

        return permute(point_list, 0, len(point_list) - 1)

    def getShapeGroupsMinimizeClusters():
        getAllPointPermutations(point_radius_list)
        lowest = float("inf")
        lowest_intersections = []
        for perm in perms:
            intersections = getShapeGroupsNaive(perm)
            if len(intersections) < lowest:
                lowest = len(intersections)
                lowest_intersections = intersections.copy()
            if lowest == 1:
                return intersections

        return lowest_intersections

    if path_planning_algorithm_config["Minimize Clusters"] == "No":
        return getShapeGroupsNaive(point_radius_list)
    return getShapeGroupsMinimizeClusters()


def getIntersectionRepresentativePointsFromCircles(point_radius_list):
    shape_groups = getShapeGroups(point_radius_list)

    for group in shape_groups:
        yield group[0].representative_point()

# returning tuples to have similar format to shpae groups
def getCirclesNotInIntersections(point_radius_list):
    circles = list(getCircleObjectsFromList(point_radius_list))
    intersections = getIntersectionsFromCircles(circles)
    circles_tmp = circles.copy()
    ret = []
    for intersection in intersections:
        for circle in circles:
            if circle.intersects(intersection):
                # Keep count of circles that aren't in any intersection
                if circle in circles_tmp:
                    circles_tmp.remove(circle)
    for circle in circles_tmp:
        ret.append([circle, [circle]])

    return ret

def getCirclePointsNotInIntersection(point_radius_list):
    circles = getCirclesNotInIntersections(point_radius_list)
    ret = []
    for circle in circles:
        ret.append(circle[0].representative_point())

    return ret

def getIntersectionPointsForShapes(point_radius_list):
    node_points = list(getIntersectionRepresentativePointsFromCircles(point_radius_list))
    node_points += getCirclePointsNotInIntersection(point_radius_list)
    for point in node_points:
        yield [point.x, point.y]

def calcLimitsFromPoints(point_list, coeff = 0):
    ux, uy, lx, ly = 0, 0, float("inf"), float("inf")
    for point_radius in point_list:
        ux = max(ux, point_radius[1])
        uy = max(uy, point_radius[0])
        lx = min(lx, point_radius[1])
        ly = min(ly, point_radius[0])
    # Make the grid slightly bigger to allow path around
    new_lx = lx - (ux - lx) * coeff
    new_ly = ly - (uy - ly) * coeff
    new_ux = ux + (ux - lx) * coeff
    new_uy = uy + (uy - ly) * coeff

    return new_lx, new_ly, new_ux, new_uy

def getGraphFromPointLimits(lx, ly, ux, uy, grid_size):
    x_step = (ux - lx) / grid_size
    y_step = (uy - ly) / grid_size
    graph = {}
    for idx, x in enumerate(np.arange(lx, ux, x_step)):
        for idy, y in enumerate(np.arange(ly, uy, y_step)):
            graph[str(idx) + "," + str(idy)] = []

    return graph

def calcDistances(x_step, y_step):
    x_dist = x_step
    y_dist = y_step
    diagonal_dist = (x_step**2 + y_step**2)**0.5
    return x_dist, y_dist, diagonal_dist
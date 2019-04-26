from shapely.geometry import Point
import numpy as np


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

def getIntersectionRepresentativePointsFromCircles(pols):
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
                    intersections[s] = curr_int.intersection(intersections[s])
                    exists = True
            if not exists:
                intersections.append(curr_int)

    for int in intersections:
        yield int.representative_point()

def getCirclePointsNotInIntersection(pols):
    circles = list(getCircleObjectsFromList(pols))
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
        ret.append(circle.representative_point())

    return ret

def getIntersectionPointsForShapes(point_radius_list):
    circles = list(getCircleObjectsFromList(point_radius_list))
    node_points = list(getIntersectionRepresentativePointsFromCircles(circles))
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
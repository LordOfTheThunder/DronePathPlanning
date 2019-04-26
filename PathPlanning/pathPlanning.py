from operator import itemgetter
from enum import Enum
from Geo import GeoHelpers

class TravelingSalesmanTypes(Enum):
    Heuristic = 1
    BruteForce = 2

# Regular path planning for points
def travelingSalesman(start_point, point_list, alg_type=TravelingSalesmanTypes.Heuristic):

    def euclideanDist(first_point, second_point):
        return ((first_point[0] - second_point[0])**2 + (first_point[1] - second_point[1])**2)**0.5

    def bruteForceTravelingSalesman():
        paths = []
        def getAllPointPermutations(point_list):
            def swap(list, a, b):
                list[a], list[b] = list[b], list[a]
                return list
            def permute(point_list, l, r):
                if r == l:
                    paths.append(point_list.copy())
                else:
                    for i in range(l, r + 1):
                        point_list = swap(point_list, l, i)
                        permute(point_list, l+1, r)
                        point_list = swap(point_list, l, i)

            return permute(point_list, 0, len(point_list) - 1)
        def getPathCost(point_list):
            cost = euclideanDist(start_point, point_list[0])
            curr_point = point_list[0]
            point_list.remove(point_list[0])
            for point in point_list:
                cost += euclideanDist(curr_point, point)
                curr_point = point
            return cost

        getAllPointPermutations(point_list.copy())
        min_path = paths[0]
        min_cost = getPathCost(min_path.copy())
        for path in paths:
            current_cost = getPathCost(path.copy())
            if current_cost < min_cost:
                min_cost = current_cost
                min_path = path

        return min_path


    def heuristicTravelingSalesman():
        curr_point = start_point
        while point_list:
            point_dist_list = [[point, euclideanDist(curr_point, point)] for point in point_list]
            closest_point = min(point_dist_list, key=itemgetter(1))[0]
            point_list.remove(closest_point)
            curr_point = closest_point
            yield closest_point

    if alg_type == TravelingSalesmanTypes.Heuristic:
        return heuristicTravelingSalesman()
    if alg_type == TravelingSalesmanTypes.BruteForce:
        return bruteForceTravelingSalesman()

# Adapted path planning for points with radii
def advancedTravelingSalesman(start_point, point_radius_list, alg_type=TravelingSalesmanTypes.Heuristic):
    def createNodes():
        circles = list(GeoHelpers.getCircleObjectsFromList(point_radius_list))
        node_points = list(GeoHelpers.getIntersectionRepresentativePointsFromCircles(circles))
        node_points += GeoHelpers.getCirclePointsNotInIntersection(point_radius_list)
        for point in node_points:
            yield [point.x, point.y]
    def bruteForceAdvancedTS():
        node_points = list(createNodes())
        return travelingSalesman(start_point, node_points, TravelingSalesmanTypes.BruteForce)
    def heuristicAdvancedTS():
        node_points = list(createNodes())
        return travelingSalesman(start_point, node_points, TravelingSalesmanTypes.Heuristic)

    if alg_type == TravelingSalesmanTypes.Heuristic:
        return heuristicAdvancedTS()
    if alg_type == TravelingSalesmanTypes.BruteForce:
        return bruteForceAdvancedTS()


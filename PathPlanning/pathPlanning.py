from operator import itemgetter
from enum import Enum
from Geo import GeoHelpers
from MainConfig import path_planning_algorithm_config
from shapely.geometry import Point

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
        return GeoHelpers.getIntersectionPointsForShapes(point_radius_list)
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

def obstacleTravelingSalesman(start_point, point_radius_list, obstacle_bboxes, alg_type=TravelingSalesmanTypes.Heuristic):

    def pointToString(point):
        return str(point[0]) + "," + str(point[1])

    def createNodes():
        return GeoHelpers.getIntersectionPointsForShapes(point_radius_list)

    # Create grid from points - calculate map bl and ur and divide by grid size for nodes
    sensor_nodes = list(createNodes())
    node_points = sensor_nodes.copy()
    node_points.append(start_point)
    grid_size = path_planning_algorithm_config["Grid Size"]
    # Each node will be of type box
    lx, ly, ux, uy = GeoHelpers.calcLimitsFromPoints(node_points, path_planning_algorithm_config["Grid Coeff"])

    def calcDistances():
        x_step = (ux - lx) / grid_size
        y_step = (uy - ly) / grid_size
        x_dist = x_step
        y_dist = y_step
        diagonal_dist = (x_step**2 + y_step**2)**0.5
        return x_dist, y_dist, diagonal_dist

    x_dist, y_dist, diagonal_dist = calcDistances()

    def createGraph():
        graph = GeoHelpers.getGraphFromPointLimits(lx, ly, ux, uy, grid_size)
        # Find target nodes in which our node points are located
        start_point_coords = [(int((start_point[1] - lx) / (ux - lx) * grid_size)),
                              int((start_point[0] - ly) / (uy - ly) * grid_size)]
        graph[pointToString(start_point_coords)].append("start")
        for sensor_node in sensor_nodes:
            graph[pointToString([int((sensor_node[1] - lx) / (ux - lx) * grid_size),
                                        int((sensor_node[0] - ly) / (uy - ly) * grid_size)])].append("sensor")
        # Find nodes in which we have obstacles
        for bbox in obstacle_bboxes:
            ll, ur = [bbox[0], bbox[1]], [bbox[2], bbox[3]]
            bbox_lx = int((ll[1] - lx) / (ux - lx) * grid_size)
            bbox_ux = int((ur[1] - lx) / (ux - lx) * grid_size)
            bbox_ly = int((ll[0] - ly) / (uy - ly) * grid_size)
            bbox_uy = int((ur[0] - ly) / (uy - ly) * grid_size)
            for i in range(bbox_lx, bbox_ux + 1):
                for j in range(bbox_ly, bbox_uy + 1):
                    graph[pointToString([i, j])].append("obstacle")

        return graph, start_point_coords

    def getNeighbours(node):
        # X neighbors
        if node[0] + 1 < grid_size:
            yield [[node[0] + 1, node[1]], x_dist]
        if node[0] - 1 >= 0:
            yield [[node[0] - 1, node[1]], x_dist]
        # Y neighbors
        if node[1] + 1 < grid_size:
            yield [[node[0], node[1] + 1], y_dist]
        if node[1] - 1 >= 0:
            yield [[node[0], node[1] - 1], y_dist]
        # Diagonal neighbors
        if node[0] + 1 < grid_size and node[1] + 1 < grid_size:
            yield [[node[0] + 1, node[1] + 1], diagonal_dist]
        if node[0] + 1 < grid_size and node[1] - 1 >= 0:
            yield [[node[0] + 1, node[1] - 1], diagonal_dist]
        if node[0] - 1 >= 0 and node[1] + 1 < grid_size:
            yield [[node[0] - 1, node[1] + 1], diagonal_dist]
        if node[0] - 1 >= 0 and node[1] - 1 >= 0:
            yield [[node[0] - 1, node[1] - 1], diagonal_dist]

    def dijkstra(graph, start_point_coords):
        dist = {}
        for key in graph:
            dist[key] = float("inf")

        dist[str(start_point_coords[0]) + "," + str(start_point_coords[1])] = 0
        curr_node = start_point_coords
        while graph:
            neighbors = list(getNeighbours(curr_node))
            for neighbor in neighbors:
                neighbor_coord, neighbor_dist = neighbor
                # get closest neighbor, continue dijkstra from here tomorrow


    graph, start_point_coords = createGraph()
    # From start point to any other point
    dijkstra(graph, start_point_coords)
    print("x")
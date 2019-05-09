from operator import itemgetter
from enum import Enum
from Geo import GeoHelpers
from MainConfig import path_planning_algorithm_config
from PathPlanning import Graph, pathPlanningHelpers
from shapely.geometry import Point

class TravelingSalesmanTypes(Enum):
    Heuristic = 1
    BruteForce = 2

# Regular path planning for points

def euclideanDist(first_point, second_point):
    return ((first_point[0] - second_point[0])**2 + (first_point[1] - second_point[1])**2)**0.5

def travelingSalesman(start_point, point_list, mapping_function = euclideanDist, alg_type=TravelingSalesmanTypes.Heuristic):

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
            cost = mapping_function(start_point, point_list[0])
            curr_point = point_list[0]
            point_list.remove(point_list[0])
            for point in point_list:
                cost += mapping_function(curr_point, point)
                curr_point = point
            cost += mapping_function(point_list[len(point_list) - 1], start_point)
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
            point_dist_list = [[point, mapping_function(curr_point, point)] for point in point_list]
            closest_point = min(point_dist_list, key=itemgetter(1))[0]
            point_list.remove(closest_point)
            curr_point = closest_point
            yield closest_point

    if alg_type == TravelingSalesmanTypes.Heuristic:
        return heuristicTravelingSalesman()
    if alg_type == TravelingSalesmanTypes.BruteForce:
        return bruteForceTravelingSalesman()

# Adapted path planning for points with radii
def advancedTravelingSalesman(start_point, point_radius_list, mapping_function = euclideanDist, alg_type=TravelingSalesmanTypes.Heuristic):
    def createNodes():
        return GeoHelpers.getIntersectionPointsForShapes(point_radius_list)
    def bruteForceAdvancedTS():
        node_points = list(createNodes())
        return travelingSalesman(start_point, node_points, mapping_function, TravelingSalesmanTypes.BruteForce)
    def heuristicAdvancedTS():
        node_points = list(createNodes())
        return travelingSalesman(start_point, node_points, mapping_function, TravelingSalesmanTypes.Heuristic)

    if alg_type == TravelingSalesmanTypes.Heuristic:
        return heuristicAdvancedTS()
    if alg_type == TravelingSalesmanTypes.BruteForce:
        return bruteForceAdvancedTS()

point_cost_paths = {}

def obstacleTravelingSalesman(start_point, point_radius_list, obstacle_bboxes, alg_type=TravelingSalesmanTypes.Heuristic):
    # Create grid from points - calculate map bl and ur and divide by grid size for nodes
    sensor_nodes = list(Graph.createNodes(point_radius_list))
    node_points = sensor_nodes.copy()
    node_points.append(start_point)
    grid_size = path_planning_algorithm_config["Grid Size"]
    # Each node will be of type box
    board_limits = lx, ly, ux, uy = GeoHelpers.calcLimitsFromPoints(node_points, path_planning_algorithm_config["Grid Coeff"])
    x_step = (ux - lx) / grid_size
    y_step = (uy - ly) / grid_size

    graph, start_point_coords = Graph.createGraph(start_point, sensor_nodes, obstacle_bboxes, board_limits)
    graph = Graph.checkValidGrid(graph)

    def getGridPointFromPoint(point):
        grid_x = int((point[1] - lx) / (ux - lx) * grid_size)
        grid_y = int((point[0] - ly) / (uy - ly) * grid_size)
        return [grid_x, grid_y]

    def calcDist(point):
        point_cost_paths[pathPlanningHelpers.pointToString(getGridPointFromPoint(point))] = Graph.dijkstra(graph, getGridPointFromPoint(point))

    def getDist(first_point, second_point):
        # Convert points to grid
        first_point = getGridPointFromPoint(first_point)
        second_point = getGridPointFromPoint(second_point)
        # Find first point in point_cost_paths
        if pathPlanningHelpers.pointToString(first_point) not in point_cost_paths:
            point_cost_paths[pathPlanningHelpers.pointToString(first_point)] = Graph.dijkstra(graph, first_point)
        return point_cost_paths[pathPlanningHelpers.pointToString(first_point)][pathPlanningHelpers.pointToString(second_point)][1]

    def getDynamicDist(first_point, second_point, groups):
        # Convert points to grid
        first_point_grid = getGridPointFromPoint(first_point)
        second_point_grid = getGridPointFromPoint(second_point)
        first_point_str = pathPlanningHelpers.pointToString(first_point_grid)
        second_point_str = pathPlanningHelpers.pointToString(second_point_grid)
        # Check if we already calculated this path previously
        if first_point_str in point_cost_paths:
            if second_point_str in point_cost_paths[first_point_str]:
                # We don't need to calculate new paths
                return point_cost_paths[first_point_str][second_point_str]
        else:
            # We need to build new graph and calculate paths and update them in point_cost_path
            dynamic_nodes = []
            for group in groups:
                dynamic_nodes += [GeoHelpers.getClosestPointFromPointToShape(Point(first_point[0], first_point[1]), group[0])]
            graph, start_point_coords = Graph.createGraph(first_point, dynamic_nodes, obstacle_bboxes, board_limits)
            graph = Graph.checkValidGrid(graph)
            point_cost_paths[first_point_str] = Graph.dijkstra(graph, first_point_grid)
            return point_cost_paths[first_point_str][second_point_str]


    def getRepPointFromGrid(grid_loc):
        grid_loc = [int(x) for x in grid_loc.split(',')]
        rep_x = lx + grid_loc[0] * x_step + x_step / 2
        rep_y = ly + grid_loc[1] * y_step + y_step / 2
        return [rep_y, rep_x]

    def calcNewPathFromPathAndGrid():
        new_path = []
        curr_point = getGridPointFromPoint(start_point)

        total_length = 0

        for point in ts_path:
            path_dist = point_cost_paths[pathPlanningHelpers.pointToString(curr_point)][
                pathPlanningHelpers.pointToString(getGridPointFromPoint(point))]
            total_length += path_dist[1]
            for grid_point in path_dist[0]:
                new_path.append(getRepPointFromGrid(grid_point))
            curr_point = getGridPointFromPoint(point)

        # Calculate path from last point to start point
        calcDist(ts_path[-1])
        path_dist = point_cost_paths[pathPlanningHelpers.pointToString(curr_point)][pathPlanningHelpers.pointToString(getGridPointFromPoint(start_point))]
        total_length += path_dist[1]
        for grid_point in path_dist[0]:
            new_path.append(getRepPointFromGrid(grid_point))
        return new_path, total_length

    def heuristicObstacleTS():
        # return travelingSalesman(start_point, sensor_nodes, getDist, TravelingSalesmanTypes.Heuristic)
        return dynamicTravelingSalesman(start_point, point_radius_list, getDynamicDist, TravelingSalesmanTypes.Heuristic)
    def bruteForceObstacleTS():
        return travelingSalesman(start_point, sensor_nodes, getDist, TravelingSalesmanTypes.BruteForce)

    if alg_type == TravelingSalesmanTypes.Heuristic:
        ts_path = list(heuristicObstacleTS())
        return calcNewPathFromPathAndGrid()

    if alg_type == TravelingSalesmanTypes.BruteForce:
        ts_path = list(bruteForceObstacleTS())
        return calcNewPathFromPathAndGrid()

def dynamicTravelingSalesman(start_point, point_radius_list, mapping_function, alg_type=TravelingSalesmanTypes.Heuristic):

    def bruteForceTravelingSalesman():
        pass

    def heuristicTravelingSalesman():
        # Setup groups of intersecting sensors given a group of point radius list
        groups = GeoHelpers.getShapeGroups(point_radius_list)
        groups += GeoHelpers.getCirclesNotInIntersections(point_radius_list)

        curr_point = start_point
        new_point_list_radius = point_radius_list.copy()
        while groups:
            point_dist_list = [[GeoHelpers.getClosestPointFromPointToShape(Point(curr_point[0], curr_point[1]), group[0]),
                                mapping_function(curr_point, GeoHelpers.getClosestPointFromPointToShape(Point(curr_point[0], curr_point[1]), group[0]), groups), group] for group in groups]
            # Calculate distances from newly calculate points
            closest_point = []
            min_path = float("inf")
            group_to_remove = []
            for point_dist in point_dist_list:
                curr_path_dist = point_dist[1][1]
                if curr_path_dist < min_path:
                    closest_point = point_dist[0]
                    group_to_remove = point_dist[2]
                    min_path = curr_path_dist

            curr_point = list(closest_point)
            yield curr_point
            # Recalculate groups
            for group_lm in group_to_remove[1]:
                for point in new_point_list_radius:
                    point_obj = Point(point[0], point[1])
                    if point_obj.within(group_lm):
                        new_point_list_radius.remove(point)
            groups = GeoHelpers.getShapeGroups(new_point_list_radius)
            groups += GeoHelpers.getCirclesNotInIntersections(new_point_list_radius)

    if alg_type == TravelingSalesmanTypes.Heuristic:
        return heuristicTravelingSalesman()
    if alg_type == TravelingSalesmanTypes.BruteForce:
        return bruteForceTravelingSalesman()
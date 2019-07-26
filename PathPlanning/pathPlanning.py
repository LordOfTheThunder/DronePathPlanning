from operator import itemgetter
from enum import Enum
from Geo import GeoHelpers
from MainConfig import path_planning_algorithm_config, waypoint_config
from PathPlanning import Graph, pathPlanningHelpers
from shapely.geometry import Point, LineString, Polygon
from PathPlanning.pathPlanningHelpers import euclideanDist

class TravelingSalesmanTypes(Enum):
    Heuristic = 1
    BruteForce = 2
    HeuristicDynamic = 3
    BruteForceDynamic = 4
    HeuristicOptimizedDynamic = 5
    BruteForceOptimizedDynamic = 6

# Regular path planning for points

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

        min_path = [[waypoint_config["stop command"], point] for point in min_path]
        return min_path


    def heuristicTravelingSalesman():
        curr_point = start_point
        while point_list:
            point_dist_list = [[point, mapping_function(curr_point, point)] for point in point_list]
            closest_point = min(point_dist_list, key=itemgetter(1))[0]
            point_list.remove(closest_point)
            curr_point = closest_point
            yield [waypoint_config["stop command"], closest_point]

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

    def calcDist(first_point, second_point):
        # Check if we can go in straight line first
        line = LineString([(first_point[0], first_point[1]), (second_point[0], second_point[1])])
        first_point_grid, second_point_grid, first_point_str, second_point_str = getGridAndStringFromPoint(first_point, second_point)

        if not checkIntersectionWithObstacles(line):
            calcStraightLinePathBetweenPoints(first_point, second_point)
            return point_cost_paths[first_point_str]

        point_cost_paths[pathPlanningHelpers.pointToString(getGridPointFromPoint(first_point))] = Graph.dijkstra(graph, getGridPointFromPoint(first_point))
        return point_cost_paths[pathPlanningHelpers.pointToString(getGridPointFromPoint(first_point))]

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
        if first_point_str in point_cost_paths and second_point_str in point_cost_paths[first_point_str]:
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


    # Calculate obstacle polygons for intersection
    def bboxToPolygon(bbox):
        bl = [bbox[0], bbox[1]]
        ur = [bbox[2], bbox[3]]
        br = [bbox[2], bbox[1]]
        ul = [bbox[0], bbox[3]]
        return [bl, br, ur, ul]

    obstacle_polygons = [Polygon(bboxToPolygon(bbox)) for bbox in obstacle_bboxes]

    def checkIntersectionWithObstacles(line):
        for polygon in obstacle_polygons:
            if polygon.intersects(line):
                return True
        return False

    def calcStraightLinePathBetweenPoints(first_point, second_point):
        first_point_grid, second_point_grid, first_point_str, second_point_str = getGridAndStringFromPoint(first_point, second_point)
        # We can go from point to point in a straight line
        path_res = [[second_point_str], euclideanDist(first_point, second_point)]
        target_paths = {
            second_point_str: path_res
        }
        if first_point_str not in point_cost_paths:
            point_cost_paths[first_point_str] = target_paths
        else:
            point_cost_paths[first_point_str][second_point_str] = path_res

    def getGridAndStringFromPoint(first_point, second_point):
        first_point_grid = getGridPointFromPoint(first_point)
        second_point_grid = getGridPointFromPoint(second_point)
        first_point_str = pathPlanningHelpers.pointToString(first_point_grid)
        second_point_str = pathPlanningHelpers.pointToString(second_point_grid)
        return first_point_grid, second_point_grid, first_point_str, second_point_str

    def getOptimizedDynamicDist(first_point, second_point, groups):
        # Try to first see if straight line path is possible
        line = LineString([(first_point[0], first_point[1]), (second_point[0], second_point[1])])
        first_point_grid, second_point_grid, first_point_str, second_point_str = getGridAndStringFromPoint(first_point, second_point)

        if not checkIntersectionWithObstacles(line):
            calcStraightLinePathBetweenPoints(first_point, second_point)
            return point_cost_paths[first_point_str][second_point_str]

        return getDynamicDist(first_point, second_point, groups)

    def getRepPointFromGrid(grid_loc):
        grid_loc = [int(x) for x in grid_loc.split(',')]
        rep_x = lx + grid_loc[0] * x_step + x_step / 2
        rep_y = ly + grid_loc[1] * y_step + y_step / 2
        return [rep_y, rep_x]

    def calcNewPathFromPathAndGrid():
        new_path = []
        curr_point = getGridPointFromPoint(start_point)

        total_length = 0

        for action, point in ts_path:
            path_dist = point_cost_paths[pathPlanningHelpers.pointToString(curr_point)][
                pathPlanningHelpers.pointToString(getGridPointFromPoint(point))]
            total_length += path_dist[1]
            for grid_point in path_dist[0]:
                new_path.append([waypoint_config["go command"], getRepPointFromGrid(grid_point)])
            # stop at last point
            new_path[-1][0] = waypoint_config["stop command"]
            curr_point = getGridPointFromPoint(point)

        # Calculate path from last point to start point
        last_action, last_coord = ts_path[-1]
        prev_last_action, prev_last_coord = ts_path[-2]
        calcDist(prev_last_coord, last_coord)
        path_dist = point_cost_paths[pathPlanningHelpers.pointToString(curr_point)][pathPlanningHelpers.pointToString(getGridPointFromPoint(start_point))]
        total_length += path_dist[1]
        for grid_point in path_dist[0]:
            new_path.append([waypoint_config["go command"], getRepPointFromGrid(grid_point)])
        return new_path, total_length

    extra_funcs = {
        "Point to grid": getGridPointFromPoint,
        "Grid to point": getRepPointFromGrid,
        "Calc Dist": calcDist,
    }

    def heuristicObstacleTS():
        return travelingSalesman(start_point, sensor_nodes, getDist, TravelingSalesmanTypes.Heuristic)
    def bruteForceObstacleTS():
        return travelingSalesman(start_point, sensor_nodes, getDist, TravelingSalesmanTypes.BruteForce)
    def heuristicDynamicObstacleTS():
        return dynamicTravelingSalesman(start_point, point_radius_list, getDynamicDist, TravelingSalesmanTypes.HeuristicDynamic)
    def bruteForceDynamicObstacleTS():
        return dynamicTravelingSalesman(start_point, point_radius_list, getDynamicDist, TravelingSalesmanTypes.BruteForceDynamic, extra_funcs)
    def heuristicDynamicOptimizedObstacleTS():
        return dynamicTravelingSalesman(start_point, point_radius_list, getOptimizedDynamicDist, TravelingSalesmanTypes.HeuristicDynamic)
    def bruteForceDynamicOptimizedObstacleTS():
        return dynamicTravelingSalesman(start_point, point_radius_list, getOptimizedDynamicDist, TravelingSalesmanTypes.BruteForceDynamic, extra_funcs)

    if alg_type == TravelingSalesmanTypes.Heuristic:
        ts_path = list(heuristicObstacleTS())
        return calcNewPathFromPathAndGrid()

    if alg_type == TravelingSalesmanTypes.BruteForce:
        ts_path = list(bruteForceObstacleTS())
        return calcNewPathFromPathAndGrid()

    if alg_type == TravelingSalesmanTypes.HeuristicDynamic:
        ts_path = list(heuristicDynamicObstacleTS())
        return calcNewPathFromPathAndGrid()

    if alg_type == TravelingSalesmanTypes.BruteForceDynamic:
        # For this one path is already calculated
        return bruteForceDynamicObstacleTS()

    if alg_type == TravelingSalesmanTypes.HeuristicOptimizedDynamic:
        ts_path = list(heuristicDynamicOptimizedObstacleTS())
        return calcNewPathFromPathAndGrid()

    if alg_type == TravelingSalesmanTypes.BruteForceOptimizedDynamic:
        return bruteForceDynamicOptimizedObstacleTS()

def dynamicTravelingSalesman(start_point, point_radius_list, mapping_function, alg_type=TravelingSalesmanTypes.HeuristicDynamic, extra_funcs_hash=None):

    def bruteForceTravelingSalesman():
        # Setup groups of intersecting sensors given a group of point radius list
        groups = GeoHelpers.getShapeGroups(point_radius_list)
        groups += GeoHelpers.getCirclesNotInIntersections(point_radius_list)

        path = []
        path_dist = float("inf")
        stop_points = []

        def findPathRecursively(groups, curr_path=None, curr_path_dist=0, new_point_radius_list=point_radius_list, curr_point=start_point, curr_stop_points=[]):
            nonlocal path_dist, path, stop_points
            if not curr_path:
                curr_path = [pathPlanningHelpers.pointToString(extra_funcs_hash["Point to grid"](start_point))]
            tmp_curr_path = curr_path.copy()
            tmp_curr_path_dist = curr_path_dist
            tmp_new_point_radius_list = new_point_radius_list.copy()
            tmp_curr_stop_points = curr_stop_points.copy()
            for group in groups:
                next_point = GeoHelpers.getClosestPointFromPointToShape(Point(curr_point[0], curr_point[1]), group[0])
                path_and_cost_to_next_point = mapping_function(curr_point,
                                 GeoHelpers.getClosestPointFromPointToShape(Point(curr_point[0], curr_point[1]),
                                                                            group[0]), groups)
                if len(path_and_cost_to_next_point[0]) > 1:
                    curr_path += path_and_cost_to_next_point[0][1:]
                else:
                    curr_path += path_and_cost_to_next_point[0]
                curr_stop_points.append(path_and_cost_to_next_point[0][-1])
                curr_path_dist += path_and_cost_to_next_point[1]

                if curr_path_dist > path_dist:
                    curr_path = tmp_curr_path.copy()
                    curr_path_dist = tmp_curr_path_dist
                    new_point_radius_list = tmp_new_point_radius_list.copy()
                    curr_stop_points = tmp_curr_stop_points.copy()
                    continue

                # Recalculate groups based on what we removed and go in recursively
                for group_lm in group[1]:
                    for point in new_point_radius_list:
                        point_obj = Point(point[0], point[1])
                        circle = point_obj.buffer(point[2])
                        if circle.within(group_lm):
                            new_point_radius_list.remove(point)
                groups_alt = GeoHelpers.getShapeGroups(new_point_radius_list)
                groups_alt += GeoHelpers.getCirclesNotInIntersections(new_point_radius_list)

                if not groups_alt:
                    # Calculate path from last point to start point
                    path_to_start, cost_to_start = extra_funcs_hash["Calc Dist"](next_point, start_point)[curr_path[0]]
                    if len(path_to_start) > 1:
                        curr_path += path_to_start[1:]
                    else:
                        curr_path += path_to_start

                    curr_path_dist += cost_to_start
                    if curr_path_dist < path_dist:
                        path_dist = curr_path_dist
                        path = curr_path.copy()
                        stop_points = curr_stop_points.copy()
                    return

                findPathRecursively(groups_alt.copy(), curr_path.copy(), curr_path_dist, new_point_radius_list.copy(), next_point, curr_stop_points)
                curr_path = tmp_curr_path.copy()
                curr_path_dist = tmp_curr_path_dist
                new_point_radius_list = tmp_new_point_radius_list.copy()
                curr_stop_points = tmp_curr_stop_points.copy()

        findPathRecursively(groups)
        # Update path based on stop points
        new_path = [[waypoint_config["stop command"], extra_funcs_hash["Grid to point"](point)]
                    if point in stop_points else [waypoint_config["go command"], extra_funcs_hash["Grid to point"](point)]
                    for point in path]
        return new_path, path_dist

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
            yield [waypoint_config["stop command"], curr_point]
            # Recalculate groups
            for group_lm in group_to_remove[1]:
                for point in new_point_list_radius:
                    point_obj = Point(point[0], point[1])
                    if point_obj.within(group_lm):
                        new_point_list_radius.remove(point)
            groups = GeoHelpers.getShapeGroups(new_point_list_radius)
            groups += GeoHelpers.getCirclesNotInIntersections(new_point_list_radius)

    if alg_type == TravelingSalesmanTypes.HeuristicDynamic:
        return heuristicTravelingSalesman()
    if alg_type == TravelingSalesmanTypes.BruteForceDynamic:
        return bruteForceTravelingSalesman()

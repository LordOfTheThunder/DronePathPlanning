from operator import itemgetter
from enum import Enum
from Geo import GeoHelpers
from MainConfig import path_planning_algorithm_config
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
    x_step = (ux - lx) / grid_size
    y_step = (uy - ly) / grid_size

    def calcDistances():
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

    def getNodeWithMinDist(Q, dist, graph):
        min_node, min_dist = None, float("inf")
        for node in Q:
            if dist[node] < min_dist and graph[node] != 'obstacle':
                min_dist = dist[node]
                min_node = node

        return min_node

    def checkValidGrid(graph):
        for key in graph:
            if len(graph[key]) > 1 and 'sensor' in graph[key] and 'obstacle' in graph[key]:
                raise Exception("Bad grid dimensions. Obstacle and sensor on the same cell")
            if len(graph[key]) == 1:
                graph[key] = graph[key][0]
            else:
                graph[key] = ""
        return graph

    def getNeighbors(key, graph, Q):
        # X neighbors
        node = [int(i) for i in key.split(',')]
        point = [node[0] + 1, node[1]]
        if point[0] < grid_size and graph[pointToString(point)] != 'obstacle' and pointToString(point) in Q:
            yield [point, x_dist]
        point = [node[0] - 1, node[1]]
        if point[0] >= 0 and graph[pointToString(point)] != 'obstacle' and pointToString(point) in Q:
            yield [point, x_dist]
        # Y neighbors
        point = [node[0], node[1] + 1]
        if point[1] < grid_size and graph[pointToString(point)] != 'obstacle' and pointToString(point) in Q:
            yield [point, y_dist]
        point = [node[0], node[1] - 1]
        if point[1] >= 0 and graph[pointToString(point)] != 'obstacle' and pointToString(point) in Q:
            yield [point, y_dist]
        # Diagonal neighbors
        point = [node[0] + 1, node[1] + 1]
        if point[0] < grid_size and point[1] < grid_size and graph[pointToString(point)] != 'obstacle' and pointToString(point) in Q:
            yield [point, diagonal_dist]
        point = [node[0] + 1, node[1] - 1]
        if point[0] < grid_size and point[1] >= 0 and graph[pointToString(point)] != 'obstacle' and pointToString(point) in Q:
            yield [point, diagonal_dist]
        point = [node[0] - 1, node[1] + 1]
        if point[0] >= 0 and point[1] < grid_size and graph[pointToString(point)] != 'obstacle' and pointToString(point) in Q:
            yield [point, diagonal_dist]
        point = [node[0] - 1, node[1] - 1]
        if point[0] >= 0 and point[1] >= 0 and graph[pointToString(point)] != 'obstacle' and pointToString(point) in Q:
            yield [point, diagonal_dist]

    def dijkstra(graph, source):
        dist, prev, Q = {}, {}, []
        for key in graph:
            dist[key] = float("inf")
            prev[key] = None
            Q.append(key)

        # Calculate targets dynamically.
        # targets = (start_point + sensor_points) - start_point_coords
        targets = []
        dist[pointToString(source)] = 0
        while Q:
            u = getNodeWithMinDist(Q, dist, graph)
            # If only obstacle nodes are left
            if u is None:
                break

            Q.remove(u)
            if u != pointToString(source) and (graph[u] == 'sensor' or graph[u] == 'start'):
                targets.append(u)

            neighbors = list(getNeighbors(u, graph, Q))
            for neighbor in neighbors:
                neighbor_coord, neighbor_dist = neighbor
                alt = dist[u] + neighbor_dist
                if alt < dist[pointToString(neighbor_coord)]:
                    dist[pointToString(neighbor_coord)] = alt
                    prev[pointToString(neighbor_coord)] = u

        # Get shortest path to each of the other nodes from source node
        target_paths = {}
        for target in targets:
            S = []
            u = target
            if prev[u] or u == source:
                while u:
                    S.append(u)
                    u = prev[u]
            S.reverse()
            target_paths[target] = [S, dist[target]]

        return target_paths

    graph, start_point_coords = createGraph()
    graph = checkValidGrid(graph)

    def getGridPointFromPoint(point):
        grid_x = int((point[1] - lx) / (ux - lx) * grid_size)
        grid_y = int((point[0] - ly) / (uy - ly) * grid_size)
        return [grid_x, grid_y]

    point_cost_paths = {}
    for source in [start_point] + sensor_nodes:
        point_cost_paths[pointToString(getGridPointFromPoint(source))] = dijkstra(graph, getGridPointFromPoint(source))

    def getDist(first_point, second_point):
        # Convert points to grid
        first_point = getGridPointFromPoint(first_point)
        second_point = getGridPointFromPoint(second_point)
        # Find first point in point_cost_paths
        for key in point_cost_paths:
            if key == pointToString(first_point):
                return point_cost_paths[key][pointToString(second_point)][1]

    def getRepPointFromGrid(grid_loc):
        grid_loc = [int(x) for x in grid_loc.split(',')]
        rep_x = lx + grid_loc[0] * x_step + x_step / 2
        rep_y = ly + grid_loc[1] * y_step + y_step / 2
        return [rep_y, rep_x]

    def calcNewPathFromPathAndGrid():
        new_path = []
        curr_point = getGridPointFromPoint(start_point)
        for point in ts_path + [start_point]:
            for grid_point in point_cost_paths[pointToString(curr_point)][pointToString(getGridPointFromPoint(point))][0]:
                new_path.append(getRepPointFromGrid(grid_point))
            curr_point = getGridPointFromPoint(point)
        return new_path

    def heuristicObstacleTS():
        return travelingSalesman(start_point, sensor_nodes, getDist, TravelingSalesmanTypes.Heuristic)
    def bruteForceObstacleTS():
        return travelingSalesman(start_point, sensor_nodes, getDist, TravelingSalesmanTypes.BruteForce)

    if alg_type == TravelingSalesmanTypes.Heuristic:
        ts_path = list(heuristicObstacleTS())
        return calcNewPathFromPathAndGrid()

    if alg_type == TravelingSalesmanTypes.BruteForce:
        ts_path = list(bruteForceObstacleTS())
        return calcNewPathFromPathAndGrid()

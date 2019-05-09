from Geo import GeoHelpers
from PathPlanning import pathPlanningHelpers
from MainConfig import path_planning_algorithm_config

grid_size = path_planning_algorithm_config["Grid Size"]

def createNodes(point_radius_list):
    return GeoHelpers.getIntersectionPointsForShapes(point_radius_list)


def createGraph(start_point, sensor_nodes, obstacle_bboxes, board_limits):
    node_points = sensor_nodes.copy()
    node_points.append(start_point)
    lx, ly, ux, uy = board_limits
    graph = GeoHelpers.getGraphFromPointLimits(lx, ly, ux, uy, grid_size)
    # Find target nodes in which our node points are located
    start_point_coords = [(int((start_point[1] - lx) / (ux - lx) * grid_size)),
                          int((start_point[0] - ly) / (uy - ly) * grid_size)]
    graph[pathPlanningHelpers.pointToString(start_point_coords)].append("start")
    for sensor_node in sensor_nodes:
        graph[pathPlanningHelpers.pointToString([int((sensor_node[1] - lx) / (ux - lx) * grid_size),
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
                graph[pathPlanningHelpers.pointToString([i, j])].append("obstacle")

    # Keep the node points in the graph structure as a graph attribute
    graph["attributes"] = {
        "node points" : node_points
    }
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
        if key == "attributes":
            continue
        if len(graph[key]) > 1 and 'sensor' in graph[key] and 'obstacle' in graph[key]:
            raise Exception("Bad grid dimensions. Obstacle and sensor on the same cell")
        if len(graph[key]) == 1:
            graph[key] = graph[key][0]
        else:
            graph[key] = ""
    return graph

def getNeighbors(key, graph, Q):
    lx, ly, ux, uy = GeoHelpers.calcLimitsFromPoints(graph["attributes"]["node points"], path_planning_algorithm_config["Grid Coeff"])
    x_step = (ux - lx) / grid_size
    y_step = (uy - ly) / grid_size
    x_dist, y_dist, diagonal_dist = GeoHelpers.calcDistances(x_step, y_step)
    # X neighbors
    node = [int(i) for i in key.split(',')]
    point = [node[0] + 1, node[1]]
    if point[0] < grid_size and graph[pathPlanningHelpers.pointToString(point)] != 'obstacle' and pathPlanningHelpers.pointToString(point) in Q:
        yield [point, x_dist]
    point = [node[0] - 1, node[1]]
    if point[0] >= 0 and graph[pathPlanningHelpers.pointToString(point)] != 'obstacle' and pathPlanningHelpers.pointToString(point) in Q:
        yield [point, x_dist]
    # Y neighbors
    point = [node[0], node[1] + 1]
    if point[1] < grid_size and graph[pathPlanningHelpers.pointToString(point)] != 'obstacle' and pathPlanningHelpers.pointToString(point) in Q:
        yield [point, y_dist]
    point = [node[0], node[1] - 1]
    if point[1] >= 0 and graph[pathPlanningHelpers.pointToString(point)] != 'obstacle' and pathPlanningHelpers.pointToString(point) in Q:
        yield [point, y_dist]
    # Diagonal neighbors
    point = [node[0] + 1, node[1] + 1]
    if point[0] < grid_size and point[1] < grid_size and graph[pathPlanningHelpers.pointToString(point)] != 'obstacle' and pathPlanningHelpers.pointToString(point) in Q:
        yield [point, diagonal_dist]
    point = [node[0] + 1, node[1] - 1]
    if point[0] < grid_size and point[1] >= 0 and graph[pathPlanningHelpers.pointToString(point)] != 'obstacle' and pathPlanningHelpers.pointToString(point) in Q:
        yield [point, diagonal_dist]
    point = [node[0] - 1, node[1] + 1]
    if point[0] >= 0 and point[1] < grid_size and graph[pathPlanningHelpers.pointToString(point)] != 'obstacle' and pathPlanningHelpers.pointToString(point) in Q:
        yield [point, diagonal_dist]
    point = [node[0] - 1, node[1] - 1]
    if point[0] >= 0 and point[1] >= 0 and graph[pathPlanningHelpers.pointToString(point)] != 'obstacle' and pathPlanningHelpers.pointToString(point) in Q:
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
    dist[pathPlanningHelpers.pointToString(source)] = 0
    while Q:
        u = getNodeWithMinDist(Q, dist, graph)
        # If only obstacle nodes are left
        if u is None:
            break

        Q.remove(u)
        if u != pathPlanningHelpers.pointToString(source) and (graph[u] == 'sensor' or graph[u] == 'start'):
            targets.append(u)

        neighbors = list(getNeighbors(u, graph, Q))
        for neighbor in neighbors:
            neighbor_coord, neighbor_dist = neighbor
            alt = dist[u] + neighbor_dist
            if alt < dist[pathPlanningHelpers.pointToString(neighbor_coord)]:
                dist[pathPlanningHelpers.pointToString(neighbor_coord)] = alt
                prev[pathPlanningHelpers.pointToString(neighbor_coord)] = u

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

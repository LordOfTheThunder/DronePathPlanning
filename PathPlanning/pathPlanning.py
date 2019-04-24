from operator import itemgetter

def travelingSalesman(start_point, point_list):

    def euclideanDist(first_point, second_point):
        return ((first_point[0] - second_point[0])**2 + (first_point[1] - second_point[1])**2)**0.5

    def bruteForceTravelingSalesman(start_point, point_list):
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


    def heuristicTravelingSalesman(start_point, point_list):
        curr_point = start_point
        while point_list:
            point_dist_list = ([point, euclideanDist(curr_point, point)] for point in point_list)
            closest_point = min(point_dist_list, key=itemgetter(1))[0]
            point_list.remove(closest_point)
            curr_point = closest_point
            yield closest_point

    # For now we will use the heuristic traveling salesman
    return heuristicTravelingSalesman(start_point, point_list)
    # return bruteForceTravelingSalesman(start_point, point_list)


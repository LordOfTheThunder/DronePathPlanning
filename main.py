from Waypoint import file
from PathPlanning import pathPlanning

if __name__ == "__main__":
    # file.createWaypointFile([[10, 10]])
    # print(list(pathPlanning.travelingSalesman([3, 3], [[5, 4], [4, 4]])))
    print(list(pathPlanning.travelingSalesman([3, 3], [[11, 3], [4, 4], [4.5, 7], [3, 5], [8, 7]])))
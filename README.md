# Drone Path Planning
A small project which creates a js simulation and a waypoint fly plan for a drone given an input of a start point, sensor locations and signal radii and obstacle locations. The project calculates the path between points using [PATH PLANNING
FOR UNMANNED AERIAL VEHICLES IN UNCERTAIN AND ADVERSARIAL ENVIRONMENTS∗](http://www.seas.ucla.edu/coopcontrol/papers/02cn04.pdf) and chooses a global path using traveling salesman.

## Getting Started
In order to get the project running you will need a working python 3.6 environment and an external shapely package installed.
### Prerequisites
- [Python 3.6](https://www.python.org/downloads/release/python-365/)
- [Pycharm](https://www.jetbrains.com/pycharm/download/#section=windows)
- [Shapely for windows](https://deparkes.co.uk/2015/01/29/install-shapely-on-anaconda/) (Shapely can be installed without Anaconda as well by using 'wheel')
- [Google Chrome](https://www.google.com/chrome/)
For installing shapely using command line do the following:
  - Download shapely.whl from [here](https://www.lfd.uci.edu/~gohlke/pythonlibs/#shapely) (use cp36 for python 3.6)
  - Run the following command in terminal: `<your pythong path> -m pip install <shapely.whl path>`
  
### Running the project
To run the project: 
- Open pycharm
- Choose the correct python interpreter (the one you installed shapely on)
- Install all other packages needed for the project (pycharm will recommend installing packages if they are missing)
- Choose Script path in configurations to be main.py
- Choose Working directory in configurations to be the repository directory
- Mark repo directory in pycharm as "Source Root"
- Run the project in pycharm

If any issues arise please contact me directly

## File hierarchy description
### Root directory
###### FlowHelper.py
In charge of parsing input files from the [input](input/) directory
###### main.py
In charge of running the flows for simulation/waypoint or both depending on definition in MainConfig
###### MainConfig.py
Defines the configurations for the project.
1. waypoint_config - holds all config needed to create `.waypoint` file for the mission planner
2. global_config - holds all the global config needed to load input files from the [input](input/) directory. Also defines which flow we would run and the point format we will use for the simulation and path planning
3. simulation_config - defines all the needed path files for creating the js simulation file and running it on the browser
4. path_planning_algorithm_config - defines variables needed for the path planning algorithm.
    - Grid Size - defines the grid size for the path planning obstacle algorithm. A bigger grid would take longer to calculate and give a more accurate path.
    - Grid Coeff - defines how much we would like to stretch the grid to allow more freedom to the algorithm
    - Relative Dist To Center - defines how close to the center of a shape we would like to get in order to collect the data
5. js_config - defines variables needed for the simulation
###### SimulationGenerator.py
In charge of creating the needed HTML and js files to run the simulation

### Geo directory
###### GeoHelpers.py
Geometrical helper functions for working with shapes and points

### PathPlanning directory
###### Graph.py
Has functions in charge of working with the graph generated from the grid.
###### pathPlanning.py
Has functions with all the path plans created for the experiments. Each algorithm has 2 variations. A heuristic variation which is calculated quickly but gives non optimal path and a brute force method which gives the best possible path.
1. Regular traveling salesman
2. Traveling salesman for points with radii
3. Obstacle traveling salesman:
    - Obstacle path using normal traveling salesman
    - Obstacle path using dynamic traveling salesman which recalculates groups after each travel
    - Obstacle path using optimized dynamic traveling salesman which uses obstacle algorithm when there are obstacles between points and otherwise goes from point to point with a straight line

###### pathPlanningHelpers.py
Helper methods to be used for path planning files

### Waypoing directory
###### file.py
Creates a waypoint file for mission planner based on start point and path points
###### loadData.py
Used to load specific config data for waypoint files

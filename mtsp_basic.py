#----------------------------
#Solving Multiple Travelling Salesman Problem (mTSP) for Multi Robot Task Allocation
#with Google OR Tools
#Basic Version with no additional constraints
#----------------------------

#Importing Constraint Solver from GoogleORTools with Pyton Wraper
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

#Importing additional Python Packages
import math

#Agent Class
#Can be instantiated with different Attributes
class agent():
    def __init__(self, position):
        self.pos = position

#Task Class
#Can be instantiated with different Attributes
class task():
    def __init__(self, position):
        self.pos = position

#Creation of the Data Model for the Solver from the defined Agents and Tasks
def create_data_model(agents, tasks):
    data = {}
    data['distance_matrix'] = [
        	[0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
    ]
    #Creating the Distance Matrix, this is the Adjacence Matrix of the Task Graph
    #The weights (distances) are the cartesian Distances between the Task Positions
    #Alternatively the matrix can be viewed as the Time Matrix (makes the future additional Constraints easier to formulate) which is the same as th
    #distance Matrix for constant speed
    for i in range(len(tasks)):
        for j in range(len(tasks)):
            if i != j:
                data['distance_matrix'][i][j] = round(math.dist(tasks[i].pos, tasks[j].pos))
    data['num_agents'] = len(agents)
    #Defining the Indicies of the Start and End Point of the Agents
    #These are modeled as Dummy Vertices in the Task Graph and correspond to the according lines in the adjacence Matrix
    data['starts'] = [0, 0]
    data['ends'] = [0, 0]
    return data

#Saving the calculated routes in a List
def get_routes(solution, routing, manager):
    routes = []
    for route_nbr in range(routing.vehicles()):
        index = routing.Start(route_nbr)
        route = [manager.IndexToNode(index)]
        while not routing.IsEnd(index):
            index = solution.Value(routing.NextVar(index))
            route.append(manager.IndexToNode(index))
        routes.append(route)
    return routes

def main():
    #Initiating the agents
    agents = [
        agent([-5, -1]),
        agent([3, -1]),
    ]

    #Initiating the tasks
    tasks = [
        task([-8, 6]),
        task([-6, -6]),
        task([-2, 3]),
        task([6, -5]),
        task([8, 5]),
    ]

    #Creating the Data Model
    data = create_data_model(agents, tasks)

    #Creating the Index Manager for the Solver
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), 
                                            data['num_agents'], 
                                            data['starts'], data['ends'])

    #Creating the Routing Model for the Solver
    routing = pywrapcp.RoutingModel(manager)

    #Creating a Distance Callback for the Solver
    #This convertes the internal indicies from the manager to nodes in the graph and returns the corresponding data point
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    #Creating an Transit Callback ID for the solver/model from the distance callback
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    #Setting the Cost of Transit (ArcCost) through the Transit Callback ID and the Distancs Callback to the Distance between Nodes
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    #Creating a new Distance Dimension
    #The solver needs dimensions to keep track of quantities accumulated by the agents through their path
    #The Distance Dimension tracks the current total Distance of each Agent 
    #This is nececary to minimize the biggest individual distance for the agents
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,     #Setting Callback for the Dimension
        0,                          #Setting Slack Variable for the Dimension
        100,                        #Setting total quantity which can be amounted along each route
        True,                       #Weather the value HAS to start at zero or not
        dimension_name)             #Name/Identifier for the Dimension
    
    #Including the Dimension in the Routing Model
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    #Sets the coeffient to make this dimension the dominant factor for the solver
    #100 is a magic number because for only one tracked dimension you just need a "large" coefficient
    #If you are tracking different dimensions the individual coefficients have to be adjusted more carefully
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    #Creating the Search Parameters for the Solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    #Defining the First Solution Strategy (Metaheuristic)
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    #Calling the Solver
    solution = routing.SolveWithParameters(search_parameters)
    #Saving the Routes
    routes = get_routes(solution, routing, manager)
    for i, route in enumerate(routes):
        print('Route', i, route)

if __name__ == '__main__':
    main()

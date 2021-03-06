#----------------------------
#Solving Multiple Travelling Salesman Problem (mTSP) for Multi Robot Task Allocation
#with Google OR Tools
#Basic Version with no additional constraints
#----------------------------
#This was just a verification test for individual cost metrics for different Agents
#----------------------------

#Importing Constraint Solver from GoogleORTools with Pyton Wraper
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

#Importing additional Python Packages
import math

#Agent Class
#Gets Position/Location as Parameter
class agent():
    def __init__(self, position):
        self.pos = position

#Task Class
#Gets Position/Location as Parameter
class task():
    def __init__(self, position):
        self.pos = position

#Vertice Class
#Gets Position/Location as Parameter
class vertex():
    def __init__(self, position):
        self.pos = position

#Creation of the Data Model for the Solver from the defined Agents and Tasks
def create_data_model(agents, tasks, finish):
    #Merging the agents, tasks and finish locations to single vertice list
    #Vertices List has Structure [finish, agents, tasks]
    vertices = [finish]
    for i in range(len(agents)):
        vertices.append(agents[i])
    for j in range(len(tasks)):
        vertices.append(tasks[j])
    
    #Creating empty Distance Matrices
    data = {}
    data['distance_matrix'] = []
    for a in range(len(agents)):
        data['distance_matrix'].append([])
        for i in range(len(vertices)):
            data['distance_matrix'][a].append([])
            for j in range(len(vertices)):
                data['distance_matrix'][a][i].append(0)
    
    #Creating the Distance Matrix, this is the Adjacence Matrix of the Vertices Graph
    #The weights are the cartesian Distances between the Vertice Positions
    #Alternatively the matrix can be viewed as the Time Matrix (makes the future additional Constraints easier to formulate) which is the same as the
    #distance Matrix for constant speed
    
    #Filling the First Matrix with the distances between the Vertices
    for i in range(len(vertices)):
        for j in range(len(vertices)):
            if i != j:
                data['distance_matrix'][0][i][j] = round(math.dist(vertices[i].pos, vertices[j].pos))

    #Creating the second matrix like the first one but with a +1 at every point to create different costs for testing
    for i in range(len(vertices)):
        for j in range(len(vertices)):
            if i != j:
                data['distance_matrix'][1][i][j] = round(math.dist(vertices[i].pos, vertices[j].pos))+1
    
    data['num_agents'] = len(agents)
    
    #Defining the Indicies of the Start and End Point of the Agents
    #These are modeled as Dummy Vertices 0, 1, 2 in the Task Graph and correspond to the according lines in the adjacence Matrix
    data['starts'] = [1, 2]
    data['ends'] = [0, 0]

    return data

#Saving the calculated routes and total distances in a List
def get_routes(solution, routing, manager):
    routes = []
    distances = []
    #Propagating through different Routes/Agents
    for route_nbr in range(routing.vehicles()):
        index = routing.Start(route_nbr)
        route = [manager.IndexToNode(index)]
        route_distance = 0
        #Propagating through the Vertices in an single Route
        while not routing.IsEnd(index):
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route.append(manager.IndexToNode(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, route_nbr)
        routes.append(route)
        distances.append(route_distance)
    return routes, distances

def main():
    #Initiating the agents with Position
    agents = [
        agent([-5, -1]),
        agent([3, -1]),
    ]

    #Initiating the tasks with Position
    tasks = [
        task([-8, 6]),
        task([-6, -6]),
        task([-2, 3]),
        task([6, -5]),
        task([8, 5]),
    ]

    #Initiating the finish vertex with position
    finish = vertex([0, 0])

    #Creating the Data Model
    data = create_data_model(agents, tasks, finish)

    #Creating the Index Manager for the Solver
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix'][0]), 
                                            data['num_agents'], 
                                            data['starts'], data['ends'])

    #Creating the Routing Model for the Solver
    routing = pywrapcp.RoutingModel(manager)

    #Creating Distance Callbacks for the Solver, one for each Agent with the corresponding distance matrix
    #This convertes the internal indicies from the manager to nodes in the graph and returns the corresponding data point
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][0][from_node][to_node]

    def distance_callback1(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][1][from_node][to_node]

    #Creating Transit Callback IDs for the solver/model from the distance callbacks
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    transit_callback_index1 = routing.RegisterTransitCallback(distance_callback1)

    #Setting the Cost of Transit (ArcCost) for each agent through the Transit Callback ID and the Distancs Callback to the Distance between Nodes
    routing.SetArcCostEvaluatorOfVehicle(transit_callback_index, 0)
    routing.SetArcCostEvaluatorOfVehicle(transit_callback_index1, 1)

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
    
    #Setting the coeffient to make this dimension the dominant factor for the solver
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
    
    #Saving the Routes and corresponding Distances
    routes, distances = get_routes(solution, routing, manager)
    for i, route in enumerate(routes):
        print('Route', i, route, 'Distance', distances[i])

if __name__ == '__main__':
    main()

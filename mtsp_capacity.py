#----------------------------
#Solving Multiple Travelling Salesman Problem (mTSP) for Multi Robot Task Allocation
#with Google OR Tools
#Extending the basic version with additional capacity constraints for each agent
#----------------------------

#Importing Constraint Solver from GoogleORTools with Pyton Wraper
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

#Importing additional Python Packages
import math

#Vertice Class
#Instantiated with Position/Location and Demand
class vertex():
    def __init__(self, position, demand):
        self.pos = position
        self.demand = demand

#Agent Class inherits from Vertice
#Instanciated with Position/Location and Capacity, Demand is set to 0
class agent(vertex):
    def __init__(self, position, capacity):
        super().__init__(position, 0)
        self.capacity = capacity

#Task Class inherits from Vertice
#Instanciated with Position/Location and Demand
class task(vertex):
    def __init__(self, position, demand):
        super().__init__(position, demand)

#Creation of the Data Model for the Solver from the defined Agents and Tasks
def create_data_model(agents, tasks, finish):
    #Merging the agents, tasks and finish locations to single vertice list
    #Vertices List has Structure [finish, agents, tasks]
    vertices = [finish]
    for i in range(len(agents)):
        vertices.append(agents[i])
    for j in range(len(tasks)):
        vertices.append(tasks[j])
    
    #Creating empty Distance Matrix
    data = {}
    data['distance_matrix'] = []
    for i in range(len(vertices)):
        data['distance_matrix'].append([])
        for j in range(len(vertices)):
            data['distance_matrix'][i].append(0)
    
    #Creating the Distance Matrix, this is the Adjacence Matrix of the Vertices Graph
    #The weights are the cartesian Distances between the Vertice Positions
    #Alternatively the matrix can be viewed as the Time Matrix (makes the future additional Constraints easier to formulate) which is the same as the
    #distance Matrix for constant speed
    
    #Filling the Matrix with the distances between the Vertices
    for i in range(len(vertices)):
        for j in range(len(vertices)):
            if i != j:
                data['distance_matrix'][i][j] = round(math.dist(vertices[i].pos, vertices[j].pos))
    
    #Defining the number of Agents/Traveling Salesman
    data['num_agents'] = len(agents)
    
    #Defining the Demands for the Tasks
    data['demands'] = []
    for i in range(len(vertices)):
        data['demands'].append(vertices[i].demand)

    #Defining the Capacities of the Agents
    data['capacities'] = []
    for i in range(len(agents)):
        data['capacities'].append(agents[i].capacity)

    #Defining the Indicies of the Start and End Point of the Agents
    #These are modeled as Dummy Vertices 0, 1, 2 in the Task Graph and correspond to the according lines in the adjacence Matrix
    data['starts'] = [1, 2]
    data['ends'] = [0, 0]
    
    return data

#Saving the calculated routes and total distances in a List
def get_routes(data, solution, routing, manager):
    routes = {}
    routes['paths'] = []
    routes['distances'] = []
    routes['loads'] = []
    #Propagating through different Routes/Agents
    for route_nbr in range(routing.vehicles()):
        index = routing.Start(route_nbr)
        route = [manager.IndexToNode(index)]
        route_distance = 0
        route_load = 0
        #Propagating through the Vertices in an single Route
        while not routing.IsEnd(index):
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route.append(manager.IndexToNode(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, route_nbr)
            route_load += data['demands'][manager.IndexToNode(index)]
        routes['paths'].append(route)
        routes['distances'].append(route_distance)
        routes['loads'].append(route_load)
    return routes

def main():
    #Initiating the agents with position and capacity
    agents = [
        agent([-5, -1], 8),
        agent([3, -1], 8),
    ]

    #Initiating the tasks with position and demand
    tasks = [
        task([-8, 6], 2),
        task([-6, -6], 3),
        task([-2, 3], 4),
        task([6, -5], 1),
        task([8, 5], 2),
    ]

    #Initiating the finish position with location and demand
    finish = vertex([0, 0], 0)

    #Creating the Data Model
    data = create_data_model(agents, tasks, finish)

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
    
    #Setting the coeffient to make this dimension the dominant factor for the solver
    #100 is a magic number because for only one tracked dimension you just need a "large" coefficient
    #If you are tracking different dimensions the individual coefficients have to be adjusted more carefully
    
    #NOTE: With additional capacity constraints the solver produces different solutions
    #weather you prioritise Individual Total Distance or not. With the Span this high, 
    #the solver produces the shortest single distance, but with no given specific Span
    #The solver produces another solution with a slightly higher individual total distance
    #but with a smaller overall distance
    #This also only happens when setting different Local Search Metaheiristics, which can
    #make the solver significantly slower (or search infinite) but produce a slightly lower
    #indiividual maximal distance
    
    #distance_dimension.SetGlobalSpanCostCoefficient(100)

    #Creating Demand Callback for the Solver
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    #Creating and an Transit Callback ID for the solver/model from the demand callback
    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

    #Adding Capacity Dimension
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index, 
        0, 
        data['capacities'], 
        True, 
        'Capacity'
    )
    capacity_dimension = routing.GetDimensionOrDie('Capacity')
    

    #Creating the Search Parameters for the Solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    
    #Defining the First Solution Strategy (Metaheuristic)
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    #Calling the Solver
    solution = routing.SolveWithParameters(search_parameters)
    
    #Saving the Routes and corresponding Distances
    routes = get_routes(data, solution, routing, manager)
    for i in range(len(routes['paths'])):
        print('Route', i, routes['paths'][i], '| Distance:', routes['distances'][i], '| Load:', routes['loads'][i])

if __name__ == '__main__':
    main()
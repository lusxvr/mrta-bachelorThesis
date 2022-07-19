#----------------------------
#Solving Multiple Travelling Salesman Problem (mTSP) for Multi Robot Task Allocation
#with Google OR Tools
#Extending the basic version with capacity constraints for each agent with teim window constraints for each task
#----------------------------
#At this complexity level you can start to tune some parameters to customise your solution:
#(-Capacity of each Agent, possible since the last iteration but with more effect with the additional time constraints)
# -Number of Collaborative Tasks
# -Slack Variable of Time Dimension (allowed waiting time at tasks)
#   (if set to zero, agents have to continue instantly, may yield worse results)
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
    def __init__(self, position, demand, collab):
        super().__init__(position, demand)
        self.collab = collab

#Creation of the Data Model for the Solver from the defined Agents and Tasks
def create_data_model(agents, tasks, finish):
    #Merging the agents, tasks and finish locations to single vertice list
    #Vertices List has Structure [finish, agents, tasks]
    vertices = [finish]
    for i in range(len(agents)):
        vertices.append(agents[i])
    for j in range(len(tasks)):
        vertices.append(tasks[j])
    
    #Creating empty (Travel-)Time Matrix
    data = {}
    data['time_matrix'] = []
    for i in range(len(vertices)):
        data['time_matrix'].append([])
        for j in range(len(vertices)):
            data['time_matrix'][i].append(0)
    
    
    #Creating the Time Matrix, this is the Adjacence Matrix of the Vertices Graph
    #The weights are the Travel Times between the Vertice Positions
    #Time is now used to incoroerate Time Constraints for the Tasks
    #The Time and Distance Matrix from previous iterations are identical for constant speed
    
    #Filling the Matrix with the travel times between the Vertices (same as the distance because we assume constant speed)
    for i in range(len(vertices)):
        for j in range(len(vertices)):
            if i != j:
                data['time_matrix'][i][j] = round(math.dist(vertices[i].pos, vertices[j].pos))
    
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

    #Checking for requested Collaborations
    #Currently only collaborations between two agents are possible
    #Collaboration requiring vertex indices are saved as pairs in a list
    data['collabs'] = []
    for i in range((len(vertices)-len(tasks)), len(vertices)):   
        if vertices[i].collab:
            if vertices[i-1].collab == vertices[i].collab:
                data['collabs'].append([i-1, i])
    #Defining the Indicies of the Start and End Point of the Agents
    #These are modeled as Dummy Vertices 0, 1, 2 in the Task Graph and correspond to the according lines in the adjacence Matrix
    data['starts'] = [1, 2]
    data['ends'] = [0, 0]
    
    return data

#Saving the calculated routes and total distances in a List
def get_routes(data, solution, routing, manager, time_dimension):
    routes = {}
    routes['paths'] = []
    routes['distances'] = []
    routes['loads'] = []
    routes['times'] = []
    #Propagating through different Routes/Agents
    for route_nbr in range(routing.vehicles()):
        index = routing.Start(route_nbr)
        route = [manager.IndexToNode(index)]
        route_distance = 0
        route_load = 0
        times = []
        time_val = time_dimension.CumulVar(index)
        times.append([solution.Min(time_val), solution.Max(time_val)])
        #Propagating through the Vertices in an single Route
        while not routing.IsEnd(index):
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route.append(manager.IndexToNode(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, route_nbr)
            route_load += data['demands'][manager.IndexToNode(index)]
            time_val = time_dimension.CumulVar(index)
            times.append([solution.Min(time_val), solution.Max(time_val)])
        routes['paths'].append(route)
        routes['distances'].append(route_distance)
        routes['loads'].append(route_load)
        routes['times'].append(times)
    return routes

def main():
    #Initiating the agents with position and capacity
    agents = [
        agent([-5, -1], 6),
        agent([3, -1], 6),
    ]

    #Initiating the tasks with position, demand, and collaboration
    #Each task pair requiring collaboration has to be marked with individual integers
    #e.g. if task 1 & 2 also require collaboration you have to mark them with e.g. 2 and
    #cannot mark them with 1 since this marker is already used
    #Collaborative Tasks also need to be next to another since this is the way the cost
    #function will implement them and since it is also just easier to check
    tasks = [
        task([-8, 6], 2, 0),
        task([-6, -6], 3, 0),
        task([-2, 3], 4, 0),
        task([6, -5], 1, 0),
        task([8, 5], 1, 1),     #Requires Collaboration (generates lower task)
        task([9, 6], 1, 1),    #'Collaborating' Task (cant be at the same location)
    ]

    #Initiating the finish position with location and demand
    finish = vertex([0, 0], 0)

    #Creating the Data Model
    data = create_data_model(agents, tasks, finish)

    #Creating the Index Manager for the Solver
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), 
                                            data['num_agents'], 
                                            data['starts'], data['ends'])

    #Creating the Routing Model for the Solver
    routing = pywrapcp.RoutingModel(manager)

    #Creating a Travel Time Callback for the Solver
    #This convertes the internal indicies from the manager to nodes in the graph and returns the corresponding data point
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    #Creating an Transit Callback ID for the solver/model from the time callback
    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    #Setting the Cost of Transit (ArcCost) through the Transit Callback ID and the Distancs Callback to the Distance between Nodes
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    #Creating a new Travel Time Dimension
    #The solver needs dimensions to keep track of quantities accumulated by the agents through their path
    #The Time Dimension tracks the current total travel time of each Agent 
    #This is nececary to minimize the biggest individual time for the agents
    dimension_name = 'Time'
    routing.AddDimension(
        transit_callback_index,     #Setting Callback for the Dimension
        10,                         #Setting Slack Variable for the Dimension (Allowed waiting Time)
        100,                        #Setting total quantity which can be amounted along each route
        True,                       #Weather the value HAS to start at zero or not
        dimension_name)             #Name/Identifier for the Dimension
    
    #Including the Dimension in the Routing Model
    time_dimension = routing.GetDimensionOrDie(dimension_name)
    
    #Creating Additional Constraint if there is a collaboration Task
    #Checks if there are requested collabs
    #Adds the constraint that collaborative Tasks have to be executed at the same Time
    if len(data['collabs']):
        for i in range(len(data['collabs'])):
            index1 = manager.NodeToIndex(data['collabs'][i][0])
            index2 = manager.NodeToIndex(data['collabs'][i][1])
            routing.solver().Add(time_dimension.CumulVar(index1) == time_dimension.CumulVar(index2))

    #Setting the coeffient to make this dimension the dominant factor for the solver
    #100 is a magic number because for only one tracked dimension you just need a "large" coefficient
    #If you are tracking different dimensions the individual coefficients have to be adjusted more carefully
    
    #NOTE: With additional capacity constraints the solver produces different solutions
    #weather you prioritise Individual Total Time or not. With the Span this high, 
    #the solver produces the shortest single distance, but with no given specific Span
    #The solver produces another solution with a slightly higher individual total time
    #but with a smaller overall distance
    #This also only happens when setting different Local Search Metaheiristics, which can
    #make the solver significantly slower (or search infinite) but produce a slightly lower
    #individual maximal time
    
    time_dimension.SetGlobalSpanCostCoefficient(100)

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
    
    #Defining the First Solution Strategy & Local Search Metaheuristic
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    #search_parameters.local_search_metaheuristic = (
    #    routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    #search_parameters.time_limit.FromSeconds(1)

    #Calling the Solver
    solution = routing.SolveWithParameters(search_parameters)
    
    #Saving the Routes and corresponding Distances
    if solution:
        routes = get_routes(data, solution, routing, manager, time_dimension)
        for i in range(len(routes['paths'])):
            print('Route', i)
            print(routes['paths'][i], '| Times:', routes['times'][i], '| Distance:', routes['distances'][i], '| Load:', routes['loads'][i])
    else:
        print('No Solution found')
        return 1

if __name__ == '__main__':
    main()
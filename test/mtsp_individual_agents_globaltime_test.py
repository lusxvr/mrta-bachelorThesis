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
import numpy as np

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
    
    #Creating empty Global Time Matrix
    data = {}
    data['global_time_matrix'] = []
    for i in range(len(vertices)):
        data['global_time_matrix'].append([])
        for j in range(len(vertices)):
            data['global_time_matrix'][i].append(0)
    #Creating Empty Cost Matrices
    data['cost_matrix'] = []
    for a in range(len(agents)):
        data['cost_matrix'].append([])
        for i in range(len(vertices)):
            data['cost_matrix'][a].append([])
            for j in range(len(vertices)):
                data['cost_matrix'][a][i].append(0)
    
    #Filling the Matricex, they are all Adjacency Matrices of the Graph
    #The weights are the Travel Times between the Vertice Positions and the costs for each Agent respectively
    
    #Filling the Global Time Matrix with the Travel Times between the Vertices proportional to the distances
    for i in range(len(vertices)):
        for j in range(len(vertices)):
            if i != j:
                data['global_time_matrix'][i][j] = round(math.dist(vertices[i].pos, vertices[j].pos))

    #Filling the cost matrixes for the agents with their individual costs (Dummy Costs in this case, for testing)
    for i in range(len(vertices)):
        for j in range(len(vertices)):
            if i != j:
                data['cost_matrix'][0][i][j] = round(math.dist(vertices[i].pos, vertices[j].pos))+1
    for i in range(len(vertices)):
        for j in range(len(vertices)):
            if i != j:
                data['cost_matrix'][1][i][j] = round(math.dist(vertices[i].pos, vertices[j].pos))-1
    
    data['num_agents'] = len(agents)
    
    #Defining the Indicies of the Start and End Point of the Agents
    #These are modeled as Dummy Vertices 0 (Finish), 1 (Start Agent 0), 2 (Start Agent 1) in the Task Graph and correspond to the according lines in the adjacency Matrix
    data['starts'] = [1, 2]
    data['ends'] = [0, 0]
    
    #print(np.array(data['global_time_matrix']))
    #print(np.array(data['cost_matrix']))

    return data

#Saving the calculated routes, total costs and total times in a List
def get_routes(solution, routing, manager, data):
    routes = []
    costs = []
    times = []
    #Propagating through different Routes/Agents
    for route_nbr in range(routing.vehicles()):
        index = routing.Start(route_nbr)
        route = [manager.IndexToNode(index)]
        route_cost = 0
        route_time = 0
        #Propagating through the Vertices in an single Route
        while not routing.IsEnd(index):
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route.append(manager.IndexToNode(index))
            route_cost += routing.GetArcCostForVehicle(previous_index, index, route_nbr)
            route_time += data['global_time_matrix'][manager.IndexToNode(previous_index)][manager.IndexToNode(index)]
        routes.append(route)
        costs.append(route_cost)
        times.append(route_time)
    return routes, costs, times

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
    manager = pywrapcp.RoutingIndexManager(len(data['global_time_matrix']), 
                                            data['num_agents'], 
                                            data['starts'], data['ends'])

    #Creating the Routing Model for the Solver
    routing = pywrapcp.RoutingModel(manager)

    #Creating Callbacks for the Solver
    #This convertes the internal indicies from the manager to nodes in the graph and returns the corresponding data point
    #First the Global Time Callback, this returns the travel times between vertices
    def global_time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['global_time_matrix'][from_node][to_node]
    #Second the Cost Callbacks for the single Agents, these are the individual Transit Costs for Agents between vertices
    def cost_callback_0(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['cost_matrix'][0][from_node][to_node]

    def cost_callback_1(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['cost_matrix'][1][from_node][to_node]

    #Creating an Transit Callback ID for the solver/model to reference the time callback
    global_transit_callback_index = routing.RegisterTransitCallback(global_time_callback)

    #Creating Transit Callback IDs for the solver/model to reference the cost callbacks
    transit_callback_index_0 = routing.RegisterTransitCallback(cost_callback_0)
    transit_callback_index_1 = routing.RegisterTransitCallback(cost_callback_1)

    #Setting the Cost of Transit (ArcCost) through the Transit Callback ID and the Cost Callbacks to the costs for each agent to be assigned to a task
    routing.SetArcCostEvaluatorOfVehicle(transit_callback_index_0, 0)
    routing.SetArcCostEvaluatorOfVehicle(transit_callback_index_1, 1)

    #Creating a new Time Dimension
    #The solver needs dimensions to keep track of quantities accumulated by the agents through their path
    #The Time Dimension tracks the current total travel Time of each Agent 
    #This is nececary to minimize the biggest individual time for the agents
    dimension_name = 'Time'
    routing.AddDimension(
        global_transit_callback_index,     #Setting Callback for the Dimension
        0,                          #Setting Slack Variable for the Dimension
        100,                        #Setting total quantity which can be amounted along each route
        True,                       #Weather the value HAS to start at zero or not
        dimension_name)             #Name/Identifier for the Dimension
    
    #Including the Dimension in the Routing Model
    time_dimension = routing.GetDimensionOrDie(dimension_name)
    
    #Setting the coeffient to make this dimension the dominant factor for the solver
    #100 is a magic number because for only one tracked dimension you just need a "large" coefficient
    #If you are tracking different dimensions the individual coefficients have to be adjusted more carefully
    time_dimension.SetGlobalSpanCostCoefficient(100)

    #Creating the Search Parameters for the Solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    
    #Defining the First Solution Strategy (Metaheuristic)
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    #Calling the Solver
    solution = routing.SolveWithParameters(search_parameters)
    
    #Saving the Routes and corresponding Costs and Times
    routes, costs, times = get_routes(solution, routing, manager, data)
    for i, route in enumerate(routes):
        print('Route', i, route, 'Cost', costs[i], 'Time', times[i])

if __name__ == '__main__':
    main()
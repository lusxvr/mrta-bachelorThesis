#----------------------------
#Solving Multiple Travelling Salesman Problem (mTSP) for Multi Robot Task Allocation with Google OR Tools
#Incorporating:
#-Individual Agent Capacities
#-Individual Agent Costs
#-Tasks needing collaboration (collaborating Agent have to execute sub-tasks at the same time)
#----------------------------
#You can start to tune some parameters to customise your solution:
# -Capacity of each Agent
# -Number of Collaborative Tasks
# -Cost Function of individual Agents
#----------------------------
#Program returns assignment in json files
#Make sure to have template.json in your directory when running the code
#----------------------------

#---Beginn Import----------------------------------------------------------------------------------
#Importing Constraint Solver from GoogleORTools with Pyton Wraper
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

#Importing additional Python Packages
import math
import json
#---End Import-------------------------------------------------------------------------------------

#---Beginn Classes---------------------------------------------------------------------------------
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
#Instanciated with Position/Location, Demand and Collaboration Index
#Collaboration Index:
#Task with Index 1 collaborates with other Task with Index 1
#Task with Index 2 collaborates with other Task with Index 2 and so on
#Task with Index 0 does not need collaboration
class task(vertex):
    def __init__(self, position, demand, collab):
        super().__init__(position, demand)
        self.collab = collab
#---End Classes------------------------------------------------------------------------------------

#Creation of the Data Model for the Solver from the defined Agents and Tasks-----------------------
def create_data_model(agents, tasks, finish):
    #Merging the agents, tasks and finish locations to single vertice list
    #Vertices List has Structure [finish, agents, tasks]
    vertices = [finish]
    for i in range(len(agents)):
        vertices.append(agents[i])
    for j in range(len(tasks)):
        vertices.append(tasks[j])
    
    #Creating empty Global Travel Time Matrix
    data = {}
    data['global_time_matrix'] = []
    for i in range(len(vertices)):
        data['global_time_matrix'].append([])
        for j in range(len(vertices)):
            data['global_time_matrix'][i].append(0)
    
    #Creating empty Cost Matrix for each Agent
    data['cost_matrix'] = []
    for a in range(len(agents)):
        data['cost_matrix'].append([])
        for i in range(len(vertices)):
            data['cost_matrix'][a].append([])
            for j in range(len(vertices)):
                data['cost_matrix'][a][i].append(0)
    
    #Filling the Matrices, they are all Adjacency Matrices of the same graph
    #The weights are the Travel Times between the Vertice Positions and the costs for each Agent respectively
    #Weights have to be integers for the solver since it is framed as an integer linear programm (either round or scale float values)
    
    #Filling the Global Time Matrix with the Travel Times between the Vertices proportional to the distances
    for i in range(len(vertices)):
        for j in range(len(vertices)):
            if i != j:
                data['global_time_matrix'][i][j] = round(math.dist(vertices[i].pos, vertices[j].pos))

    #Filling the cost matrixes for the agents with their individual costs (TODO: Implementing Individual cost functions)
    for i in range(len(vertices)):
        for j in range(len(vertices)):
            if i != j:
                data['cost_matrix'][0][i][j] = round(math.dist(vertices[i].pos, vertices[j].pos))+1
    for i in range(len(vertices)):
        for j in range(len(vertices)):
            if i != j:
                data['cost_matrix'][1][i][j] = round(math.dist(vertices[i].pos, vertices[j].pos))-1
    
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
    #Only collaborations between two agents are possible
    #If a task requires collaboration, the corresponding vertex indices are saved in a list
    data['collabs'] = []
    for i in range((len(vertices)-len(tasks)), len(vertices)):   
        if vertices[i].collab:
            if vertices[i-1].collab == vertices[i].collab:
                data['collabs'].append([i-1, i])
    #Defining the Indicies of the Start and End Point of the Agents
    #These are modeled as the Dummy Vertices 
    # 0 (Finish for both Agents), 
    # 1 (Start for Agent 0), 
    # 2 (Start for Agent 1)
    # in the Graph
    data['starts'] = [1, 2]
    data['ends'] = [0, 0]
    
    return data

#Saving the calculated route paths, costs, loads, task time windows, movment time and total time in a List
def get_routes(data, solution, routing, manager, time_dimension):
    routes = {}
    routes['paths'] = []
    routes['costs'] = []
    routes['loads'] = []
    routes['mov_times'] = []
    routes['total_times'] = []
    routes['vertice_times'] = []
    routes['transit_times'] = []
    #Propagating through different Routes/Agents
    for route_nbr in range(routing.vehicles()):
        #Starting Index of current route
        index = routing.Start(route_nbr)
        #Initialising path list with the first Vertex (Node)
        path = [manager.IndexToNode(index)]
        #Initialising variables as 0 for each new calculated route
        cost = 0
        load = 0
        mov_time = 0
        total_time = 0
        vertice_times = []
        transit_times = []
        #Getting the cumultative value of the Time Dimension for the current route and vertex
        time_val = time_dimension.CumulVar(index)
        #Appending the Time Window of the first Vertex of the route to the list
        vertice_times.append(solution.Value(time_val))
        #Propagating through the Vertices in an single Route
        while not routing.IsEnd(index):
            #Saving the previous node
            previous_index = index
            #Getting the new node
            index = solution.Value(routing.NextVar(index))
            #Appending the new node to the path
            path.append(manager.IndexToNode(index))
            #Adding the cost for transit between the previous and new node for the respective agent/route to the total cost
            cost += routing.GetArcCostForVehicle(previous_index, index, route_nbr)
            #Adding the demand of the new node to the total load
            load += data['demands'][manager.IndexToNode(index)]
            #Adding the time taken between the previous and new node to the total moving time
            mov_time += data['global_time_matrix'][manager.IndexToNode(previous_index)][manager.IndexToNode(index)]
            #Getting the cum value of the time dimension and adding the time windows for the new node
            time_val = time_dimension.CumulVar(index)
            vertice_times.append(solution.Value(time_val))
            #Saving the current total used time
            total_time = solution.Max(time_val)
            transit_times.append(data['global_time_matrix'][manager.IndexToNode(previous_index)][manager.IndexToNode(index)])
        #Appending the final lists/values to the routes list after finishing the single route
        routes['paths'].append(path)
        routes['costs'].append(cost)
        routes['loads'].append(load)
        routes['mov_times'].append(mov_time)
        routes['total_times'].append(total_time)
        routes['vertice_times'].append(vertice_times)
        routes['transit_times'].append(transit_times)
    #Writes routes dict to a json file
    #json_object = json.dumps(routes, indent = 4)
    #with open('routes.json', 'w') as newfile:
    #    newfile.write(json_object)
    return routes

def write_json(results):
    try:
        f_inst = open('template.json', 'r')
        instructions = json.load(f_inst)
        f_inst.close()
    except :
        print('Could not read template.json file')
        return 1
        
    for i in range(len(results['paths'])):
        for j in range(len(results['paths'][i])):
            identifier = results['paths'][i][j]
            instructions['task'] = 'move_to_pos_%i' %identifier
            instructions['parameters']['skills']['CartPose2TestPose']['skill']['objects']['GoalPose'] = 'TaskPose_%i' %identifier
            json_object = json.dumps(instructions, indent = 4)
            name = str(i) + str(j) + 'task_' + '_' + str(identifier)
            with open('%s.json' %name, 'w') as newfile:
                newfile.write(json_object)

def main(agents, tasks_single, finish):
    #Scanning the task list for collaborative Tasks, splitting and creating the new task list
    collab_num = 0
    for i in range(len(tasks_single)):
        if tasks_single[i].collab:
            collab_num += 1
    tasks = []
    for i in range(len(tasks_single)):
        tasks.append(tasks_single[i])
        if tasks_single[i].collab:
            pos = tasks_single[i].pos
            tasks.append(task([pos[0]+1, pos[1]+1], 0, tasks_single[i].collab))
    
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
    #First the Time Callback, this returns the travel times between vertices
    def time_callback(from_index, to_index):
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
    global_transit_callback_index = routing.RegisterTransitCallback(time_callback)

    #Creating Transit Callback IDs for the solver/model to reference the cost callbacks
    transit_callback_index_0 = routing.RegisterTransitCallback(cost_callback_0)
    transit_callback_index_1 = routing.RegisterTransitCallback(cost_callback_1)

    #Setting the Cost of Transit (ArcCost) through the Transit Callback IDs and the cost callbacks to the costs for each agent to be assigned to a task
    routing.SetArcCostEvaluatorOfVehicle(transit_callback_index_0, 0)
    routing.SetArcCostEvaluatorOfVehicle(transit_callback_index_1, 1)

    #Creating a new Travel Time Dimension
    #The solver needs dimensions to keep track of quantities accumulated by the agents through their path
    #The Time Dimension tracks the current total travel time of each Agent 
    #This is nececary to minimize the biggest individual time for the agents
    dimension_name = 'Time'
    routing.AddDimension(
        global_transit_callback_index,     #Setting Callback for the Dimension
        10,                                #Setting Slack Variable for the Dimension (Allowed waiting time)
        100,                               #Setting total quantity which can be amounted along each route
        True,                              #Weather the value HAS to start at zero or not
        dimension_name)                    #Name/Identifier for the Dimension
    
    #Getting the Dimension Identifier from the routing model
    time_dimension = routing.GetDimensionOrDie(dimension_name)
    
    #Creating Additional Constraint if there is a collaboration Task
    #Checks if there are requested collabs
    #Adds the constraint that collaborative Tasks have to be executed at the same Time
    if len(data['collabs']):
        for i in range(len(data['collabs'])):
            index1 = manager.NodeToIndex(data['collabs'][i][0])
            index2 = manager.NodeToIndex(data['collabs'][i][1])
            routing.solver().Add(time_dimension.CumulVar(index1) == time_dimension.CumulVar(index2))


    #Setting the coeffient to make the time dimension the dominant factor for the solver
    #100 is a magic number because for only one tracked dimension you just need a "large" coefficient
    #If you are tracking different dimensions the individual coefficients have to be adjusted more carefully
    time_dimension.SetGlobalSpanCostCoefficient(100)
    #routing.SetPrimaryConstrainedDimension(dimension_name)
    #NOTE: With additional capacity constraints the solver produces different solutions
    #weather you prioritise Individual Total Time or not. With the Span this high, 
    #the solver produces the shortest single distance, but with no given specific Span
    #The solver produces another solution with a slightly higher individual total time
    #but with a smaller overall distance
    #This also only happens when setting different Local Search Metaheiristics, which can
    #make the solver significantly slower (or search infinite) but produce a slightly lower
    #individual maximal time
    
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
        data['capacities'],                 #Setting the maximum allowed value of the dimension to the maximum capacities of the vehicles
        True, 
        'Capacity'
    )    

    #Creating the Search Parameters for the Solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    
    #Defining the First Solution Strategy & Local Search Metaheuristic
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(1)

    #Calling the Solver with the search parameters
    solution = routing.SolveWithParameters(search_parameters)

    #Saving the Routes and corresponding Data
    if solution:
        routes = get_routes(data, solution, routing, manager, time_dimension)
        write_json(routes)
        #for i in range(len(routes['paths'])):
        #    print('Route', i)
        #    print('Path: ', routes['paths'][i], '| Vertice Times:', routes['vertice_times'][i], '| Transit Times:', routes['transit_times'][i])
        #    print('Cost:', routes['costs'][i], '| Load:', routes['loads'][i], '| Moving Time:', routes['mov_times'][i], '| Total Time:', routes['total_times'][i])
        #    print()
    else:
        print('No Solution found')
        return 1

if __name__ == '__main__':
    #Initiating the agents with position and capacity
    agents = [
        agent([-5, -1], 6),
        agent([3, -1], 6),
    ]

    #Initiating the tasks with position, demand, and collaboration -> task(position, demand, collab)
    #Each task pair requiring collaboration has to be marked with individual integers
    #e.g. if task 1 & 2 also require collaboration you have to mark them with e.g. 2 and
    #cannot mark them with 1 since this marker is already used
    #Collaborative Tasks also need to be next to another since this is the way the cost
    #function will implement them and since it is also just easier to check
    tasks = [
        task([-8, 6], 2, 0),
        task([-6, -6], 3, 0),
        task([-2, 3], 4, 1),    #1 is indicating first needed collaboration
        task([6, -5], 1, 0),
        task([8, 5], 2, 2),     #2 in indicating second needed collaboration
    ]

    #Initiating the finish position with location and demand 0
    finish = vertex([0, 0], 0)

    main(agents, tasks, finish)
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
#Program returns assignment in yaml file but also has the method to return json files
#Make sure to have template.yaml (and template.json) in your directory when running the code
#----------------------------

#---Beginn Import----------------------------------------------------------------------------------
#Importing Constraint Solver from GoogleORTools with Pyton Wraper
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

#Importing additional Python Packages
import math
import json
import yaml
import uuid
import sys
import random
from matplotlib import pyplot as plt
import datetime
#---End Import-------------------------------------------------------------------------------------

#---Beginn Classes---------------------------------------------------------------------------------
#Vertice Class
#Instantiated with Position/Location and Demand
class vertex():
    def __init__(self, id, position, demand_small, demand_large):
        self.id = id
        self.pos = position
        self.demand_small = demand_small
        self.demand_large = demand_large

#Agent Class inherits from Vertice
#Instanciated with Position/Location and Capacity, Demand is set to 0
class agent(vertex):
    def __init__(self, id, position, capacity_small, capacity_large):
        super().__init__(id, position, 0, 0)
        self.capacity_small = capacity_small
        self.capacity_large = capacity_large

#Task Class inherits from Vertice
#Instanciated with Position/Location, Demand and Collaboration Index
#Collaboration Index:
#Task with Index 1 collaborates with other Task with Index 1
#Task with Index 2 collaborates with other Task with Index 2 and so on
#Task with Index 0 does not need collaboration
class task(vertex):
    def __init__(self, id, position, demand_small, demand_large, collab):
        super().__init__(id, position, demand_small, demand_large)
        self.collab = collab
#---End Classes------------------------------------------------------------------------------------

#Creating cost functions for the Agents
#ATM the cost is just the distance between points, with a distortion for each agent so the costs are different
#TODO: Implementing sophisticated cost functions
def cost_fkt_agent_0(vert_1, vert_2):
    return round(math.dist(vert_1.pos, vert_2.pos))+1

def cost_fkt_agent_1(vert_1, vert_2):
    return round(math.dist(vert_1.pos, vert_2.pos))-1

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
    data['IDs']= []
    for i in range(len(vertices)):
        data['IDs'].append(vertices[i].id)

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

    #Filling the cost matrixes for the agents with their individual costs
    for i in range(len(vertices)):
        for j in range(len(vertices)):
            if i != j:
                data['cost_matrix'][0][i][j] = cost_fkt_agent_0(vertices[i], vertices[j])
    for i in range(len(vertices)):
        for j in range(len(vertices)):
            if i != j:
                data['cost_matrix'][1][i][j] = cost_fkt_agent_1(vertices[i], vertices[j])
    
    #Defining the number of Agents/Traveling Salesman
    data['num_agents'] = len(agents)
    
    #Defining the Demands for the Tasks
    data['demands_small'] = []
    for i in range(len(vertices)):
        data['demands_small'].append(vertices[i].demand_small)

    data['demands_large'] = []
    for i in range(len(vertices)):
        data['demands_large'].append(vertices[i].demand_large)

    #Defining the Capacities of the Agents
    data['capacities_small'] = []
    for i in range(len(agents)):
        data['capacities_small'].append(agents[i].capacity_small)

    data['capacities_large'] = []
    for i in range(len(agents)):
        data['capacities_large'].append(agents[i].capacity_large)

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
    routes['loads_small'] = []
    routes['loads_large'] = []
    routes['mov_times'] = []
    routes['total_times'] = []
    routes['vertice_times'] = []
    routes['transit_times'] = []
    #Propagating through different Routes/Agents
    for route_nbr in range(routing.vehicles()):
        #Starting Index of current route
        index = routing.Start(route_nbr)
        #Initialising path list with the first Vertex (Node)
        path = [data['IDs'][manager.IndexToNode(index)]]
        #Initialising variables as 0 for each new calculated route
        cost = 0
        load_small = 0
        load_large = 0
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
            path.append(data['IDs'][manager.IndexToNode(index)])
            #Adding the cost for transit between the previous and new node for the respective agent/route to the total cost
            cost += routing.GetArcCostForVehicle(previous_index, index, route_nbr)
            #Adding the demand of the new node to the total load
            load_small += data['demands_small'][manager.IndexToNode(index)]
            load_large += data['demands_large'][manager.IndexToNode(index)]
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
        routes['loads_small'].append(load_small)
        routes['loads_large'].append(load_large)
        routes['mov_times'].append(mov_time)
        routes['total_times'].append(total_time)
        routes['vertice_times'].append(vertice_times)
        routes['transit_times'].append(transit_times)
    return routes

def write_json(routes):
    try:
        f_inst = open('template.json', 'r')
        instructions = json.load(f_inst)
        f_inst.close()
    except :
        print('Could not read template.json file')
        return 1
        
    for i in range(len(routes['paths'])):
        for j in range(len(routes['paths'][i])):
            identifier = routes['paths'][i][j]
            instructions['task'] = 'move_to_pos_%i' %identifier
            instructions['parameters']['skills']['CartPose2TestPose']['skill']['objects']['GoalPose'] = 'TaskPose_%i' %identifier
            json_object = json.dumps(instructions, indent = 4)
            name = str(i) + str(j) + 'task_' + '_' + str(identifier)
            with open('%s.json' %name, 'w') as newfile:
                newfile.write(json_object)

def write_yaml(routes):
    #Skeletons for the skills
    hold_pose = {
        'Name': 'HoldPose',
        'IP': '192.168.0.000',
        'Time': '0.000', 
        'UID': '00000000-0000-0000-0000-000000000000'
    }

    base_pose = {
        'Name': 'BasePose',
        'IP': '192.168.0.000',
        'PosName': 'TestBase',
        'yzOffset': '0.0000 0.0000', 
        'Compliance': '1000 1000 1000 100 100 100',
        'UID': '00000000-0000-0000-0000-000000000000'
    }

    cart_pose = {
        'Name': 'CartPose',
        'IP': '192.168.0.000',
        'PosName': 'TestBase',
        'velocity': '0.1000 0.5000', 
        'acceleration': '0.5000 1.0000',
        'offset': '0.0000 0.0000 0.0000 0.0000 0.0000 0.0000',
        'finger_width': '-1.0000',
        'finger_speed': '0.0000',
        'Compliance': '1000 1000 1000 100 100 100',
        'UID': '00000000-0000-0000-0000-000000000000'
    }

    cart_pose_travel = {
        'Name': 'CartPose',
        'IP': '192.168.0.000',
        'PosName': 'TravelPose',
        'velocity': '0.1000 0.5000', 
        'acceleration': '0.5000 1.0000',
        'offset': '0.0000 0.0000 0.0000 0.0000 0.0000 0.0000',
        'finger_width': '-1.0000',
        'finger_speed': '0.0000',
        'Compliance': '1000 1000 1000 100 100 100',
        'UID': '00000000-0000-0000-0000-000000000000'
    }

    #Collapsing a Dict to a string so it can be saved in the yaml files
    def collapse_dict_to_string(dict):
        string = ''
        keys = list(dict.keys())
        for x in dict:
            string += dict[x]
            if x != keys[-1]:
                string += ' '
        return string

    #Opening the Template
    with open('C:/Users/luisw/Studium/TUM Maschinenwesen/6. Semester/Bachelorarbeit/Algorithmen/mrta-bachelorThesis/template.yaml') as f:
        
        template = yaml.full_load(f)

    #Creating an empty list for appending the tasks
    template['dual-arm']['MRTA'] = []
    task_list = []

    #Finding the max length of allocated vertices
    max_len = 0
    for path in routes['paths']:
        max_len = max(len(path), max_len)

    #Declaring the running variables
    i = 1
    j = 1
    k = 0
    x = 1
    y = 1

    #While the longest route is not finished
    while k < max_len:
        #Looping over the first path
        if i < len(routes['paths'][0]):
            identifier = str(routes['paths'][0][i])
            if identifier == '0':
                identifier = '00'
            #Configuring the CartPose for Travel
            cart_pose_travel['IP'] = '192.168.1.104'
            cart_pose_travel['UID'] = str(uuid.uuid4())
            #Configuring the BasePose
            base_pose['IP'] = '192.168.1.104'
            base_pose['PosName'] = identifier                   #'PosTask_%s' %
            base_pose['UID'] = str(uuid.uuid4())
            #Configuring the CartPose
            cart_pose['IP'] = '192.168.1.104'
            cart_pose['PosName'] = identifier                   #'PosTask_%s' %
            cart_pose['UID'] = str(uuid.uuid4())
            #Collapsing the Poses to strings
            string_cart_travel = collapse_dict_to_string(cart_pose_travel)
            string_base = collapse_dict_to_string(base_pose)
            string_cart = collapse_dict_to_string(cart_pose)
            #Appending the strings to the file
            task_list.append(string_cart_travel)
            template['dual-arm']['MRTA'].append(string_cart_travel)
            task_list.append(string_base)
            template['dual-arm']['MRTA'].append(string_base)
            task_list.append(string_cart)
            template['dual-arm']['MRTA'].append(string_cart)
            #Checking if there is waiting time between two tasks
            if x < len(routes['paths'][0]):
                wait_time = routes['vertice_times'][0][x] - routes['vertice_times'][0][x-1] - routes['transit_times'][0][x-1]
                #Configuring, collapsing and appending the HoldPose if necessary
                if wait_time:
                    hold_pose['IP'] = '192.168.1.104'
                    hold_pose['Time'] = str(wait_time)
                    hold_pose['UID'] = str(uuid.uuid4())
                    string_hold = collapse_dict_to_string(hold_pose)
                    task_list.append(string_hold)
                    template['dual-arm']['MRTA'].append(string_hold)
                x += 1
            i += 1
        #Looping over the second path
        if j < len(routes['paths'][1]):
            identifier = str(routes['paths'][1][j])
            if identifier == '0':
                identifier = '01'
            #Configuring the CartPose for Travel
            cart_pose_travel['IP'] = '192.168.2.105'
            cart_pose_travel['UID'] = str(uuid.uuid4())
            #Configuring the BasePose
            base_pose['IP'] = '192.168.2.105'
            base_pose['PosName'] = identifier                       #'PosTask_%s' %
            base_pose['UID'] = str(uuid.uuid4())
            #Configuring the CartPose
            cart_pose['IP'] = '192.168.2.105'
            cart_pose['PosName'] = identifier                       #'PosTask_%s' %
            cart_pose['UID'] = str(uuid.uuid4())
            #Collapsing the poses to strings
            string_cart_travel = collapse_dict_to_string(cart_pose_travel)
            string_base = collapse_dict_to_string(base_pose)
            string_cart = collapse_dict_to_string(cart_pose)
            #Appending the strings to the file
            task_list.append(string_cart_travel)
            template['dual-arm']['MRTA'].append(string_cart_travel)
            task_list.append(string_base)
            template['dual-arm']['MRTA'].append(string_base)
            task_list.append(string_cart)
            template['dual-arm']['MRTA'].append(string_cart)
            #Checking if there is waiting time between the tasks
            if y < len(routes['paths'][1]):
                wait_time = routes['vertice_times'][1][y] - routes['vertice_times'][1][y-1] - routes['transit_times'][1][y-1]
                #Configuring, collapsing and appending the HoldPose if necessary
                if wait_time:
                    hold_pose['IP'] = '192.168.2.105'
                    hold_pose['Time'] = str(wait_time)
                    hold_pose['UID'] = str(uuid.uuid4())
                    string_hold = collapse_dict_to_string(hold_pose)
                    task_list.append(string_hold)
                    template['dual-arm']['MRTA'].append(string_hold)
                y += 1
            j += 1
        k += 1

    #Appending the Stop
    task_list.append('Stop')
    template['dual-arm']['MRTA'].append('Stop')

    #Dumping the generated sequence to a yaml file
    with open('skills.yaml', 'w') as file:
        yaml.dump(template, file, sort_keys=False, width=200)

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
            tasks.append(task([pos[0]+1, pos[1]+1, pos[2]+1], 0, 0, tasks_single[i].collab))
    
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
    max_time_capacity = sum(map(sum, data["global_time_matrix"]))
    if max_time_capacity.bit_length() > 63:
        raise ValueError('Desired Path is too long for internal variable data types, split commanded tasks into subgroups and run again')
        
    dimension_name = 'Time'
    routing.AddDimension(
        global_transit_callback_index,                          #Setting Callback for the Dimension
        10000,                                                  #Setting Slack Variable for the Dimension (Allowed waiting time)
        max_time_capacity,                                      #Setting total quantity which can be amounted along each route, is set to sum of entries of golbal time matrix
        True,                                                   #Weather the value HAS to start at zero or not
        dimension_name)                                         #Name/Identifier for the Dimension

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
    routing.SetPrimaryConstrainedDimension(dimension_name)
    #NOTE: With additional capacity constraints the solver produces different solutions
    #weather you prioritise Individual Total Time or not. With the Span this high, 
    #the solver produces the shortest single distance, but with no given specific Span
    #The solver produces another solution with a slightly higher individual total time
    #but with a smaller overall distance
    #This also only happens when setting different Local Search Metaheiristics, which can
    #make the solver significantly slower (or search infinite) but produce a slightly lower
    #individual maximal time
    
    #Creating Small Demand Callback for the Solver
    def small_demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data['demands_small'][from_node]

    #Creating and an Transit Callback ID for the solver/model from the small demand callback
    small_demand_callback_index = routing.RegisterUnaryTransitCallback(small_demand_callback)

    #Adding Capacity Dimension
    routing.AddDimensionWithVehicleCapacity(
        small_demand_callback_index, 
        0, 
        data['capacities_small'],                 #Setting the maximum allowed value of the dimension to the maximum capacities of the vehicles
        True, 
        'Capacity_Small'
    )    

    #Creating Large Demand Callback for the Solver
    def large_demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data['demands_large'][from_node]

    #Creating and an Transit Callback ID for the solver/model from the large demand callback
    large_demand_callback_index = routing.RegisterUnaryTransitCallback(large_demand_callback)

    #Adding Capacity Dimension
    routing.AddDimensionWithVehicleCapacity(
        large_demand_callback_index, 
        0, 
        data['capacities_large'],                 #Setting the maximum allowed value of the dimension to the maximum capacities of the vehicles
        True, 
        'Capacity_Large'
    )    

    #Creating the Search Parameters for the Solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    
    #Defining the First Solution Strategy & Local Search Metaheuristic
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC)
    #search_parameters.local_search_metaheuristic = (
    #    routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    #search_parameters.time_limit.FromSeconds(1)

    #Calling the Solver with the search parameters
    solution = routing.SolveWithParameters(search_parameters)

    calculated_distances = []

    #Saving the Routes and corresponding Data
    if solution:
        routes = get_routes(data, solution, routing, manager, time_dimension)
        
        #write_json(routes)
        #write_yaml(routes)
        
        #for i in range(len(routes['paths'])):
        #    print('Route', i)
        #    print('Path: ', routes['paths'][i], '| Vertice Times:', routes['vertice_times'][i], '| Transit Times:', routes['transit_times'][i])
        #    print('Cost:', routes['costs'][i], '| Load Small:', routes['loads_small'][i] , '| Load Large:', routes['loads_large'][i], '| Moving Time:', routes['mov_times'][i], '| Total Time:', routes['total_times'][i])
        #    print()

        calculated_distances.append((routes['total_times'][0], routes['total_times'][1]))
    else:
        print('No Solution found')
        return 1


    
    #Random Choice allocation
    task_list = []
    num_tasks = len(tasks)
    for i in range(num_tasks):
        task_list.append(i+2)


    tasks_r1 = [0]
    tasks_r2 = [1]

    while len(task_list) != 0:
        choice = random.choice(task_list)
        tasks_r1.append(choice)
        task_list.remove(choice)
        if len(task_list) == 0:
            break
        choice = random.choice(task_list)
        tasks_r2.append(choice)
        task_list.remove(choice)
    
    tasks_r1.append(num_tasks+2)
    tasks_r2.append(num_tasks+2)

    time_r1_rand = 0
    for i in range(len(tasks_r1)-1):
        time_r1_rand += data['global_time_matrix'][tasks_r1[i]][tasks_r1[i+1]]

    time_r2_rand = 0
    for j in range(len(tasks_r2)-1):
        time_r2_rand += data['global_time_matrix'][tasks_r2[j]][tasks_r2[j+1]]


    calculated_distances.append((time_r1_rand, time_r2_rand))
    

    
    #NN-Allocation
    task_list_nn = []
    num_tasks = len(tasks)
    for i in range(num_tasks):
        task_list_nn.append(i+2)

    tasks_r1_nn = [0]
    tasks_r2_nn = [1]

    while len(task_list_nn) != 0:
        potential_tasks_r1 = {}
        for i in range(len(task_list_nn)):
            potential_tasks_r1[task_list_nn[i]] = data['global_time_matrix'][tasks_r1_nn[-1]][task_list_nn[i]]
        choice_nn = min(potential_tasks_r1, key=potential_tasks_r1.get)
        tasks_r1_nn.append(choice_nn)
        task_list_nn.remove(choice_nn)

        if len(task_list_nn) == 1:
            break

        potential_tasks_r2 = {}
        for i in range(len(task_list_nn)):
            potential_tasks_r2[task_list_nn[i]] = data['global_time_matrix'][tasks_r2_nn[-1]][task_list_nn[i]]
        choice_nn = min(potential_tasks_r2, key=potential_tasks_r2.get)
        tasks_r2_nn.append(choice_nn)
        task_list_nn.remove(choice_nn)

    tasks_r1_nn.append(num_tasks+2)
    tasks_r2_nn.append(num_tasks+2)

    time_r1_nn = 0
    for i in range(len(tasks_r1_nn)-1):
        time_r1_nn += data['global_time_matrix'][tasks_r1_nn[i]][tasks_r1_nn[i+1]]
    
    time_r2_nn = 0
    for j in range(len(tasks_r2_nn)-1):
        time_r2_nn += data['global_time_matrix'][tasks_r2_nn[j]][tasks_r2_nn[j+1]]

    calculated_distances.append((time_r1_nn, time_r2_nn))
    

    return calculated_distances

if __name__ == '__main__':
    distances = []

    iterations = 100
    total_num_tasks = 100
    for i in range(iterations):
        print(i)
        agents = [
            agent(0, [random.randint(0, 10000), random.randint(0, 20000), random.randint(0, 20000)], 4, 2),
            agent(1, [random.randint(0, 10000), random.randint(0, 20000), random.randint(0, 20000)],4, 2),
            ]

        tasks = []
        for i in range(total_num_tasks):
            tasks.append(task(i+2, [random.randint(0, 10000), random.randint(0, 20000), random.randint(0, 20000)], 0, 0, 0))

        finish = vertex(len(agents)+len(tasks), [random.randint(0, 10000), random.randint(0, 20000), random.randint(0, 20000)], 0, 0)

        dist = main(agents, tasks, finish)
        distances.append(dist)

    date = datetime.datetime.now()

    json_object = json.dumps(distances, indent = 4)
    name = str(date.strftime('%j_%H_%M_%S')) + '_T' + str(total_num_tasks) + '_I' + str(iterations)
    with open('%s.json' %name, 'w') as newfile:
        newfile.write(json_object)



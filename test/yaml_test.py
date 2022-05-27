import yaml
import uuid
import json

f_routes = open('routes.json', 'r')
routes = json.load(f_routes)
f_routes.close()

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
with open('template.yaml') as f:
    
    template = yaml.full_load(f)

#Creating an empty list for appending the tasks
template['dual-arm']['Test'] = []
task_list = []

#Finding the max length of allocated vertices
max_len = 0
for path in routes['paths']:
    max_len = max(len(path), max_len)

#Declaring the running variables
i = 0
j = 0
k = 0
x = 1
y = 1

#While the longest route is not finished
while k < max_len:
    #Looping over the first path
    if i < len(routes['paths'][0]):
        #Configuring the BasePose
        base_pose['IP'] = '192.168.1.104'
        base_pose['PosName'] = 'PosTask_%i' %routes['paths'][0][i]
        base_pose['UID'] = str(uuid.uuid4())
        #Configuring the CartPose
        cart_pose['IP'] = '192.168.1.104'
        cart_pose['PosName'] = 'PosTask_%i' %routes['paths'][0][i]
        cart_pose['UID'] = str(uuid.uuid4())
        #Collapsing the Poses to strings
        string_base = collapse_dict_to_string(base_pose)
        string_cart = collapse_dict_to_string(cart_pose)
        #Appending the strings to the file
        task_list.append(string_base)
        template['dual-arm']['Test'].append(string_base)
        task_list.append(string_cart)
        template['dual-arm']['Test'].append(string_cart)
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
                template['dual-arm']['Test'].append(string_hold)
            x += 1
        i += 1
    #Looping over the second path
    if j < len(routes['paths'][1]):
        #Configuring the BasePose
        base_pose['IP'] = '192.168.2.105'
        base_pose['PosName'] = 'PosTask_%i' %routes['paths'][1][j]
        base_pose['UID'] = str(uuid.uuid4())
        #Configuring the CartPose
        cart_pose['IP'] = '192.168.2.105'
        cart_pose['PosName'] = 'PosTask_%i' %routes['paths'][1][j]
        cart_pose['UID'] = str(uuid.uuid4())
        #Collapsing the poses to strings
        string_base = collapse_dict_to_string(base_pose)
        string_cart = collapse_dict_to_string(cart_pose)
        #Appending the strings to the file
        task_list.append(string_base)
        template['dual-arm']['Test'].append(string_base)
        task_list.append(string_cart)
        template['dual-arm']['Test'].append(string_cart)
        #Checking if there is waiting time between the tasks
        if y < len(routes['paths'][1]):
            wait_time = routes['vertice_times'][1][y] - routes['vertice_times'][1][y-1] - routes['transit_times'][1][y-1]
            #Configuring, collapsing and appending the HoldPose if necessary
            if wait_time:
                hold_pose['IP'] = '192.168.2.105'
                hold_pose['Time'] = str(wait_time)
                hold_pose['UID'] = str(uuid.uuid4())
                string_hold = collapse_dict_to_string(hold_pose)
            y += 1
        j += 1
    k += 1

#Appending the Stop
task_list.append('Stop')
template['dual-arm']['Test'].append('Stop')

#Dumping the generated sequence to a yaml file
with open('assignment.yaml', 'w') as file:
    yaml.dump(template, file, sort_keys=False, width=200)
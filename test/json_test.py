import json


try:
    f_inst = open('template.json', 'r')
    instructions = json.load(f_inst)
    f_inst.close()
except FileNotFoundError:
    print('could not read template file')

try:
    f_assign = open('assignment.json', 'r')
    results = json.load(f_assign )
    f_assign .close()
except:
    print('could not read results file')

for i in range(len(results['paths'])):
    for j in range(len(results['paths'][i])):
        #continue
        identifier = results['paths'][i][j]
        instructions['task'] = 'move_to_pos_%i' %identifier
        instructions['parameters']['skills']['CartPose2TestPose']['skill']['objects']['GoalPose'] = 'TaskPose_%i' %identifier
        json_object = json.dumps(instructions, indent = 4)
        name = str(i) + str(j) + 'task_' + '_' + str(identifier)
        with open('%s.json' %name, 'w') as newfile:
            newfile.write(json_object)
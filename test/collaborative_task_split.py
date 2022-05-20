import math

class vertex():
    def __init__(self, position, demand):
        self.pos = position
        self.demand = demand

class task(vertex):
    def __init__(self, position, demand, collab):
        super().__init__(position, demand)
        self.collab = collab

def main():
    t = [
        task([-8, 6], 2, 0),
        task([-6, -6], 3, 1),
        task([-2, 3], 4, 2),
        task([6, -5], 1, 0),
        task([8, 5], 1, 0),     #Requires Collaboration (generates lower task)
    ]

    col_num = 0
    for i in range(len(t)):
        if t[i].collab:
            col_num += 1
    tasks = []
    for i in range(len(t)):
        tasks.append(t[i])
        if t[i].collab:
            pos = t[i].pos
            col = t[i].collab
            tasks.append(task([pos[0]+1, pos[1]+1], 0, col))

    for i in range(len(t)):
        print(t[i].pos, t[i].demand, t[i].collab)
    print(col_num)
    for i in range(len(tasks)):
        print(tasks[i].pos, tasks[i].demand, tasks[i].collab)
    

main()
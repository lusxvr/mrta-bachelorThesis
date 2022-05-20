import matplotlib.pyplot as plt

fig, gnt = plt.subplots()

gnt.set_ylim(0, 30)
gnt.set_xlim(0, 50)

gnt.set_xlabel('Seconds since start')
gnt.set_ylabel('Agent')

gnt.set_yticks([10, 20])
gnt.set_yticklabels(['1', '2'])

gnt.grid(True)

#for i in range(len(routes['paths'])):
#    for j in range(len(routes['paths'][i])-1):
#        gnt.broken_barh([(routes['vertice_times'][i][j], routes['transit_times'][i][j])], ((i*10)+10, 5))

gnt.broken_barh([(0, 5)], (5, 10), facecolors =('tab:orange'))

plt.show()
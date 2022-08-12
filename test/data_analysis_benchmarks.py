import json
import numpy as np
from matplotlib import pyplot as plt

try:
    f_data = open('BenchmarkData/210_09_27_36_T100_I100.json', 'r')
    data = json.load(f_data)
    f_data.close()
except :
    print('Could not read template.json file')

print(len(data))

data_r1_mrta = []
data_r2_mrta = []
data_r1_rand = []
data_r2_rand = []
data_r1_nn = []
data_r2_nn = []
for i in range(len(data)):
    data_r1_mrta.append(data[i][0][0])
    data_r2_mrta.append(data[i][0][1])
    data_r1_rand.append(data[i][1][0])
    data_r2_rand.append(data[i][1][1])
    data_r1_nn.append(data[i][2][0])
    data_r2_nn.append(data[i][2][1])

#Average
avg_r1_mrta = sum(data_r1_mrta) / len(data_r1_mrta)
avg_r2_mrta = sum(data_r2_mrta) / len(data_r2_mrta)
avg_r1_rand = sum(data_r1_rand) / len(data_r1_rand)
avg_r2_rand = sum(data_r2_rand) / len(data_r2_rand)
avg_r1_nn = sum(data_r1_nn) / len(data_r1_nn)
avg_r2_nn = sum(data_r2_nn) / len(data_r2_nn)

#Variance
std_r1_mrta = np.std(data_r1_mrta)
std_r2_mrta = np.std(data_r2_mrta)
std_r1_rand = np.std(data_r1_rand)
std_r2_rand = np.std(data_r2_rand)
std_r1_nn = np.std(data_r1_nn)
std_r2_nn = np.std(data_r2_nn)

#Max
max_r1_mrta = max(data_r1_mrta)
max_r2_mrta = max(data_r2_mrta)
max_r1_rand = max(data_r1_rand)
max_r2_rand = max(data_r2_rand)
max_r1_nn = max(data_r1_nn)
max_r2_nn = max(data_r2_nn)

#Mean Differencen between Robots
diff_mrta = np.mean(np.abs(np.array(data_r1_mrta) - np.array(data_r2_mrta)))
diff_rand = np.mean(np.abs(np.array(data_r1_rand) - np.array(data_r2_rand)))
diff_nn = np.mean(np.abs(np.array(data_r1_nn) - np.array(data_r2_nn)))

print('Metrics:')
print('MRTA:')
print('Averages:')
print(avg_r1_mrta)
print(avg_r2_mrta)
print('Standart Deviation:')
print(std_r1_mrta)
print(std_r2_mrta)
print('Max:')
print(max_r1_mrta)
print(max_r2_mrta)
print('Mean Difference between Robots:')
print(diff_mrta)
print('-----------------------------------')
print('Random:')
print('Averages:')
print(str(avg_r1_rand) + ' - ' + str(avg_r1_rand/avg_r1_mrta))
print(str(avg_r2_rand) + ' - ' + str(avg_r2_rand/avg_r2_mrta))
print('Standart Deviation:')
print(str(std_r1_rand) + ' - ' + str(std_r1_rand/std_r1_mrta))
print(str(std_r2_rand) + ' - ' + str(std_r2_rand/std_r2_mrta))
print('Max:')
print(str(max_r1_rand) + ' - ' + str(max_r1_rand/max_r1_mrta))
print(str(max_r2_rand) + ' - ' + str(max_r2_rand/max_r2_mrta))
print('Mean Difference between Robots:')
print(str(diff_rand) + ' - ' + str(diff_rand/diff_mrta))
print('-----------------------------------')
print('NN:')
print('Averages:')
print(str(avg_r1_nn) + ' - ' + str(avg_r1_nn/avg_r1_mrta))
print(str(avg_r2_nn) + ' - ' + str(avg_r2_nn/avg_r2_mrta))
print('Standart Deviation:')
print(str(std_r1_nn) + ' - ' + str(std_r1_nn/std_r1_mrta))
print(str(std_r2_nn) + ' - ' + str(std_r2_nn/std_r2_mrta))
print('Max:')
print(str(max_r1_nn) + ' - ' + str(max_r1_nn/max_r1_mrta))
print(str(max_r2_nn) + ' - ' + str(max_r2_nn/max_r2_mrta))
print('Mean Difference between Robots:')
print(str(diff_nn) + ' - ' + str(diff_nn/diff_mrta))



plt.subplots(figsize=(32,18))
#plt.rcParams.update({'font.size': 30})
plt.plot(data_r1_rand, label='Random R1', color='springgreen')
plt.plot(data_r2_rand, label='Random R2', color='forestgreen')
plt.plot(data_r1_nn, label='NN R1', color='brown')
plt.plot(data_r2_nn, label='NN R2', color='red')
plt.plot(data_r1_mrta, label='MRTA R1', color='blue')
plt.plot(data_r2_mrta, label='MRTA R2', color='dodgerblue')
plt.xlabel("Iterations", fontsize=20)
plt.ylabel("Distance [cm]", fontsize=20)
plt.legend(fontsize=18, markerscale=3)


plt.show()



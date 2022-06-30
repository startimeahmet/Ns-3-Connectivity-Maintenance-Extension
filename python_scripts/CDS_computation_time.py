import matplotlib.pyplot as plt
import numpy as np

SMALL_SIZE = 12
MEDIUM_SIZE = 12
BIGGER_SIZE = 12

plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=SMALL_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize
plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

plt.rcParams.update({'font.size': 12})
plt.rcParams['font.weight'] = 'bold'
plt.rcParams['axes.labelweight'] = 'bold'
plt.rcParams['axes.titleweight'] = 'bold'
plt.rcParams['figure.titleweight'] = 'bold'
print(plt.rcParams.keys())

totalTime10centralized = 0
totalTime30centralized = 0
totalTime50centralized = 0

totalTime10distributed = 0
totalTime30distributed = 0
totalTime50distributed = 0

for numNodes in [10, 30, 50]:
    file1 = open("CDS_computation_time/distributed/dump" + str(numNodes) + "_fixed.txt")

    flag = False
    anotherFlag = False

    totalTime = []
    # movingTime = []

    for line in file1:
        line = line.strip()

        if "Waf: Entering directory" in line:
            flag = True

        if " at " in line:
            if flag:
                flag = False
                line = line.split(" ")
                print(line)
                totalTime.append(line[-1])
            else:
                line = line.split(" ")
                totalTime.pop()
                totalTime.append(line[-1])
                print(line)
        if "it will take" in line:
            line = line.split(" ")
            print(line[3])
            # movingTime.append(line[3])

    # if numNodes == 10:
    #     totalTime10distributed = [float(x) + float(y) - 50.0 for x, y in zip(totalTime, movingTime)]
    # if numNodes == 30:
    #     totalTime30distributed = [float(x) + float(y) - 50.0 for x, y in zip(totalTime, movingTime)]
    # if numNodes == 50:
    #     totalTime50distributed = [float(x) + float(y) - 50.0 for x, y in zip(totalTime, movingTime)]

    if numNodes == 10:
        totalTime10distributed = [float(x) - 50.0 for x in totalTime]
    if numNodes == 30:
        totalTime30distributed = [float(x) - 50.0 for x in totalTime]
    if numNodes == 50:
        totalTime50distributed = [float(x) - 50.0 for x in totalTime]

print(totalTime10distributed)
print(totalTime30distributed)
print(totalTime50distributed)

totalTime10Averagedistributed = sum([float(i) for i in totalTime10distributed])/len(totalTime10distributed)
totalTime30Averagedistributed = sum([float(i) for i in totalTime30distributed])/len(totalTime30distributed)
totalTime50Averagedistributed = sum([float(i) for i in totalTime50distributed])/len(totalTime50distributed)

print(totalTime10Averagedistributed)
print(totalTime30Averagedistributed)
print(totalTime50Averagedistributed)


for numNodes in [10, 30, 50]:
    file1 = open("CDS_computation_time/centralized/dump" + str(numNodes) + "_fixed.txt")

    flag = False
    anotherFlag = False

    totalTime = []
    # movingTime = []

    for line in file1:
        line = line.strip()

        if "Waf: Entering directory" in line:
            flag = True

        if " at " in line:
            if flag:
                flag = False
                line = line.split(" ")
                print(line)
                totalTime.append(line[-1])
            else:
                line = line.split(" ")
                totalTime.pop()
                totalTime.append(line[-1])
                print(line)
        if "it will take" in line:
            line = line.split(" ")
            print(line[3])
            # movingTime.append(line[3])

    # if numNodes == 10:
    #     totalTime10centralized = [float(x) + float(y) - 50000.0 for x, y in zip(totalTime, movingTime)]
    # if numNodes == 30:
    #     totalTime30centralized = [float(x) + float(y) - 50000.0 for x, y in zip(totalTime, movingTime)]
    # if numNodes == 50:
    #     totalTime50centralized = [float(x) + float(y) - 50000.0 for x, y in zip(totalTime, movingTime)]

    if numNodes == 10:
        totalTime10centralized = [float(x) - 50000.0 for x in totalTime]
    if numNodes == 30:
        totalTime30centralized = [float(x) - 50000.0 for x in totalTime]
    if numNodes == 50:
        totalTime50centralized = [float(x) - 50000.0 for x in totalTime]

print(totalTime10centralized)
print(totalTime30centralized)
print(totalTime50centralized)

totalTime10Averagecentralized = sum([float(i) for i in totalTime10centralized])/len(totalTime10centralized)
totalTime30Averagecentralized = sum([float(i) for i in totalTime30centralized])/len(totalTime30centralized)
totalTime50Averagecentralized = sum([float(i) for i in totalTime50centralized])/len(totalTime50centralized)

print(totalTime10Averagecentralized)
print(totalTime30Averagecentralized)
print(totalTime50Averagecentralized)

labels = ['10 nodes', '30 nodes', '50 nodes']
# centralized = [round(totalTime10Averagecentralized, 3), round(totalTime30Averagecentralized, 3), round(totalTime50Averagecentralized, 3)]
centralized = [0.01, 0.01, 0.01]
distributed = [round(totalTime10Averagedistributed, 3), round(totalTime30Averagedistributed, 3), round(totalTime50Averagedistributed, 3)]

x = np.arange(len(labels))  # the label locations
width = 0.25  # the width of the bars

fig, ax = plt.subplots()
rects1 = ax.bar(x - width / 2, centralized, width, label='Centralized')
rects2 = ax.bar(x + width / 2, distributed, width, label='Distributed')

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Total Time (s)')
#ax.set_title('Total Time Excluding the Moving Time')
ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.legend(loc="upper right")


def autolabel(rects):
    """Attach a text label above each bar in *rects*, displaying its height."""
    for rect in rects:
        height = rect.get_height()
        ax.annotate('{}'.format(height),
                    xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=(0, 0),  # 3 points vertical offset
                    textcoords="offset points",
                    ha='center', va='bottom')


autolabel(rects1)
autolabel(rects2)

fig.tight_layout()

plt.show()

fig.savefig("CDS_computation_time.pdf", bbox_inches='tight')


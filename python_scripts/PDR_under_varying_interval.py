import matplotlib.pyplot as plt
import numpy as np

_100msPDR = 0
_50msPDR = 0
_25msPDR = 0

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

for interval in [25, 50, 100]:
    Rx = 0
    Tx = 0
    ETED = 0
    counter = 0
    file1 = open("PDR_under_varying_interval/" + str(interval) + "ms.txt")

    for line in file1:
        line = line.strip()

        if "Rx Packets" in line:
            line = line.replace(" ", "")
            line = line.split("=")
            counter += 1
            Rx += int(line[1])

        if "Tx Packets" in line:
            line = line.replace(" ", "")
            line = line.split("=")

            Tx += int(line[1])

        if "+" in line:
            line = line.replace("+", "")
            line = line.replace("ns", "")
            ETED += float(line)/pow(10, 9)

    print("Rx: " + str(Rx))
    print("Tx: " + str(Tx))
    print("PDR: " + str(Rx/Tx))
    print(counter)
    print("Delay: " + str(1000 * (ETED / (counter))) + " ms")

    if interval == 100:
        _100msPDR = Rx/Tx
        _10nodescentralizedETED = 1000*(ETED/counter)
    elif interval == 50:
        _50msPDR = Rx/Tx
        _30nodescentralizedETED = 1000*(ETED/counter)
    else:
        _25msPDR = Rx/Tx
        _50nodescentralizedETED = 1000*(ETED/counter)


labels = ['100 ms', '50 ms', '25 ms']
centralized = [round(_100msPDR, 3), round(_50msPDR, 3), round(_25msPDR, 3)]
# distributed = [round(_10nodesdistributedETED, 3), round(_30nodesdistributedETED, 3), round(_50nodesdistributedETED, 3)]

x = np.arange(len(labels))  # the label locations
width = 0.35  # the width of the bars

fig, ax = plt.subplots()
rects1 = ax.bar(x, centralized, width, label='Distributed & 10 nodes')
# rects2 = ax.bar(x + width / 2, distributed, width, label='Distributed')

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('PDR')
#ax.set_title('PDR Comparison for Different Beacon-like Intervals')
ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.legend(loc="lower left")


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
# autolabel(rects2)

fig.tight_layout()

plt.show()

fig.savefig("PDR_under_varying_interval.pdf", bbox_inches='tight')

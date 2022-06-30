import matplotlib.pyplot as plt
import numpy as np

# font = {'family' : 'normal',
#         'weight' : 'bold',
#         'size'   : 12}
# #
# plt.rc('font', **font)

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

labels = ['100 ms', '50 ms', '25 ms']
# centralized = [0.01, 0.01, 0.01]
distributed = [1.239, 0.618, 0.308]

x = np.arange(len(labels))  # the label locations
width = 0.25  # the width of the bars

fig, ax = plt.subplots()
# rects1 = ax.bar(x - width / 2, centralized, width, label='Centralized')
rects2 = ax.bar(x, distributed, width, label='Distributed & 30 nodes')

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Total Time (s)')
#ax.set_xlabel('Beacon-like packet interval (ms)')
#ax.set_title('CDS Calculation Time with Respect to Beacon-Like Interval')
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


# autolabel(rects1)
autolabel(rects2)

fig.tight_layout()

plt.show()

fig.savefig("CDS_time_under_varying_interval.pdf", bbox_inches='tight')


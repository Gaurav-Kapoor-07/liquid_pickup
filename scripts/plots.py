
# Importing the matplotlib.pyplot
import matplotlib.pyplot as plt
import csv
import numpy as np




# Declaring a figure "gnt"
fontsize=30
plt.rcParams.update({'font.size': fontsize, 'axes.titlesize':fontsize})
plt.rc('font', size=fontsize)          # controls default text sizes
plt.rc('axes', titlesize=fontsize)     # fontsize of the axes title
plt.rc('axes', labelsize=fontsize)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=fontsize)    # fontsize of the tick labels
plt.rc('ytick', labelsize=fontsize)    # fontsize of the tick labels
plt.rc('legend', fontsize=fontsize)    # legend fontsize
plt.rc('figure', titlesize=fontsize) 
fig, gnt = plt.subplots(figsize=[20,7])
plt.subplots_adjust(left=0.06, bottom=.3, right=.96, top=.95, wspace=10, hspace=0)

# Setting Y-axis limits
gnt.set_ylim(0, 50)
 
# Setting X-axis limits
gnt.set_xlim(0, 1600)
 
# Setting labels for x-axis and y-axis
gnt.set_xlabel('Time [s]')
gnt.set_ylabel('Tasks')
 
# Setting ticks on y-axis
gnt.set_yticks([15, 25, 35])
# Labelling tickes of y-axis
gnt.set_yticklabels(['T1', 'T2', 'T3'])
 
# Setting graph attribute
gnt.grid(axis='x')
gnt.set_axisbelow(True)
 
# Declaring a bar in schedule

 
# Declaring multiple bars in at same level and same width
file = open("move_base.csv")
data = []
dur_movebase = 0
csvreader = csv.reader(file, delimiter=',')
for i in range(3):
    next(csvreader)
for row in csvreader:
    dur = float(row[2])-float(row[1])
    data.append((float(row[1]), dur))
    dur_movebase = dur_movebase + dur
file.close()  
gnt.broken_barh(data, (10, 10), facecolors ='tab:blue')

file = open("move_group.csv")
data = []
dur_movegroup = 0
csvreader = csv.reader(file, delimiter=',')
for i in range(3):
    next(csvreader)
for row in csvreader:
    dur = float(row[2])-float(row[1])
    data.append((float(row[1]), dur))
    dur_movegroup = dur_movegroup + dur
file.close()  
gnt.broken_barh(data,(20,10),facecolors =([.7,0.2,0.2]))

file = open("image_processing.csv")
data = []
dur_img = 0
csvreader = csv.reader(file, delimiter=',')
for i in range(3):
    next(csvreader)
for row in csvreader:
    dur = float(row[2])-float(row[1])
    data.append((float(row[1]), dur))
    dur_img = dur_img + dur
file.close() 

gnt.broken_barh(data, (30, 10), facecolors =('tab:orange'))
gnt.legend(['T1: move_base', 'T2: MoveGroup','T3: Perception'], bbox_to_anchor=(0.5, -0.2), ncol=3,loc='upper center')
plt.savefig("waterfall.png",dpi=500)



def pietext(task, duration, alldurations):
    percent = (duration/np.sum(alldurations)*100)
    return "{:s}\n{:.1f} s ({:.1f} %)".format(task, duration, percent)

fontsize = 18
plt.rcParams.update({'font.size': fontsize, 'axes.titlesize':fontsize})
plt.rc('font', size=fontsize)          # controls default text sizes
plt.rc('axes', titlesize=fontsize)     # fontsize of the axes title
plt.rc('axes', labelsize=fontsize)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=fontsize)    # fontsize of the tick labels
plt.rc('ytick', labelsize=fontsize)    # fontsize of the tick labels
plt.rc('legend', fontsize=fontsize)    # legend fontsize
plt.rc('figure', titlesize=fontsize) 

fig2, pie = plt.subplots(figsize=[10,6])
#plt.subplots_adjust(left=.1, bottom=.0, right=1, top=1, wspace=10, hspace=0)
durations = [dur_movebase, dur_movegroup, dur_img]
wedges, texts  = pie.pie(durations, colors=['tab:blue', [.7,0.2,0.2], 'tab:orange'], wedgeprops=dict(width=0.5))
bbox_props = dict(boxstyle="square,pad=0.3", fc="w", ec="w", lw=0.72)
kw = dict(arrowprops=dict(arrowstyle="-"),
          bbox=bbox_props, zorder=0, va="center")
tasks=['T1: move_base', 'T2: MoveGroup','T3: Perception']
for i, p in enumerate(wedges):
    ang = (p.theta2 - p.theta1)/2. + p.theta1
    y = np.sin(np.deg2rad(ang))
    x = np.cos(np.deg2rad(ang))
    horizontalalignment = {-1: "right", 1: "left"}[int(np.sign(x))]
    connectionstyle = "angle,angleA=0,angleB={}".format(ang)
    kw["arrowprops"].update({"connectionstyle": connectionstyle})
    pie.annotate(pietext(tasks[i],durations[i],durations), xy=(x, y), xytext=(1.35*np.sign(x), 1.4*y),
                horizontalalignment=horizontalalignment, **kw)
plt.savefig("pie.png",dpi=500)


fontsize=30
plt.rcParams.update({'font.size': fontsize, 'axes.titlesize':fontsize})
plt.rc('font', size=fontsize)          # controls default text sizes
plt.rc('axes', titlesize=fontsize)     # fontsize of the axes title
plt.rc('axes', labelsize=fontsize)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=fontsize)    # fontsize of the tick labels
plt.rc('ytick', labelsize=fontsize)    # fontsize of the tick labels
plt.rc('legend', fontsize=fontsize)    # legend fontsize
plt.rc('figure', titlesize=fontsize) 

fig, group = plt.subplots(figsize=[20,7])
plt.subplots_adjust(left=0.08, bottom=.35, right=.96, top=.95, wspace=10, hspace=0)
file = open("move_group_split.csv")
data = np.array([np.array([(0.0,0.0)]),np.array([(0.0,0.0)]),np.array([(0.0,0.0)]),np.array([(0.0,0.0)]),np.array([(0.0,0.0)]),np.array([(0.0,0.0)]),np.array([(0.0,0.0)])])
drop = []
grasp = []
scan = []
post = []
pre = []
validate = []
dur_img = 0
csvreader = csv.reader(file, delimiter=',')
for i in range(3):
    next(csvreader)
for row in csvreader:
    idx = 0
    for k in [1,9,13,17,21,25]:
        if(row[k]):
            duration = float(row[k+1]) - float(row[k])
            if(idx==0):
                drop.append((float(row[k]), duration))
            if(idx==1):
                grasp.append((float(row[k]), duration))
            if(idx==2):
                scan.append((float(row[k]), duration))
            if(idx==3):
                post.append((float(row[k]), duration))
            if(idx==4):
                pre.append((float(row[k]), duration))
            if(idx==5):
                validate.append((float(row[k]), duration))
        idx = idx+1   
 
file.close() 
group.broken_barh(scan,(0,5),facecolors =[.5,0,0])
group.broken_barh(validate,(5,5),facecolors =[.6,0.1,0.1])
group.broken_barh(pre,(10,5),facecolors =[.7,0.2,0.2])
group.broken_barh(grasp,(15,5),facecolors =[.8,0.3,0.3])
group.broken_barh(post,(20,5),facecolors =[.9,0.4,0.4])
group.broken_barh(drop,(25,5),facecolors =[1,0.5,0.5])
group.grid(axis='x')
group.set_ylim(0, 30)
group.set_xlim(0,1600)
group.set_xlabel('Time [s]')
group.set_ylabel('Tasks')
group.set_yticks([2.5,7.5,12.5,17.5,22.5,27.5])
group.set_yticklabels(['T2.1','T2.2','T2.3','T2.4','T2.5','T2.6'])
group.legend(['T2.1: Scan', 'T2.2: Validate','T2.3: Pregrasp','T2.4: Grasp','T2.5: Postgrasp','T2.6: Drop'],loc='upper center', bbox_to_anchor=(0.5, -0.2), ncol=3)
#group.legend(['T2.1: Scan', 'T2.2: Validate','T2.3: Pregrasp','T2.4: Grasp','T2.5: Postgrasp','T2.6: Drop'], ncol=3,loc='upper center')
def mapper(a):
    return a[1]
print(np.array(list(map(mapper,drop))).sum())
plt.savefig("group.png",dpi=500)
groupduration = []
groupduration.append(np.array(list(map(mapper,scan))).sum())
groupduration.append(np.array(list(map(mapper,validate))).sum())
groupduration.append(np.array(list(map(mapper,pre))).sum())
groupduration.append(np.array(list(map(mapper,grasp))).sum())
groupduration.append(np.array(list(map(mapper,post))).sum())
groupduration.append(np.array(list(map(mapper,drop))).sum())


fontsize=18
plt.rcParams.update({'font.size': fontsize, 'axes.titlesize':fontsize})
plt.rc('font', size=fontsize)          # controls default text sizes
plt.rc('axes', titlesize=fontsize)     # fontsize of the axes title
plt.rc('axes', labelsize=fontsize)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=fontsize)    # fontsize of the tick labels
plt.rc('ytick', labelsize=fontsize)    # fontsize of the tick labels
plt.rc('legend', fontsize=fontsize)    # legend fontsize
plt.rc('figure', titlesize=fontsize) 
fig4, pie_group = plt.subplots(figsize=[10,6])
#plt.subplots_adjust(left=.1, bottom=.0, right=1, top=1, wspace=10, hspace=0)
durations = [dur_movebase, dur_movegroup, dur_img]
wedges, texts  = pie_group.pie(groupduration,startangle=90, colors=[[.5,0,0], [.6,0.1,0.1], [.7,0.2,0.2],[.8,0.3,0.3],[.9,0.4,0.4],[1,0.5,0.5]], wedgeprops=dict(width=0.5))
bbox_props = dict(boxstyle="square,pad=0.3", fc="w", ec="w", lw=0.72)
kw = dict(arrowprops=dict(arrowstyle="-"),
          bbox=bbox_props, zorder=0, va="center")
tasks=['T2.1: Scan', 'T2.2: Validate','T2.3: Pregrasp','T2.4: Grasp','T2.5: Postgrasp','T2.6: Drop']
for i, p in enumerate(wedges):
    ang = (p.theta2 - p.theta1)/2. + p.theta1
    y = np.sin(np.deg2rad(ang))
    x = np.cos(np.deg2rad(ang))
    horizontalalignment = {-1: "right", 1: "left"}[int(np.sign(x))]
    connectionstyle = "angle,angleA=0,angleB={}".format(ang)
    kw["arrowprops"].update({"connectionstyle": connectionstyle})
    pie_group.annotate(pietext(tasks[i],groupduration[i],groupduration), xy=(x, y), xytext=(1.35*np.sign(x), 1.4*y),
                horizontalalignment=horizontalalignment, **kw)
plt.savefig("pie_group.png",dpi=500)
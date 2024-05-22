from turtle import fillcolor
import matplotlib.pyplot as plt
import csv
import numpy as np
from requests import patch
file = open("loc_scan.csv")
x1 = []
y1 = []
z1 = []
r1 = []
csvreader = csv.reader(file, delimiter=',')
for row in csvreader:
    x1.append(float(row[0]))
    y1.append(float(row[1]))
    z1.append(float(row[2]))
    r1.append(float(row[3])*2)
file.close()  

file = open("loc_validate.csv")
x2 = []
y2 = []
z2 = []
r2 = []
csvreader = csv.reader(file, delimiter=',')
for row in csvreader:
    x2.append(float(row[0]))
    y2.append(float(row[1]))
    z2.append(float(row[2]))
    r2.append(float(row[3])*2)
file.close() 

fontsize=20
plt.rcParams.update({'font.size': fontsize, 'axes.titlesize':fontsize})
plt.rc('font', size=fontsize)          # controls default text sizes
plt.rc('axes', titlesize=fontsize)     # fontsize of the axes title
plt.rc('axes', labelsize=fontsize)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=fontsize)    # fontsize of the tick labels
plt.rc('ytick', labelsize=fontsize)    # fontsize of the tick labels
plt.rc('legend', fontsize=fontsize)    # legend fontsize
plt.rc('figure', titlesize=fontsize) 

fig,ax = plt.subplots(figsize=[20,4])
plt.subplots_adjust(left=0.12, bottom=.2, right=.96, top=.95, wspace=10, hspace=0)
ax.plot([0.08, 0.08], [0,3],'-.', color='r')
ax.boxplot([r1,r2], vert=False,widths=(.8,.8),patch_artist=True, boxprops=dict(facecolor='w', color='k'),)
ax.set_yticks([1,2])
ax.set_yticklabels(['LookForObject','Validation'])
ax.set_xlabel("Diameter [m]")
#plt.show()

""" fig2,(hist1, hist2) = plt.subplots(1,2, sharey=True, sharex=False)
hist1.hist(r1)
hist2.hist(r2)
plt.show() """

fig, (ax1,ax2,ax3) = plt.subplots(1,3, sharey=True, figsize=[15,4]) # note we must use plt.subplots, not plt.subplot
# (or if you have an existing figure)
# fig = plt.gcf()
# ax = fig.gca()

def getyticks(middle):
    return [ middle-0.05, middle, middle +0.05]


y_soll = np.full((25,1),-2.1)
x_soll = np.arange(-3,2.1,5/24)

for i in range(25):
    circle1 = plt.Circle((x_soll[i],y_soll[i]), 0.04, color='r', fill=False)
    circle2 = plt.Circle((x_soll[i],y_soll[i]), 0.04, color='r', fill=False)
    circle3 = plt.Circle((x_soll[i],y_soll[i]), 0.04, color='r', fill=False)
    circle4 = plt.Circle((x_soll[i],y_soll[i]), 0.04, color='r', fill=False)
    ax1.add_patch(circle1)
    ax2.add_patch(circle2)
    ax3.add_patch(circle3)
file = open("loc_scan.csv")
csvreader = csv.reader(file, delimiter=',')
x_scan = []
y_scan = []
for row in csvreader:
    """ x_scan.append(float(row[0]))
    y_scan.append(float(row[1])) """
    circle1 = plt.Circle((float(row[0]),float(row[1])),float(row[3]), color="b", fill=False)
    circle2 = plt.Circle((float(row[0]),float(row[1])),float(row[3]), color="b", fill=False)
    circle3 = plt.Circle((float(row[0]),float(row[1])),float(row[3]), color="b", fill=False)
    circle4 = plt.Circle((float(row[0]),float(row[1])),float(row[3]), color="b", fill=False)
    ax1.add_patch(circle1)
    ax2.add_patch(circle2)
    ax3.add_patch(circle3)
file.close()  
file = open("loc_validate.csv")
x_val=[]
y_val=[]
csvreader = csv.reader(file, delimiter=',')
for row in csvreader:
    """  x_val.append(float(row[0]))
    y_val.append(float(row[1])) """
    circle1 = plt.Circle((float(row[0]),float(row[1])),float(row[3]), color="orange", fill=False)
    circle2 = plt.Circle((float(row[0]),float(row[1])),float(row[3]), color="orange", fill=False)
    circle3 = plt.Circle((float(row[0]),float(row[1])),float(row[3]), color="orange", fill=False)
    circle4 = plt.Circle((float(row[0]),float(row[1])),float(row[3]), color="orange", fill=False)
    ax1.add_patch(circle1)
    ax2.add_patch(circle2)
    ax3.add_patch(circle3)
file.close()  

xlim1=-0.292
xlim2=0.332
xlim3=1.583
ax1.set_xlim([xlim1-0.1,xlim1+0.1])
ax1.set_ylim([-2.2,-2])
ax1.set_xticks(getyticks(xlim1))
ax1.set_xticklabels(labels=[-0.05,0,0.05])
ax1.set_aspect('equal')
ax1.set_title('Situation 1')
# ax1.scatter(x_soll, y_soll, color="red")
ax1.scatter(x_scan,y_scan, color="b", marker="x")
ax1.scatter(x_val,y_val, color="orange", marker="+")
ax1.grid(True)
ax1.set_xlabel("Deviation x [m]")
ax1.set_ylabel("Deviation y [m]")

ax2.set_xlim([xlim2-0.1,xlim2+0.1])
#ax2.set_ylim([-2.2,-2])
print(getyticks(xlim2))
ax2.set_xticks(getyticks(xlim2))
ax2.set_xticklabels(labels=[-0.05,0,0.05])
ax2.set_aspect('equal')
# ax2.scatter(x_soll, y_soll, color="red")
ax2.scatter(x_scan,y_scan, color="b", marker="x")
ax2.scatter(x_val,y_val, color="orange", marker="+")
ax2.grid(True)
ax2.set_xlabel("Deviation x [m]")
ax2.set_title('Situation 2')

ax3.set_xlim([xlim3-0.1,xlim3+0.1])
ax3.set_ylim([-2.2,-2])
ax3.set_aspect('equal')
ax3.set_xticks(getyticks(xlim3))
ax3.set_xticklabels(labels=[-0.05,0,0.05])
ax3.set_yticks([-2.15,-2.1,-2.05])
ax3.set_yticklabels(labels=[-0.05,0,0.05])
# ax3.scatter(x_soll, y_soll, color="red")
ax3.scatter(x_scan,y_scan, color="b", marker="x")
ax3.scatter(x_val,y_val, color="orange", marker="+")
ax3.grid(True)
ax3.set_xlabel("Deviation x [m]")
ax3.set_title('Situation 3')


figz,axz = plt.subplots(figsize=[20,4])
plt.subplots_adjust(left=0.12, bottom=.2, right=.96, top=.95, wspace=10, hspace=0)
# axz.plot([0.08, 0.08], [0,3],'-.', color='r')
axz.boxplot([z1,z2], vert=False,widths=(.8,.8),patch_artist=True, boxprops=dict(facecolor='w', color='k'),)
axz.set_yticks([1,2])
axz.set_yticklabels(['LookForObject','Validation'])
axz.set_xlabel("Height (z) [m]")


plt.show()
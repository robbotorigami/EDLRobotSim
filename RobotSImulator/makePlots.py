import matplotlib.pyplot as plt
import numpy
import csv

xvaluesd = []
yvaluesd = []

xvaluesr = []
yvaluesr = []

xvaluesrm = []
yvaluesrm = []


with open('dubinspath.csv', 'rb') as csvfile:
    dubinReader = csv.reader(csvfile, delimiter=',')
    for row in dubinReader:
        xvaluesd.append(row[0])
        yvaluesd.append(row[1])

with open('robotspath.csv', 'rb') as csvfile:
    robotReader = csv.reader(csvfile, delimiter=',')
    for row in robotReader:
        xvaluesr.append(row[1])
        yvaluesr.append(row[2])

with open('robotsmodel.csv', 'rb') as csvfile:
    robotReader = csv.reader(csvfile, delimiter=',')
    for row in robotReader:
        xvaluesrm.append(row[0])
        yvaluesrm.append(row[1])
        
#plt.figure()
#plt.plot(xvaluesrm, yvaluesrm, 'black', linewidth = 8.0)
#plt.plot(xvaluesr, yvaluesr,'red', linewidth = 1.0)
#plt.grid(True)

plt.figure()
plt.plot(xvaluesd, yvaluesd)
plt.plot(xvaluesr, yvaluesr,'red', linewidth = 1.0)

plt.axes().set_aspect('equal', 'datalim')
plt.show()

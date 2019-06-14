#!/usr/bin/env python

# import necessary libraries
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as plt3d
from mpl_toolkits.mplot3d import Axes3D
import time
import io
import math


class DrawCableBot:
    def __init__(self):
        # set object variables
        self.length_list = [0, 0, 0, 0]
        self.length = 0.98
        self.width = 0.98
        self.heighth = 0.8
        self.posX = 0.49
        self.posY = 0.49
        self.posZ = 0.075
        # Numbers for the frame of the robot
        self.XYZ = np.array([[0, 0, 0],
                [self.length, 0, 0],
                [0, self.width, 0],
                [self.length, self.width, 0],
                [0, 0, self.heighth],
                [self.length, 0, self.heighth],
                [self.length, self.width, self.heighth],
                [0, self.width, self.heighth]])
        self.cables=[]

    def initializaton(self):
        # initialization

        self.fig = plt.figure()
        self.fig.set_size_inches(10, 10)
        self.ax = self.fig.add_subplot(111, projection='3d', aspect='equal')
        self.ax.view_init(azim=30)

    def drawNodes(self):
        # plot the nodes
        for x, y, z in self.XYZ:
            self.ax.scatter(x, y, z, color='black', marker='s')

    def drawLines(self):
        # plot the lines
        # draw rectangular
        lineOrder = [(0, 1), (1, 3), (3, 2), (2, 0),
                     (4, 5), (5, 6), (6, 7), (7, 4),
                     (0, 4), (1, 5), (2, 7), (3, 6)]
        for (p1, p2) in lineOrder:
            xs = (self.XYZ[p1][0], self.XYZ[p2][0])
            ys = (self.XYZ[p1][1], self.XYZ[p2][1])
            zs = (self.XYZ[p1][2], self.XYZ[p2][2])
            line = plt3d.art3d.Line3D(xs, ys, zs)
            self.ax.add_line(line)
    # draw goal is the function called from the front end to calculate cable lengths and update the visualization
    def drawGoal(self,delta=0.0):
        # draw cable and end effector
        self.ax.scatter(self.posX, self.posY, self.posZ, color='red', marker='s')
        NoCable=False
        if(len(self.cables)==0):
            NoCable=True
        else:
            for line in self.cables:
                line.remove()
                pass
        self.cables=[1,1,1,1]
        cableIndex=0
        for xyz in self.XYZ[4:8]:
            if cableIndex == 0:
                xs = (xyz[0], self.posX+delta)
                ys = (xyz[1], self.posY+delta)
            elif cableIndex == 1:
                xs = (xyz[0], self.posX+delta)
                ys = (xyz[1], self.posY+delta)
            elif cableIndex == 2:
                xs = (xyz[0], self.posX+delta)
                ys = (xyz[1], self.posY+delta)
            elif cableIndex == 3:
                xs = (xyz[0], self.posX+delta)
                ys = (xyz[1], self.posY+delta)
            zs = (xyz[2], self.posZ)
            lengthcal=round(lengthCal(xs,ys,zs,cableIndex),3)

            self.length_list[cableIndex]= lengthcal
            self.temporary = self.length_list

            line = plt3d.art3d.Line3D(xs, ys, zs)
            self.cables[cableIndex]=line
            self.ax.add_line(self.cables[cableIndex])
            cableIndex=cableIndex+1

        # clean any points tha are older than 10 steps ago
        # This keeps matplot lib from causing memory leaks
        if (len(self.ax.collections)>10):
            del self.ax.collections[0]

        return self.length_list

    # Set position of end effector
    def setPos(self,x,y,z):
        self.posX=x
        self.posY=y
        self.posZ=z

# Function to calculate cable lengths
def lengthCal(xs,ys,zs,cableIndex):
    cablelength=math.sqrt((xs[0]-xs[1])**2+(ys[0]-ys[1])**2+(zs[0]-zs[1])**2)
    print('Length of Cable',cableIndex,'is:', cablelength)
    return cablelength


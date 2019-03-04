#!/usr/bin/env python3

#Author: Zhiwei Luo

from map import Map
from mouse import Micromouse
from strategy import StrategyTestMultiDFS
from controller import COREController
from socket import *


mazeMap = Map(16, 16)
mazeMap.readFromFile('/home/parallels/Micromouse/mazes/2012japan-ef.txt') # load map
micromouse = Micromouse(mazeMap)
index = gethostname()[1:]
initPoint = {'1':(0,0), '2':(15,0), '3':(0,15), '4':(15,15)} # 4 robot initial positions
num_bots=len(initPoint)
micromouse.setMotorController(COREController(index, initPoint[index], '10.0.0.254'))
micromouse.setInitPoint(initPoint[index][0], initPoint[index][1])
# micromouse.addTask(StrategyTestMultiDFS(micromouse))
# micromouse.addTask(Rendezvous(micromouse))
micromouse.addTask(StrategyTestGoDown(Strategy))
micromouse.run()

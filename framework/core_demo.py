#!/usr/bin/env python3

#Author: Zhiwei Luo

from map import Map
from mouse import Micromouse
# from strategy import StrategyTestMultiDFS
from strategy import StrategyTestRendezvous
from controller import COREController
from socket import *
import time


mazeMap = Map(16, 16)
mazeMap.readFromFile('/home/parallels/Micromouse/mazes/2012japan-ef.txt') # load map
micromouse = Micromouse(mazeMap)
index = gethostname()[1:]
initPoint = {'1':(0,0), '2':(15,0), '3':(0,15), '4':(15,15)} # 4 robot initial positions
# initPoint = {'1': (0,0)}
num_bots = len(initPoint)
micromouse.setMotorController(COREController(index, initPoint[index], '10.0.0.254'))
micromouse.setInitPoint(initPoint[index][0], initPoint[index][1])
# micromouse.addTask(StrategyTestMultiDFS(micromouse))
print("broken task below")
micromouse.addTask(StrategyTestRendezvous(micromouse, initPoint, num_bots))
# micromouse.addTask(StrategyTestRendezvous(micromouse, initPoint, num_bots))
print("after initialization")
# num bots
tic = time.time()
micromouse.run()
toc = time.time()
time = int(toc - tic)
print("time(s):", time)

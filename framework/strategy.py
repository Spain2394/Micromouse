#!/usr/bin/env python3

# Author: Zhiwei Luo

from task import Strategy, NetworkInterface
# from map import Map
from time import sleep
import math
# import numpy as np


class StrategyTestProgress(Strategy):
    progress = 10

    def checkFinished(self):
        return self.progress <= 0

    def go(self):
        self.progress = self.progress - 1
        print(self.progress)


class StrategyTestCount(Strategy):
    progress = 0

    def checkFinished(self):
        return self.progress > 10

    def go(self):
        self.progress = self.progress + 1
        print(self.progress)
        sleep(1)


class StrategyTestGoDown(Strategy):
    mouse = None
    mapPainter = None
    progress = 0

    def __init__(self, mouse, mapPainter):
        self.mouse = mouse
        self.mapPainter = mapPainter

    def checkFinished(self):
        return self.progress >= 1

    def go(self):
        self.progress = self.progress + 1
        print(self.progress)
        sleep(1)
        self.mouse.goDown()
        self.mouse.goDown()
        self.mouse.goDown()
        self.mouse.goDown()
        self.mouse.goRight()
        self.mouse.goUp()
        cell = self.mouse.mazeMap.getCell(self.mouse.x, self.mouse.y)
        self.mapPainter.putRobotInCell(cell)
        sleep(1)


class StrategyTestDFS(Strategy):
    mouse = None
    mapPainter = None
    isVisited = []
    path = []
    isBack = False

    def __init__(self, mouse, mapPainter):
        self.mouse = mouse
        self.mapPainter = mapPainter
        self.isVisited = [[0 for i in range(self.mouse.mazeMap.width)] for j in range(
            self.mouse.mazeMap.height)]
        self.isVisited[self.mouse.x][self.mouse.y] = 1

    def checkFinished(self):
        return self.isBack

    def go(self):
        cell = self.mouse.mazeMap.getCell(self.mouse.x, self.mouse.y)
        self.mapPainter.drawCell(cell, 'grey')

        if self.mouse.canGoLeft() and not self.isVisited[self.mouse.x - 1][self.mouse.y]:
            self.path.append([self.mouse.x, self.mouse.y])
            self.isVisited[self.mouse.x - 1][self.mouse.y] = 1
            self.mouse.goLeft()
        elif self.mouse.canGoUp() and not self.isVisited[self.mouse.x][self.mouse.y - 1]:
            self.path.append([self.mouse.x, self.mouse.y])
            self.isVisited[self.mouse.x][self.mouse.y - 1] = 1
            self.mouse.goUp()
        elif self.mouse.canGoRight() and not self.isVisited[self.mouse.x + 1][self.mouse.y]:
            self.path.append([self.mouse.x, self.mouse.y])
            self.isVisited[self.mouse.x + 1][self.mouse.y] = 1
            self.mouse.goRight()
        elif self.mouse.canGoDown() and not self.isVisited[self.mouse.x][self.mouse.y + 1]:
            self.path.append([self.mouse.x, self.mouse.y])
            self.isVisited[self.mouse.x][self.mouse.y + 1] = 1
            self.mouse.goDown()
        else:
            if len(self.path) != 0:
                x, y = self.path.pop()
                if x < self.mouse.x:
                    self.mouse.goLeft()
                elif x > self.mouse.x:
                    self.mouse.goRight()
                elif y < self.mouse.y:
                    self.mouse.goUp()
                elif y > self.mouse.y:
                    self.mouse.goDown()
            else:
                self.isBack = True

        cell = self.mouse.mazeMap.getCell(self.mouse.x, self.mouse.y)
        self.mapPainter.putRobotInCell(cell)
        sleep(0.05)


class StrategyTestMultiDFS(Strategy):
    mouse = None
    isVisited = []
    path = []
    isBack = False
    network = None

    def __init__(self, mouse):
        self.mouse = mouse
        self.isVisited = [[0 for i in range(self.mouse.mazeMap.width)] for j in range(self.mouse.mazeMap.height)]
        self.isVisited[self.mouse.x][self.mouse.y] = 1
        self.network = NetworkInterface()
        self.network.initSocket()
        self.network.startReceiveThread()

    def checkFinished(self):
        return self.isBack

    def go(self):
        self.mouse.senseWalls()
        print(self.mouse.getCurrentCell().getWhichIsWall())
        sendData = {'x': self.mouse.x, 'y': self.mouse.y, 'up': not self.mouse.canGoUp(
        ), 'down': not self.mouse.canGoDown(), 'left': not self.mouse.canGoLeft(), 'right': not self.mouse.canGoRight()}
        print(self.network.sendStringData(sendData))
        recvData = self.network.retrieveData()
        print("recvData%s: "%recvData)

        while recvData:
            otherMap = recvData
            cell = self.mouse.mazeMap.getCell(otherMap['x'], otherMap['y'])
            self.isVisited[otherMap['x']][otherMap['y']] = 1
            if otherMap['up']:
                self.mouse.mazeMap.setCellUpAsWall(cell)
            if otherMap['down']:
                self.mouse.mazeMap.setCellDownAsWall(cell)
            if otherMap['left']:
                self.mouse.mazeMap.setCellLeftAsWall(cell)
            if otherMap['right']:
                self.mouse.mazeMap.setCellRightAsWall(cell)
            recvData = self.network.retrieveData()

        if self.mouse.canGoLeft() and not self.isVisited[self.mouse.x - 1][self.mouse.y]:
            self.path.append([self.mouse.x, self.mouse.y])
            self.isVisited[self.mouse.x - 1][self.mouse.y] = 1
            self.mouse.goLeft()
        elif self.mouse.canGoUp() and not self.isVisited[self.mouse.x][self.mouse.y - 1]:
            self.path.append([self.mouse.x, self.mouse.y])
            self.isVisited[self.mouse.x][self.mouse.y - 1] = 1
            self.mouse.goUp()
        elif self.mouse.canGoRight() and not self.isVisited[self.mouse.x + 1][self.mouse.y]:
            self.path.append([self.mouse.x, self.mouse.y])
            self.isVisited[self.mouse.x + 1][self.mouse.y] = 1
            self.mouse.goRight()
        elif self.mouse.canGoDown() and not self.isVisited[self.mouse.x][self.mouse.y + 1]:
            self.path.append([self.mouse.x, self.mouse.y])
            self.isVisited[self.mouse.x][self.mouse.y + 1] = 1
            self.mouse.goDown()
        else:
            if len(self.path) != 0:
                x, y = self.path.pop()
                if x < self.mouse.x:
                    self.mouse.goLeft()
                elif x > self.mouse.x:
                    self.mouse.goRight()
                elif y < self.mouse.y:
                    self.mouse.goUp()
                elif y > self.mouse.y:
                    self.mouse.goDown()
            else:
                self.isBack = True

        sleep(0.5)


class StrategyTestRendezvous(Strategy):
    mouse = None
    isVisited = []
    path = []
    network = None
    neighbors_states = {} # group states including self
    topological_neighbors = []
    switchGoal = False

    stop_condition = False
    whoami = -1
    dx = []
    dy = []
    isBack = False
    iterations = 0
    num_bots = -1
    # starting_pose = ()
    starting_pose = ()
    drive = (None, None)
    # mazeMap = None


    # define number of robots
    def __init__(self, mouse, initPoint, num_bots):
        # print("INTIA")

        self.mouse = mouse
        self.num_bots = num_bots
        self.isVisited = [[0 for i in range(self.mouse.mazeMap.width)] for j in range(self.mouse.mazeMap.height)]
        # print(self.isVisited)
        self.isVisited[self.mouse.x][self.mouse.y] = 1
        # print("made it past initialization")
        for i in range(1, self.num_bots + 1):
            if initPoint[str(i)] == (self.mouse.x, self.mouse.y):
                self.whoami = i
            self.neighbors_states[i] = {'robot': i, 'x': initPoint[str(i)][0], 'y': initPoint[str(i)][1], 'direction':'UP'}
            # print(self.neighbors_states[i])
            # print("whoami:%s"%self.whoami)

        self.starting_pose = initPoint[str(self.whoami)]
        self.network = NetworkInterface()
        self.network.initSocket()
        self.network.startReceiveThread()
        # print("network start receive thread")

    def checkFinished(self):
        return self.isBack


    def distance_to_near_neigh(self):
        dx_temp = 0
        dy_temp = 0
        follow_him = -1
        distance = 100 # some big number
        temp_distance = 0

        for bots in range(1,self.num_bots+1):
            if bots == self.whoami:
                continue
            else:
                dx_temp = self.neighbors_states[bots]['x'] - self.mouse.x

                dy_temp = self.neighbors_states[bots]['y'] - self.mouse.y

                temp_distance = (dx_temp*dx_temp + dy_temp*dy_temp)**(1/2)

                if temp_distance < distance:
                    distance = temp_distance
                    follow_him = bots

        return distance,follow_him

    def distance_to_far_neigh(self):
        dx_temp = 0
        dy_temp = 0
        follow_him = -1
        distance = 0 # some small number
        temp_distance = 0
        cost = 5

        for bots in range(1,self.num_bots+1):

            if bots == self.whoami:
                continue
            else:
                dx_temp = self.neighbors_states[bots]['x'] - self.mouse.x

                dy_temp = self.neighbors_states[bots]['y'] - self.mouse.y

                temp_distance = (dx_temp*dx_temp + dy_temp*dy_temp)**(1/2)

                if temp_distance > distance:
                    distance = temp_distance
                    follow_him = bots

        return distance,follow_him

    def distance_to_you(self):
        distance = -1
        if abs(dx_temp) < abs(self.dx):
            self.dx = dx_temp
            if self.dx < 0: x_Dir = "LEFT"
            else: x_Dir = "RIGHT"

        elif abs(dy_temp) < abs(self.dy):
            self.dy = dx_temp
            if self.dy < 0: y_Dir = "UP" # opposite to intuition
            else: y_Dir = "DOWN"

        if abs(self.dx) < abs(self.dy):return x_Dir
        # elif abs(self.dx) == abs(self.dy):return y_Dir # just make up defualt when tie
        else: return y_Dir


    def Centroid(self):
        area = 0
        centroid = ()
        weights = 0
        dx = 0
        dy = 0
        dx_temp = 0
        dy_temp = 0
        weighted_x = 0
        weighted_y = 0

        for bot in self.neighbors_states:
            dx_temp = abs(self.neighbors_states[bot]['x']-self.mouse.x)
            dy_temp = abs(self.neighbors_states[bot]['y']-self.mouse.y)
            if dx_temp > dx: dx = dx_temp
            if dy_temp > dy: dy = dy_temp

        weighted_x = math.floor(((dx/dy)*(self.mouse.x + dx))/2)
        weighted_y = math.floor(((dy/dx)*(self.mouse.y + dy))/2)

        centroid = (weighted_x, weighted_y)
        return centroid

    def GroupCentroid(self):
        group_centroid = ()
        sumCx = 0
        sumCy = 0
        sumAc = 0
        lstP = self.neighbors_states.copy() # copy the neighbors_states
        for i in range(1,self.num_bots+1):
            sumCx += lstP[i]['x']
            sumCy += lstP[i]['y']

        group_centroid = (math.ceil(sumCx/self.num_bots),math.ceil(sumCy/self.num_bots))
        print("group_centroid check:" , group_centroid)
        return group_centroid

    def BestMove(self,utility,maze):
        direction = ['Up','Down','Left','Right']


        return(direction_best, movement_best,state_next_best,reward_max)


    def check_greatest_distance(self):
        x_Dir = None
        y_Dir = None
        cost = 2

        shortest_path_list_x = []
        shortest_path_list_y = []
        # print("I'm in")

        for bots in self.neighbors_states:
            if bots != self.whoami:
                dx_temp = self.neighbors_states[bots]['x'] - self.mouse.x
                dy_temp = self.neighbors_states[bots]['y'] - self.mouse.y

                if dy_temp < 0 and self.mouse.direction is not 'DOWN':
                    dy_temp *= cost
                elif dy_temp > 0 and self.mouse.direction is not 'UP':
                    dy_temp *= cost

                if dx_temp < 0 and self.mouse.direction is not 'LEFT':
                    dx_temp *= cost
                elif dx_temp > 0 and self.mouse.direction is not 'RIGHT':
                    dx_temp *= cost

                shortest_path_list_x.append(dx_temp)
                shortest_path_list_y.append(dy_temp)

        return shortest_path_list_x, shortest_path_list_y

    def distance_to_wall(self,cell,direction):
        check = True
        open_distance = 0
        move_dir = {'U': [0,-1],'D': [0,1],'L': [-1,0],'R': [1,0]}


        current_cell = [cell.x,cell.y] # are we modifying current cell?? if so make a copy
        dir = direction[0]
        # print(dir)

        height = self.mouse.mazeMap.height
        width = self.mouse.mazeMap.width

        while check:
            if current_cell[0] >= 0 and current_cell[1] >=0 and current_cell[0] < width and current_cell[1] < height:
                if self.mouse.mazeMap.getCell(current_cell[0],current_cell[1]).getIsThereWall(dir):
                    # print("in the conditional")
                    open_distance +=1
                    current_cell[0] += move_dir[dir][0]
                    # print("current_cell_0:",current_cell[0])
                    current_cell[1] += move_dir[dir][1]
                else: check = False
            else: check = False
        return open_distance

    def cost(self,goal):

        direction_list = {'U': [0,-1],'D': [0,1],'L': [-1,0],'R': [1,0]}
        my_dir = 'U'  #
        moves = []
        state = (self.mouse.x,self.mouse.y)
        cost = 100 # some very high cost
        isGoal = False
        gamma = 0.01 # cost is lower when distance is longer
        expense = 0

        hasBeen = [[0 for i in range(self.mouse.mazeMap.width)] for j in range(self.mouse.mazeMap.height)]
        takeAction = []
        hasBeen[state[0]][state[1]] = 1 # has been at initial location
        open = [(expense,cost,state,my_dir)] # some constants a start point and an end point

        while len(open) < 5 and len(open) >0: # give me 20 good points
            item = open.pop(0) # pop appended item

            expense = item[0]
            cost = item[1]
            state = item[2]
            my_dir = item[3]

            counter = 0
            for d in direction_list:
                if self.mouse.mazeMap.getCell(state[0],state[1]).getIsThereWall(d) == False: # while mouse can move in some direction
                    counter +=1

                    delta = direction_list[d]
                    next_state = (state[0] + delta[0], state[1] + delta[1])
                    if hasBeen[next_state[0]][next_state[1]] == 0: # hasn't been
                        next_cost = cost + 1 if my_dir is d else 2 #

                        expense = self.priority(next_state,d,goal)
                        takeAction.append([expense,next_state,d])
                        hasBeen[next_state[0]][next_state[1]] = 1

                        open.append((expense,next_cost,next_state,d))

            takeAction.sort()
            open.sort() # sort based on low expense
            return takeAction # return after only one iteration

    def dist(self,xy):
        x,y = xy
        dx_temp = 0
        dy_temp = 0
        distance = 0 # some small number

        dx_temp = x - self.mouse.x

        dy_temp = y - self.mouse.y

        distance = (dx_temp*dx_temp + dy_temp*dy_temp)**(1/2)

        return distance

    def priority(self,state,d,goal):
        goal_x,goal_y = goal

        x,y = state[0],state[1]

        alpha = 10 # weight for going in a straight line
        beta = 0.001 # weight for going towards gradient
        zeta = 1
        epsilon = 0.1
        energy = ((self.mouse.x - x)**2 + (self.mouse.y -y)**2)**(1/2)

        straight_line = 0
        shortest_distance = 0
        gradient = 0
        expense = 0

        cell = self.mouse.mazeMap.getCell(x,y)

        straight_line = self.distance_to_wall(cell,d)
        # print("straight line: ", straight_line)
        # gradient = (((x - self.GroupCentroid()[0])**2 + (y-self.GroupCentroid()[1])**2)**(1/2))
        gradient = (((x - goal_x)**2 + (y-goal_y)**2)**(1/2))
        expense = gradient/2
        return expense

    def follow_it(self, near_bot):
        cost = 0
        state = (self.neighbors_states[near_bot]['x'], self.neighbors_states[near_bot]['y'])
        direction = self.neighbors_states[near_bot]['direction']
        action = [(cost, state, direction)]
        return action

    def go(self):
        self.iterations +=1
        self.mouse.senseWalls()
        self.mouse.getCurrentCell().getWhichIsWall()
        sendData = {'robot': self.whoami, 'x': self.mouse.x, 'y': self.mouse.y, 'up': not self.mouse.canGoUp(
        ), 'down': not self.mouse.canGoDown(), 'left': not self.mouse.canGoLeft(), 'right': not self.mouse.canGoRight(), 'direction':self.mouse.direction}
        # print(sendData)
        self.network.sendStringData(sendData)
        recvData = self.network.retrieveData()

        while recvData:

            otherMap = recvData
            cell = self.mouse.mazeMap.getCell(otherMap['x'], otherMap['y'])
            self.neighbors_states[otherMap['robot']] = {'robot':otherMap['robot'], 'x': otherMap['x'], 'y': otherMap['y'], 'direction':self.mouse.direction} # update neighbors_states as received
            if otherMap['up']:
                self.mouse.mazeMap.setCellUpAsWall(cell)
            if otherMap['down']:
                self.mouse.mazeMap.setCellDownAsWall(cell)
            if otherMap['left']:
                self.mouse.mazeMap.setCellLeftAsWall(cell)
            if otherMap['right']:
                self.mouse.mazeMap.setCellRightAsWall(cell)
            recvData = self.network.retrieveData()


        print('-----------------------------')
        group_centroid = ()
        distance = 0
        goal = ()
        action = ()
        cell = self.mouse.mazeMap.getCell(self.mouse.x,self.mouse.y)
        direction = self.mouse.direction
        group_centroid = self.GroupCentroid()
        print("group centroid: ", group_centroid)
        far_distance,far_bot = self.distance_to_far_neigh()
        print("far distance:",far_distance)
        self.switchGoal = False
        moved = False
        threshold = 1
        distance, near_bot = self.distance_to_near_neigh() # goal begins as near neighbor
        goal_1 = (None,None)

        head = True if near_bot > self.whoami else False
        goal = (self.neighbors_states[near_bot]['x'],self.neighbors_states[near_bot]['y'])

        print("far_distance", far_distance)
        print("far bot", far_bot)
        if far_distance < threshold:
            print("distance to far neighbor:", far_distance)
            self.isBack = True


        if (self.mouse.x,self.mouse.y) == goal:
            if not head:
                self.switchGoal = True
                action = self.follow_it(near_bot)

            else:
                 far_distance,far_bot = self.distance_to_far_neigh()
                 group_centroid = self.group_centroid
                 close_group = (self.neighbors_states[far_bot]['x'], self.neighbors_states[far_bot]['y'])
                 cent = self.dist(group_centroid)
                 far = self.dist(close_group)
                 goal = close_group if far < cent else group_centroids
                 # goal = self.group_centroid # try group centroid

        else: switchGoal = False

        action = self.cost(goal)

        # if self.isBack:
        #      print("RENDEZVOUS")
        #      self.checkFinished() # check if finished


        for moves in action: # best action loop
            print(moves)
            x,y = moves[1]
            direction = moves[2]

            if moved == True: break

            print("x,y", (x,y))
            print("self : ", self.switchGoal)
            print("head : ", head)


            if (x,y) == goal:
                if not head:
                    self.switchGoal = True #
                    x,y = (self.neighbors_states[near_bot]['x'],self.neighbors_states[near_bot]['y'])


            if self.isVisited[x][y] == 0 or self.switchGoal: # may be some confusion here if not continuos
                print("in")
                print("self.isVisited[x][y]: ", self.isVisited[x][y]==0)

                print(self.mouse.getCurrentCell().getWhichIsWall())
                if self.mouse.x < x and self.mouse.canGoRight():
                    self.path.append([self.mouse.x, self.mouse.y])
                    self.isVisited[self.mouse.x + 1][self.mouse.y] = 1
                    self.mouse.goRight()
                    moved = True
                    self.neighbors_states[self.whoami] = {'robot': self.whoami, 'x':self.mouse.x , 'y': self.mouse.y,'direction':'RIGHT'}
                    print("RIGHT")
                elif self.mouse.x > x and self.mouse.canGoLeft():
                    self.path.append([self.mouse.x, self.mouse.y])
                    self.isVisited[self.mouse.x-1][self.mouse.y] = 1
                    self.mouse.goLeft()
                    moved = True
                    self.neighbors_states[self.whoami] = {'robot': self.whoami, 'x':self.mouse.x , 'y': self.mouse.y,'direction':'LEFT'}
                    print("LEFT")
                elif self.mouse.y > y and self.mouse.canGoUp():
                    self.path.append([self.mouse.x, self.mouse.y])
                    self.isVisited[self.mouse.x][self.mouse.y-1] = 1
                    self.mouse.goUp()
                    moved = True
                    self.neighbors_states[self.whoami] = {'robot': self.whoami, 'x':self.mouse.x , 'y': self.mouse.y,'direction':'UP'}
                    print("UP")
                elif self.mouse.y < y and self.mouse.canGoDown():
                    self.path.append([self.mouse.x, self.mouse.y])
                    self.isVisited[self.mouse.x][self.mouse.y+1] = 1
                    self.mouse.goDown()
                    moved = True
                    self.neighbors_states[self.whoami] = {'robot': self.whoami, 'x':self.mouse.x , 'y': self.mouse.y,'direction':'DOWN'}
                    print("DOWN")
        sleep(0.01)

        if moved == False: #backtrack if unable to move by best action
            if len(self.path) != 0:
                x, y = self.path.pop()
                if x < self.mouse.x:
                    self.mouse.goLeft()
                    self.neighbors_states[self.whoami] = {'robot': self.whoami, 'x':self.mouse.x , 'y': self.mouse.y,'direction':'LEFT'}
                    print("LEFT")
                    moved = True
                elif x > self.mouse.x:
                    self.mouse.goRight()
                    self.neighbors_states[self.whoami] = {'robot': self.whoami, 'x':self.mouse.x , 'y': self.mouse.y,'direction':'RIGHT'}
                    moved = True
                    print("RIGHT")
                elif y < self.mouse.y:
                    self.mouse.goUp()
                    self.neighbors_states[self.whoami] = {'robot': self.whoami, 'x':self.mouse.x , 'y': self.mouse.y,'direction':'UP'}
                    moved = True
                    print("UP")
                elif y > self.mouse.y:
                    self.mouse.goDown()
                    self.neighbors_states[self.whoami] = {'robot': self.whoami, 'x':self.mouse.x , 'y': self.mouse.y,'direction':'DOWN'}
                    moved = True
                    print("DOWN")
            else:
                self.isBack = True
        sleep(0.05)


  
class StrategyTestDFSEV3(Strategy):
    mouse = None
    # mapPainter = None
    isVisited = []
    path = []
    isBack = False

    def __init__(self, mouse):
        self.mouse = mouse
        # self.mapPainter = mapPainter
        self.isVisited = [[0 for i in range(self.mouse.mazeMap.width)] for j in range(
            self.mouse.mazeMap.height)]
        self.isVisited[self.mouse.x][self.mouse.y] = 1

    def checkFinished(self):
        return self.isBack

    def go(self):
        # cell = self.mouse.mazeMap.getCell(self.mouse.x, self.mouse.y)
        # self.mapPainter.drawCell(cell, 'grey')
        self.mouse.senseWalls()
        print(self.mouse.getCurrentCell().getWhichIsWall())

        if self.mouse.canGoLeft() and not self.isVisited[self.mouse.x - 1][self.mouse.y]:
            self.path.append([self.mouse.x, self.mouse.y])
            self.isVisited[self.mouse.x - 1][self.mouse.y] = 1
            self.mouse.goLeft()
        elif self.mouse.canGoUp() and not self.isVisited[self.mouse.x][self.mouse.y - 1]:
            self.path.append([self.mouse.x, self.mouse.y])
            self.isVisited[self.mouse.x][self.mouse.y - 1] = 1
            self.mouse.goUp()
        elif self.mouse.canGoRight() and not self.isVisited[self.mouse.x + 1][self.mouse.y]:
            self.path.append([self.mouse.x, self.mouse.y])
            self.isVisited[self.mouse.x + 1][self.mouse.y] = 1
            self.mouse.goRight()
        elif self.mouse.canGoDown() and not self.isVisited[self.mouse.x][self.mouse.y + 1]:
            self.path.append([self.mouse.x, self.mouse.y])
            self.isVisited[self.mouse.x][self.mouse.y + 1] = 1
            self.mouse.goDown()
        else:
            if len(self.path) != 0:
                x, y = self.path.pop()
                if x < self.mouse.x:
                    self.mouse.goLeft()
                elif x > self.mouse.x:
                    self.mouse.goRight()
                elif y < self.mouse.y:
                    self.mouse.goUp()
                elif y > self.mouse.y:
                    self.mouse.goDown()
            else:
                self.isBack = True

        # cell = self.mouse.mazeMap.getCell(self.mouse.x, self.mouse.y)
        # self.mapPainter.putRobotInCell(cell)


class StrategyTestGoStepEV3(Strategy):
    mouse = None
    progress = 0

    def __init__(self, mouse):
        self.mouse = mouse

    def checkFinished(self):
        return self.progress >= 1

    def go(self):
        self.progress = self.progress + 1
        self.mouse.senseWalls()
        print(self.mouse.getCurrentCell().getWhichIsWall())
        self.mouse.goLeft()
        self.mouse.senseWalls()
        print(self.mouse.getCurrentCell().getWhichIsWall())
        self.mouse.goRight()
        self.mouse.senseWalls()
        print(self.mouse.getCurrentCell().getWhichIsWall())
        self.mouse.goUp()
        self.mouse.senseWalls()
        print(self.mouse.getCurrentCell().getWhichIsWall())
        self.mouse.goDown()
        self.mouse.senseWalls()
        print(self.mouse.getCurrentCell().getWhichIsWall())
        sleep(1)


class StrategyTestInitEV3(Strategy):
    mouse = None
    flag = False

    def __init__(self, mouse):
        self.mouse = mouse

    def checkFinished(self):
        return self.flag

    def go(self):
        self.mouse.commandTranslator.motorController.gyreset()
        self.flag = True
        sleep(1)


class StrategyTestDFSDisplayEV3(Strategy):
    mouse = None
    isVisited = []
    path = []
    isBack = False

    def __init__(self, mouse):
        self.mouse = mouse
        self.isVisited = [[0 for i in range(self.mouse.mazeMap.width)] for j in range(
            self.mouse.mazeMap.height)]
        self.isVisited[self.mouse.x][self.mouse.y] = 1
        self.network = NetworkInterface()
        self.network.initSocket()

    def checkFinished(self):
        return self.isBack

    def go(self):
        self.mouse.senseWalls()
        print(self.mouse.getCurrentCell().getWhichIsWall())
        sendData = {'x': self.mouse.x, 'y': self.mouse.y, 'up': self.mouse.canGoUp(
        ), 'down': self.mouse.canGoDown(), 'left': self.mouse.canGoLeft(), 'right': self.mouse.canGoRight()}
        self.network.sendStringData(sendData)

        if self.mouse.canGoLeft() and not self.isVisited[self.mouse.x - 1][self.mouse.y]:
            self.path.append([self.mouse.x, self.mouse.y])
            self.isVisited[self.mouse.x - 1][self.mouse.y] = 1
            self.mouse.goLeft()
        elif self.mouse.canGoUp() and not self.isVisited[self.mouse.x][self.mouse.y - 1]:
            self.path.append([self.mouse.x, self.mouse.y])
            self.isVisited[self.mouse.x][self.mouse.y - 1] = 1
            self.mouse.goUp()
        elif self.mouse.canGoRight() and not self.isVisited[self.mouse.x + 1][self.mouse.y]:
            self.path.append([self.mouse.x, self.mouse.y])
            self.isVisited[self.mouse.x + 1][self.mouse.y] = 1
            self.mouse.goRight()
        elif self.mouse.canGoDown() and not self.isVisited[self.mouse.x][self.mouse.y + 1]:
            self.path.append([self.mouse.x, self.mouse.y])
            self.isVisited[self.mouse.x][self.mouse.y + 1] = 1
            self.mouse.goDown()
        else:
            if len(self.path) != 0:
                x, y = self.path.pop()
                if x < self.mouse.x:
                    self.mouse.goLeft()
                elif x > self.mouse.x:
                    self.mouse.goRight()
                elif y < self.mouse.y:
                    self.mouse.goUp()
                elif y > self.mouse.y:
                    self.mouse.goDown()
            else:
                self.isBack = True

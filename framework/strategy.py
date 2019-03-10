#!/usr/bin/env python3

# Author: Zhiwei Luo

from task import Strategy, NetworkInterface
from map import Map
from time import sleep
import math


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
        print("hello")
        self.mouse = mouse
        self.isVisited = [[0 for i in range(self.mouse.mazeMap.width)] for j in range(
            self.mouse.mazeMap.height)]
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

    stop_condition = False
    whoami = -1
    dx = []
    dy = []
    isBack = False
    iterations = 0
    num_bots = -1
    # starting_pose = ()
    starting_pose = ()
    centroid = ()
    mazeMap = None


    # define number of robots
    def __init__(self, mouse, initPoint,num_bots,mazeMap):
        # add
        self.mazeMap = mazeMap
        self.mouse = mouse
        self.num_bots = num_bots
        self.centroid = (mazeMap.width,mazeMap.height)
        self.isVisited = [[0 for i in range(self.mouse.mazeMap.width)] for j in range(
            self.mouse.mazeMap.height)]
        self.isVisited[self.mouse.x][self.mouse.y] = 1
        for i in range(1, num_bots + 1):
            if initPoint[str(i)] == (self.mouse.x, self.mouse.y):
                self.whoami = i
            self.neighbors_states[i] = {'robot': i, 'x': initPoint[str(i)][0], 'y': initPoint[str(i)][1], 'direction':'UP'}
            # print(self.neighbors_states[i])
            # print("whoami:%s"%self.whoami)

        self.starting_pose = initPoint[str(self.whoami)]
        self.network = NetworkInterface()
        self.network.initSocket()
        self.network.startReceiveThread()

    def checkFinished(self):
        return self.isBack

    def check_greatest_distance(self):
        x_Dir = None
        y_Dir = None
        cost = 2


        shortest_path_list_x = []
        shortest_path_list_y = []
        print("I'm in")

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

        # shortest_path_list_x.sort()
        # shortest_path_list_y.sort()
        return shortest_path_list_x, shortest_path_list_y


    def check_priority(self, dx_list, dy_list):
        N = len(dx_list)
        dx_list.sort() # make sure you are comparing only the smallest values
        dy_list.sort() # dx and dy are already least -> greatest
        priority = []
        trash = []
        #TODO come up with a second order priority
        # print("check priority")
        for i in range(N):
            # go for closest
            if i == self.whoami:
                continue

            # the priority will be based on largest distance,
            # dx represents robot i's position
            #
            if dx_list[i] < dy_list[i]:
                if dx_list[i] < 0:
                    priority.append('L')
                else: priority.append('R')

            else:
                if dy_list[i] < 0: priority.append('D')
                else: priority.append('U')


        # print("returning: %s"% priority)
        return priority

    def distance_to_near_neigh(self):
        dx_temp = 0
        dy_temp = 0
        # closest_bot = {}
        follow_him = -1
        distance = 100 # some big number
        temp_distance = 0
        cost = 5


        print("made it into distance_to_near_neigh 1")
        for bots in range(1,self.num_bots+1):
            print("made it into distance_to_near_neigh 2")
            print("bots%s"%bots)
            if bots == self.whoami:
                continue
            else:
                dx_temp = self.neighbors_states[bots]['x'] - self.mouse.x

                dy_temp = self.neighbors_states[bots]['y'] - self.mouse.y

                temp_distance = (dx_temp*dx_temp + dy_temp*dy_temp)**(1/2)
                print("temp_distance: %s"%temp_distance)

                if temp_distance < distance:
                    distance = temp_distance
                    follow_him = bots
                    print("distance: %s"%distance)
                    print("following_him: %s"%follow_him)

        print("final distance: %s"%distance)
        return distance,follow_him


    # def distance_to_you(self):
    #     distance = -1



        #

            # smallest number
        #     if abs(dx_temp) < abs(self.dx):
        #         self.dx = dx_temp
        #         if self.dx < 0: x_Dir = "LEFT"
        #         else: x_Dir = "RIGHT"
        #
        #     elif abs(dy_temp) < abs(self.dy):
        #         self.dy = dx_temp
        #         if self.dy < 0: y_Dir = "UP" # opposite to intuition
        #         else: y_Dir = "DOWN"
        #
        #
        # if abs(self.dx) < abs(self.dy):return x_Dir
        # # elif abs(self.dx) == abs(self.dy):return y_Dir # just make up defualt when tie
        # else: return y_Dir

    # def find_best_move(self,goal):
    #     # goal = furthest robot position?
    #     g = 0
    #     h =

    # def find_best_move(self,target):
    #     g = 0
    #     h = self.starting_pose
    #     f = g + h
    #     open = [(f,h,g,)]

    # def getGoodMoved(self):
        # going straiht is good
        # go towards target
        # target defined by centroid


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


        print("centroid function")
        for bot in self.neighbors_states:
            dx_temp = abs(self.neighbors_states[bot]['x']-self.mouse.x)
            dy_temp = abs(self.neighbors_states[bot]['y']-self.mouse.y)
            if dx_temp > dx: dx = dx_temp
            if dy_temp > dy: dy = dy_temp

        print("out of centroid loop")
        weighted_x = math.floor(((dx/dy)*(self.mouse.x + dx))/2)
        weighted_y = math.floor(((dy/dx)*(self.mouse.y + dy))/2)
        print("weighted_x: ",weighted_x)

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
        return group_centroid

    # def BestMove(self,utility,maze):
    #     direction = ['Up','Down','Left','Right']
    #
    #
    #     return(direction_best, movement_best,state_next_best,reward_max)

    def distance_to_wall(self,cell,direction):
        # how long can mouse go in direction,
        print("got into distance to cell")
        print("cellx:",cell[0])
        print("celly:",cell[1])
        move_dir = {'Up': [0,-1],'Down': [0,1],'Left': [-1,0],'Right': [1,0]}
        print(move_dir)
        current_cell = list(cell) # copy the current cell of robot
        print(current_cell)
        check = True
        # mazemap allows me to get cell object which cas getWhichIsWall

        while check:
            if self.mazeMap.getCell(current_cell[0],current_cell[1]).getWhichIsWall() is not direction:
                distance +=1
                current_cell[0] = move_dir[direction][0]
                current_cell[1] = move_dir[direction][1]
            else: check = False
        return distance


    def go(self):
        self.iterations +=1
        print("ITER: %s"%self.iterations)
        self.mouse.senseWalls()
        print(self.mouse.getCurrentCell().getWhichIsWall())
        sendData = {'robot': self.whoami, 'x': self.mouse.x, 'y': self.mouse.y, 'up': not self.mouse.canGoUp(
        ), 'down': not self.mouse.canGoDown(), 'left': not self.mouse.canGoLeft(), 'right': not self.mouse.canGoRight(), 'direction':self.mouse.direction}
        print(sendData)
        # print(self.network.sendStringData(sendData))


        recvData = self.network.retrieveData()
        print("recvData: %s"% recvData)
        # one packet at a time
        while recvData:

            otherMap = recvData
            cell = self.mouse.mazeMap.getCell(otherMap['x'], otherMap['y'])
            self.isVisited[otherMap['x']][otherMap['y']] = 1
            self.neighbors_states[otherMap['robot']] = {'robot':otherMap['robot'], 'x': otherMap['x'], 'y': otherMap['y'], 'direction':self.mouse.direction} # update neighbors_states as received
            print(self.neighbors_states[otherMap['robot']]) # update neighbors_states as received)
            if otherMap['up']:
                self.mouse.mazeMap.setCellUpAsWall(cell)
            if otherMap['down']:
                self.mouse.mazeMap.setCellDownAsWall(cell)
            if otherMap['left']:
                self.mouse.mazeMap.setCellLeftAsWall(cell)
            if otherMap['right']:
                self.mouse.mazeMap.setCellRightAsWall(cell)
            recvData = self.network.retrieveData()


        group_centroid = ()
        distance = 0
        cell = self.mazeMap.getCell(self.mouse.x,self.mouse.y)
        direction = self.mouse.direction

        self.centroid = self.Centroid()
        group_centroid = self.GroupCentroid()
        print("group centroid", group_centroid)
        distance = self.distance_to_wall(cell,direction)
        print("distance to wall: ", distance)


        if self.mouse.canGoLeft() and not self.isVisited[self.mouse.x-1][self.mouse.y]:
            self.path.append([self.mouse.x, self.mouse.y])
            self.isVisited[self.mouse.x - 1][self.mouse.y] = 1
            self.mouse.goLeft()
            self.neighbors_states[self.whoami] = {'robot': self.whoami, 'x':self.mouse.x , 'y': self.mouse.y,'direction':'LEFT'}
            moved = True
        elif self.mouse.canGoUp() and not self.isVisited[self.mouse.x][self.mouse.y-1]:
            self.path.append([self.mouse.x, self.mouse.y])
            self.isVisited[self.mouse.x][self.mouse.y - 1] = 1
            self.mouse.goUp()
            self.neighbors_states[self.whoami] = {'robot': self.whoami, 'x':self.mouse.x , 'y': self.mouse.y,'direction':'UP'}
            moved = True
        elif self.mouse.canGoRight() and not self.isVisited[self.mouse.x+1][self.mouse.y]:
            self.path.append([self.mouse.x, self.mouse.y])
            self.isVisited[self.mouse.x + 1][self.mouse.y] = 1
            self.mouse.goRight()
            self.neighbors_states[self.whoami] = {'robot': self.whoami, 'x':self.mouse.x , 'y': self.mouse.y,'direction':'RIGHT'}
            moved = True
        elif self.mouse.canGoDown() and not self.isVisited[self.mouse.x][self.mouse.y+1]:
            self.path.append([self.mouse.x, self.mouse.y])
            self.isVisited[self.mouse.x][self.mouse.y + 1] = 1
            self.mouse.goDown()
            self.neighbors_states[self.whoami] = {'robot': self.whoami, 'x':self.mouse.x , 'y': self.mouse.y,'direction':'DOWN'}
            moved = True
        else: # if no gradient available, then backtrack
            if len(self.path) !=0:
                x, y = self.path.pop()
                if x < self.mouse.x:
                    self.mouse.goLeft()
                    self.neighbors_states[self.whoami] = {'robot': self.whoami, 'x':self.mouse.x , 'y': self.mouse.y,'direction':'LEFT'}
                    moved = True
                elif x > self.mouse.x:
                    self.mouse.goRight()
                    self.neighbors_states[self.whoami] = {'robot': self.whoami, 'x':self.mouse.x , 'y': self.mouse.y,'direction':'RIGHT'}
                    moved = True
                elif y < self.mouse.y:
                    self.mouse.goUp()
                    self.neighbors_states[self.whoami] = {'robot': self.whoami, 'x':self.mouse.x , 'y': self.mouse.y,'direction':'UP'}
                    moved = True
                elif y > self.mouse.y:
                    self.mouse.goDown()
                    self.neighbors_states[self.whoami] = {'robot': self.whoami, 'x':self.mouse.x , 'y': self.mouse.y,'direction':'DOWN'}
                    moved = True
            else:
                self.isBack = True

        sleep(0.5)


            #
            # 		# calculate the direction to pursue
            # 		def slope(self, num_bots):
            # 			# find largest delta x, and delta deltay
            # 			for k in range(1, num_bots + 1):
            # 				if k != self.whoami:
            # 					dx_temp = self.mouse.x - self.neighbors_states[str(k)]['x']
            # 					dy_temp = self.mouse.y - self.neighbors_states[str(k)]['y']
            #
            # 					if dx_temp > dx:
            # 						self.dx = dx_temp # right/left
            # 					if dy_temp > dy:
            # 						self.dy = dy_temp # up/down
            #
            #
            #         if self.mouse.canGoLeft() and self.dx < 0 and (self.dx > self.dy):
            #             self.path.append([self.mouse.x, self.mouse.y])
            # 			self.mouse.goLeft()
            # 			self.neighbors_states[self.whoami] = {'robot:' whoami, 'x': self.mouse.x, 'y': self.mouse.y}
            #         elif self.mouse.canGoUp() and self.dy > 0 and (self.dy > self.dx):
            #             self.path.append([self.mouse.x, self.mouse.y])
            #             self.mouse.goUp()
            # 			self.neighbors_states[self.whoami] = {'robot:' whoami, 'x': self.mouse.x, 'y': self.mouse.y}
            #         elif self.mouse.canGoRight() and self.dx > 0 and (self.dx > self.dy):
            #             self.path.append([self.mouse.x, self.mouse.y])
            #             self.mouse.goRight()
            # 			self.neighbors_states[self.whoami] = {'robot:' whoami, 'x': self.mouse.x, 'y': self.mouse.y}
            #         elif self.mouse.canGoDown() and self.dy < 0 and (self.dy > self.dx):
            #             self.path.append([self.mouse.x, self.mouse.y])
            #             self.mouse.goDown()
            # 			self.neighbors_states[self.whoami] = {'robot:' whoami, 'x': self.mouse.x, 'y': self.mouse.y}
            #         else:
            # 			# define !stop condition
            #             if len(self.path) != 0:
            #                 x, y = self.path.pop()
            #                 if x < self.mouse.x:
            #                     self.mouse.goLeft()s
            #                 elif x > self.mouse.x:
            #                     self.mouse.goRight()
            #                 elif y < self.mouse.y:
            #                     self.mouse.goUp()
            #                 elif y > self.mouse.y:
            #                     self.mouse.goDown()
            #             else:
            #                 self.stop_condition = 1
            #
            #         sleep(0.5)

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

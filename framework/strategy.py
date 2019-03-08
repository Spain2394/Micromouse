#!/usr/bin/env python3

# Author: Zhiwei Luo

from task import Strategy, NetworkInterface
from time import sleep


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
    neighbors_states = {}
    topological_neighbors = []

    stop_condition = False
    whoami = -1
    dx = []
    dy = []
    isBack = False
    iterations = 0

    # define number of robots
    def __init__(self, mouse, initPoint):
        # add
        self.mouse = mouse
        num_bots = len(initPoint)
        self.isVisited = [[0 for i in range(self.mouse.mazeMap.width)] for j in range(
            self.mouse.mazeMap.height)]
        self.isVisited[self.mouse.x][self.mouse.y] = 1
        for i in range(1, num_bots + 1):
            if initPoint[str(i)] != (self.mouse.x, self.mouse.y):
                self.neighbors_states[i] = {'robot': i, 'x': initPoint[str(i)][0], 'y': initPoint[str(i)][1]}
                print(self.neighbors_states[i])
            else:
                self.whoami = i
        self.network = NetworkInterface()
        self.network.initSocket()
        self.network.startReceiveThread()

    def checkFinished(self):
        return self.isBack


    def check_greatest_distance(self):
        x_Dir = None
        y_Dir = None

        shortest_path_list_x = []
        shortest_path_list_y = []
        print("I'm in")

        for bots in self.neighbors_states:
            dx_temp = self.neighbors_states[bots]['x'] - self.mouse.x
            dy_temp = self.neighbors_states[bots]['y'] - self.mouse.y
            shortest_path_list_x.append(dx_temp)
            shortest_path_list_y.append(dy_temp)
            print("dx: %s"%dx_temp)
            print("dy: %s"%dy_temp)

        # shortest_path_list_x.sort()
        # shortest_path_list_y.sort()
        print("%s,%s"%(shortest_path_list_x,shortest_path_list_y))
        return shortest_path_list_x, shortest_path_list_y


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

    def go(self):
        self.iterations +=1
        print("ITER: %s"%self.iterations)
        self.mouse.senseWalls()
        print(self.mouse.getCurrentCell().getWhichIsWall())
        sendData = {'robot': self.whoami, 'x': self.mouse.x, 'y': self.mouse.y, 'up': not self.mouse.canGoUp(
        ), 'down': not self.mouse.canGoDown(), 'left': not self.mouse.canGoLeft(), 'right': not self.mouse.canGoRight()}
        print(sendData)
        print(self.network.sendStringData(sendData))


        recvData = self.network.retrieveData()
        print("recvData: %s"% recvData)
        # one packet at a time
        while recvData:

            otherMap = recvData
            cell = self.mouse.mazeMap.getCell(otherMap['x'], otherMap['y'])
            self.isVisited[otherMap['x']][otherMap['y']] = 1
            self.neighbors_states[otherMap['robot']] = {'robot':otherMap['robot'], 'x': otherMap['x'], 'y': otherMap['y']} # update neighbors_states as received
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

        self.dx,self.dy = self.check_greatest_distance()
        print(self.dx)
        print(self.dy)
        # print("far bot direction: %s,%s"%(dx[0],dy[0]))

        #TODO If you want visited to be accurate it needs to be updated here
        if self.mouse.canGoLeft() and self.dx[0] < 0 and not self.isVisited[self.mouse.x-1][self.mouse.y]:
            self.path.append([self.mouse.x,self.mouse.y])
            self.isVisited[self.mouse.x - 1][self.mouse.y] = 1
            self.mouse.goLeft()
            # whoami makes more sense with a cool id
            self.neighbors_states[self.whoami] = {'robot': self.whoami, 'x':self.mouse.x , 'y': self.mouse.y}
        elif self.mouse.canGoRight() and self.dx[0] > 0 and not self.isVisited[self.mouse.x+1][self.mouse.y]:
            self.path.append([self.mouse.x,self.mouse.y])
            self.isVisited[self.mouse.x + 1][self.mouse.y] = 1
            self.mouse.goRight()
            self.neighbors_states[self.whoami] = {'robot': self.whoami, 'x':self.mouse.x , 'y': self.mouse.y}
        elif self.mouse.canGoUp() and self.dy[0] > 0 and not self.isVisited[self.mouse.x][self.mouse.y-1]:
            self.path.append([self.mouse.x,self.mouse.y])
            self.isVisited[self.mouse.x][self.mouse.y-1] = 1
            self.mouse.goUp()
            self.neighbors_states[self.whoami] = {'robot': self.whoami, 'x':self.mouse.x , 'y': self.mouse.y}
        elif self.mouse.canGoDown() and self.dy[0] < 0 and not self.isVisited[self.mouse.x][self.mouse.y+1]:
            self.path.append([self.mouse.x,self.mouse.y])
            self.isVisited[self.mouse.x][self.mouse.y+1] = 1
            self.mouse.goDown()
            self.neighbors_states[self.whoami] = {'robot': self.whoami, 'x':self.mouse.x , 'y': self.mouse.y}

        # first see if the bot can go towards gradient
        else:
            print("in the last section")
            if self.mouse.canGoLeft() and not self.isVisited[self.mouse.x-1][self.mouse.y]:
                self.path.append([self.mouse.x, self.mouse.y])
                self.isVisited[self.mouse.x - 1][self.mouse.y] = 1
                self.mouse.goLeft()
                self.neighbors_states[self.whoami] = {'robot': self.whoami, 'x':self.mouse.x , 'y': self.mouse.y}
            elif self.mouse.canGoUp() and not self.isVisited[self.mouse.x][self.mouse.y-1]:
                self.path.append([self.mouse.x, self.mouse.y])
                self.isVisited[self.mouse.x][self.mouse.y - 1] = 1
                self.mouse.goUp()
                self.neighbors_states[self.whoami] = {'robot': self.whoami, 'x':self.mouse.x , 'y': self.mouse.y}
            elif self.mouse.canGoRight() and not self.isVisited[self.mouse.x+1][self.mouse.y]:
                self.path.append([self.mouse.x, self.mouse.y])
                self.isVisited[self.mouse.x + 1][self.mouse.y] = 1
                self.mouse.goRight()
            elif self.mouse.canGoDown() and not self.isVisited[self.mouse.x][self.mouse.y+1]:
                self.path.append([self.mouse.x, self.mouse.y])
                self.isVisited[self.mouse.x][self.mouse.y + 1] = 1
                self.mouse.goDown()
                self.neighbors_states[self.whoami] = {'robot': self.whoami, 'x':self.mouse.x , 'y': self.mouse.y}
            else: # if no gradient available, then backtrack
                if len(self.path) !=0:
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
            # if len(self.path)

        # if len(self.path) < 10: self.path = self.path[1:]

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

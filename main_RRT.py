import random
import math
import pygame

class RRTMap:
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        self.start = start
        self.goal = goal
        self.MapDims = MapDimensions
        self.MapH, self.MapW = self.MapDims
        
        self.MapWindowName = 'RRT path'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.MapW, self.MapH))
        self.map.fill((255, 255, 255))
        self.nodeRad = 2
        self.nodeThickness = 1
        self.edgeThickness = 1
        
        self.obstacles = []
        self.obsdim = obsdim
        self.obsnum = obsnum
        
        self.grey = (70, 70, 70)
        self.blue = (0, 0, 255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.white = (255, 255, 255)
        self.violet = (127, 0, 55)
        
    def drawMap(self, obstacles):
        pygame.draw.circle(self.map, self.green, self.start, self.nodeRad + 5, 0)
        pygame.draw.circle(self.map, self.red, self.goal, self.nodeRad + 10, 0)
        self.drawObstacles(obstacles)
        
    def drawPath(self, path):
        for vertice in path:
            pygame.draw.circle(self.map, self.violet, vertice, self.nodeRad + 3, 0)
        
    def drawObstacles(self, obstacles):
        obsList = obstacles.copy()
        while obsList:
            obstacle = obsList.pop()
            pygame.draw.rect(self.map, self.grey, obstacle)
        

class RRTGraph:
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        (x, y) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.MapH, self.MapW = MapDimensions
        
        self.X = [x]
        self.Y = [y]
        self.parent = [0]
        
        self.obstacles = []
        self.obsdim = obsdim
        self.obsnum = obsnum
        
        self.goalstate = None
        self.path = []
        
    def makeRandomRect(self):
        ucx = int(random.uniform(0, self.MapW - self.obsdim))
        ucy = int(random.uniform(0, self.MapH - self.obsdim))
        return (ucx, ucy)
    
    def makeObs(self):
        obs = []
        for i in range(self.obsnum):
            rectang = None
            startgoalcol = True
            while startgoalcol:
                upper = self.makeRandomRect()
                rectang = pygame.Rect(upper, (self.obsdim, self.obsdim))
                if (not rectang.collidepoint(self.start)) and (not rectang.collidepoint(self.goal)):
                    startgoalcol = False
            obs.append(rectang)
        self.obstacles = obs.copy()
        return obs
                    
    def addNode(self, n, x, y):
        self.X.insert(n, x)
        self.Y.insert(n, y)
        
    def removeNode(self, n):
        self.X.pop(n)
        self.Y.pop(n)
        
    def addEdge(self, parent, child):
        self.parent.insert(child, parent)
        
    def removeEdge(self, n):
        self.parent.pop()
        
    def numOfNodes(self):
        return len(self.X)
    
    def dist(self, n1, n2):
        x1, y1 = self.X[n1], self.Y[n1]
        x2, y2 = self.X[n2], self.Y[n2]
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def sample_env(self):
        x = int(random.uniform(0, self.MapW))
        y = int(random.uniform(0, self.MapH))
        return x, y
    
    def nearest(self, n):
        dmin = self.dist(0, n)
        indnear = 0
        for i in range(n):
            if self.dist(i, n) < dmin:
                dmin = self.dist(i, n)
                indnear = i
        return indnear
                
    def isFree(self):
        n = self.numOfNodes() - 1
        x, y = self.X[n], self.Y[n]
        for obst in self.obstacles:
            if obst.collidepoint(x, y):
                self.removeNode(n)
                return False
        return True
    
    def crossObstacle(self, x1, x2, y1, y2):
        for obst in self.obstacles:
            if obst.clipline((x1, y1), (x2, y2)):
                return True
        return False
    
    def connect(self, n1, n2):
        x1, y1 = self.X[n1], self.Y[n1]
        x2, y2 = self.X[n2], self.Y[n2]
        if self.crossObstacle(x1, x2, y1, y2):
            self.removeNode(n2)
            return False
        else:
            self.addEdge(n1, n2)
            return True
        
    def step(self, indnear, nrand, dmax = 35):
        d = self.dist(indnear, nrand)
        xnear, ynear = self.X[indnear], self.Y[indnear]
        xrand, yrand = self.X[nrand], self.Y[nrand]
        px, py = xrand - xnear, yrand - ynear
        if d > dmax:
            u = dmax / d
            A, B = xnear + px * u, ynear + py * u
        else:
            xrand, yrand = self.X[nrand], self.Y[nrand]
            A, B = xrand, yrand
        self.removeNode(nrand)
        self.addNode(nrand, A, B)
            
    def path_to_goal(self):
        if self.goalFlag:
            self.path = [self.goalstate]
            newpos = self.parent[self.goalstate]
            while newpos != 0:
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(newpos)
        return self.goalFlag
            
    def getPathCoords(self):
        PathCoords = []
        for vertice in self.path:
            x, y = self.X[vertice], self.Y[vertice]
            PathCoords.append((x, y))
        return PathCoords
    
    def bias(self, ngoal):
        n = self.numOfNodes()
        self.addNode(n, ngoal[0], ngoal[1])
        indnear = self.nearest(n)
        self.step(indnear, n)
        if self.connect(indnear, n):
            if (abs(self.X[n] - self.goal[0]) < 5) and (abs(self.Y[n] - self.goal[1]) < 5):
                self.goalstate = n
                self.goalFlag = True
        return self.X, self.Y, self.parent
    
    def expand(self):
        n = self.numOfNodes()
        x, y = self.sample_env()
        self.addNode(n, x, y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest, n)
            if self.connect(xnearest, n):
                if (abs(self.X[n] - self.goal[0]) < 5) and (abs(self.Y[n] - self.goal[1]) < 5):
                    self.goalstate = n
                    self.goalFlag = True
        return self.X, self.Y, self.parent
    
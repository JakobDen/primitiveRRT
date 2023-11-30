import random
import math
import pygame

class ManipMap:
    def __init__(self, position, length, start_angles, goal, MapDimensions, obsdim, obsnum):
        self.MapDims = MapDimensions
        self.MapH, self.MapW = self.MapDims
        self.position = position
        self.x, self.y = position
        self.l1, self.l2 = length
        self.theta1, self.theta2 = start_angles
        self.X_goal, self.Y_goal = goal
        
        self.MapWindowName = 'MyManipulator'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.MapW, self.MapH))
        self.map.fill((255, 255, 255))
        
        self.nodeRad = 2
        self.nodeThickness = 1
        self.edgeThickness = 2
        
        self.obstacles = []
        self.obsdim = obsdim
        self.obsnum = obsnum
        
        self.grey = (70, 70, 70)
        self.blue = (0, 0, 255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.white = (255, 255, 255)
        self.violet = (127, 0, 55)
        
    def first_appearance(self):
        pygame.draw.circle(self.map, self.violet, self.position, self.nodeRad + 5, 0)
        self.draw_manip(self.theta1, self.theta2)
        pygame.draw.circle(self.map, self.red, (self.X_goal, self.Y_goal), self.nodeRad + 5, 0)
        
    def draw_manip(self, theta1, theta2):
        line1_coords = [self.position, [self.x + self.l1 * math.cos(theta1), self.y + self.l1 * math.sin(theta1)]]
        line2_coords = [line1_coords[1], [line1_coords[1][0] + self.l2 * math.cos(theta2 + theta1), line1_coords[1][1] + self.l2 * math.sin(theta2 + theta1)]]
        pygame.draw.line(self.map, self.blue, line1_coords[0], line1_coords[1], self.edgeThickness)
        pygame.draw.line(self.map, self.grey, line2_coords[0], line2_coords[1], self.edgeThickness)
        
    def drawObstacles(self, obstacles):
        obsList = obstacles.copy()
        while obsList:
            obstacle = obsList.pop()
            pygame.draw.rect(self.map, self.grey, obstacle)
        
        
class ManipGraph:
    def __init__(self, position, length, start_angles, goal, MapDimensions, obsdim, obsnum):
        self.MapDims = MapDimensions
        self.MapH, self.MapW = self.MapDims
        self.position = position
        self.x, self.y = position
        self.L1, self.L2 = length
        self.theta1, self.theta2 = start_angles
        self.X_goal, self.Y_goal = goal
        self.theta1_goal, self.theta2_goal = self.coord_to_angles(self.X_goal - self.x, self.Y_goal - self.y)
        
        self.goal = goal
        self.goalFlag = False
        self.MapH, self.MapW = MapDimensions
        
        self.T1 = [self.theta1]
        self.T2 = [self.theta2]
        self.parent = [0]
        
        self.obstacles = []
        self.obsdim = obsdim
        self.obsnum = obsnum
        
        self.goalstate = None
        self.path = []
        
    def coord_to_angles(self, x, y):
        eps = 0.0001
        theta1 = math.atan2(y, (x + eps)) - math.acos((self.L1 ** 2 + x ** 2 + y ** 2 - self.L2 ** 2)/(2 * self.L1 * math.sqrt(x ** 2 + y ** 2)))
        theta2 = math.atan2((y - self.L1 * math.sin(theta1)), (x - self.L1 * math.cos(theta1))) - theta1
        return theta1, theta2
    
    def angles_to_coord(self, theta1, theta2):
        x = self.L1 * math.cos(theta1) + self.L2 * math.cos(theta1 + theta2)
        y = self.L1 * math.sin(theta1) + self.L2 * math.sin(theta1 + theta2)
        return x, y
          
    def addNode(self, n, x, y):
        self.T1.insert(n, x)
        self.T2.insert(n, y)
        
    def removeNode(self, n):
        self.T1.pop(n)
        self.T2.pop(n)
        
    def addEdge(self, parent, child):
        self.parent.insert(child, parent)
        
    def removeEdge(self, n):
        self.parent.pop()
        
    def numOfNodes(self):
        return len(self.T1)
    
    def dist(self, n1, n2):
        theta1, theta2 = self.T1[n1], self.T2[n1]
        theta3, theta4 = self.T1[n2], self.T2[n2]
        d1 = math.sqrt((theta1 - (theta3)) ** 2 + (theta2 - (theta4)) ** 2)
        d2 = math.sqrt((theta1 - (theta3 - 2 * math.pi)) ** 2 + (theta2 - (theta4)) ** 2)
        d3 = math.sqrt((theta1 - (theta3 + 2 * math.pi)) ** 2 + (theta2 - (theta4)) ** 2)
        d4 = math.sqrt((theta1 - (theta3)) ** 2 + (theta2 - (theta4 - 2 * math.pi)) ** 2)
        d5 = math.sqrt((theta1 - (theta3)) ** 2 + (theta2 - (theta4 + 2 * math.pi)) ** 2)
        real_dist = min([d1, d2, d3, d4, d5])
        if d1 == real_dist:
            return d1, (theta3 - theta1, theta4 - theta2)
        elif d2 == real_dist:
            return d2, (theta3 - 2 * math.pi - theta1, theta4 - theta2)
        elif d3 == real_dist:
            return d3, (theta3 + 2 * math.pi - theta1, theta4 - theta2)
        elif d4 == real_dist:
            return d4, (theta3 - theta1, theta4 - 2 * math.pi - theta2)
        else:
            return d5, (theta3 - theta1, theta4 + 2 * math.pi - theta2)

    def sample_env(self):
        x = int(random.uniform(0, 2 * math.pi))
        y = int(random.uniform(0, 2 * math.pi))
        return x, y
    
    def nearest(self, n):
        dmin, vec_towards = self.dist(0, n)
        indnear = 0
        for i in range(n):
            if self.dist(i, n)[0] < dmin:
                dmin = self.dist(i, n)[0]
                indnear = i
        return indnear
                
    def connect(self, n1, n2):
        x1, y1 = self.T1[n1], self.T2[n1]
        x2, y2 = self.T1[n2], self.T2[n2]
        dt1, dt2 = x2 - x1, y2 - y1
        for i in range(1, 101):
            line_1_c, line_2_c = self.manip_lines(x1 + (i / 100) * dt1, y1 + (i / 100) * dt2)
            if self.crossObstacle(*line_1_c):
                self.removeNode(n2)
                return False
            if self.crossObstacle(*line_2_c):
                self.removeNode(n2)
                return False
        self.addEdge(n1, n2)
        return True
        
    def step(self, indnear, nrand, dmax = 0.1):
        d, (p1, p2) = self.dist(indnear, nrand)
        xnear, ynear = self.T1[indnear], self.T2[indnear]
        if d > dmax:
            u = dmax / d
            A, B = xnear + p1 * u, ynear + p2 * u
        else:
            xrand, yrand = self.T1[nrand], self.T2[nrand]
            A, B = xrand, yrand
        self.removeNode(nrand)
        self.addNode(nrand, A, B)
            
    def bias(self, ngoal):
        n = self.numOfNodes()
        self.addNode(n, ngoal[0], ngoal[1])
        indnear = self.nearest(n)
        self.step(indnear, n)
        if self.connect(indnear, n):
            xpos, ypos = self.angles_to_coord(self.T1[n], self.T2[n])
            if (abs(self.position[0] + xpos - self.X_goal) < 5) and (abs(self.position[1] + ypos - self.Y_goal) < 5):
                self.goalstate = n
                self.goalFlag = True
        return self.T1, self.T2, self.parent
    
    def expand(self):
        n = self.numOfNodes()
        x, y = self.sample_env()
        self.addNode(n, x, y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest, n)
            if self.connect(xnearest, n):
                xpos, ypos = self.angles_to_coord(self.T1[n], self.T2[n])
                if (abs(self.position[0] + xpos - self.X_goal) < 7) and (abs(self.position[1] + ypos - self.Y_goal) < 7):
                    self.goalstate = n
                    self.goalFlag = True
        return self.T1, self.T2, self.parent
    
    def makeRandomRect(self):
        ucx = int(random.uniform(0, self.MapW - self.obsdim))
        ucy = int(random.uniform(0, self.MapH - self.obsdim))
        return (ucx, ucy)
    
    def makeObs(self):
        obs = []
        line1_coords, line2_coords = self.manip_lines(self.theta1, self.theta2)
        for i in range(self.obsnum):
            rectang = None
            startgoalcol = True
            while startgoalcol:
                upper = self.makeRandomRect()
                rectang = pygame.Rect(upper, (self.obsdim, self.obsdim))
                if (not rectang.clipline(*line1_coords)) and (not rectang.clipline(*line2_coords)) and (not rectang.collidepoint(self.goal)):
                    startgoalcol = False
            obs.append(rectang)
        self.obstacles = obs.copy()
        return obs
    
    def isFree(self):
        n = self.numOfNodes() - 1
        x, y = self.angles_to_coord(self.T1[n], self.T2[n])
        for obst in self.obstacles:
            if obst.collidepoint(x, y):
                self.removeNode(n)
                return False
        return True
    
    def crossObstacle(self, x1, y1, x2, y2):
        for obst in self.obstacles:
            if obst.clipline((x1, y1), (x2, y2)):
                return True
        return False
    
    def manip_lines(self, theta1, theta2):
        line1_coords = [self.position[0], self.position[1], self.x + self.L1 * math.cos(theta1), self.y + self.L1 * math.sin(theta1)]#x1y1x2y2
        line2_coords = [line1_coords[2], line1_coords[3],line1_coords[2] + self.L2 * math.cos(theta2 + theta1), line1_coords[3] + self.L2 * math.sin(theta2 + theta1)]
        return line1_coords, line2_coords
    
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
    
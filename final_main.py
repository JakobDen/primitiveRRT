import pygame
import random
import math
from final import ManipMap, ManipGraph
from final import RRTGraph, RRTMap

def main():
    dimensions = (600, 1000)
    start_position = (500, 300)
    lengths = (50, 25)
    start_angles = (random.uniform(0, 2 * math.pi), math.pi)
    goal_position = (490, 350)
    obssize = 30
    obsnum = 80
    start = (50, 50)
    
    pygame.init()
    
    Manip = ManipMap(start_position, lengths, start_angles, goal_position ,dimensions, obssize, obsnum)
    MGraph = ManipGraph(start_position, lengths, start_angles, goal_position,dimensions, obssize, obsnum)
    
    goal = (MGraph.theta1_goal, MGraph.theta2_goal)
    obstacles = MGraph.makeObs()
    Manip.drawObstacles(obstacles)
    
    Map = RRTMap(start, (500, 300), dimensions, obssize, obsnum)
    Graph = RRTGraph(start, (500, 300), dimensions, obssize, obsnum)
    Map.obstacles = obstacles
    Map.drawMap(obstacles)
    Graph.obstacles = obstacles
    j = 0
    while not Graph.path_to_goal():
        if j % 10 == 0:
            X, Y, Parent = Graph.bias(start_position)
            pygame.draw.circle(Map.map, Map.grey, (Graph.X[-1], Graph.Y[-1]), Map.nodeRad + 2, 0)
            pygame.draw.line(Map.map, Map.blue, (Graph.X[-1], Graph.Y[-1]), (Graph.X[Parent[-1]], Graph.Y[Parent[-1]]), Map.edgeThickness)
        else:
            X, Y, Parent = Graph.expand()
            pygame.draw.circle(Map.map, Map.grey, (Graph.X[-1], Graph.Y[-1]), Map.nodeRad + 2, 0)
            pygame.draw.line(Map.map, Map.blue, (Graph.X[-1], Graph.Y[-1]), (Graph.X[Parent[-1]], Graph.Y[Parent[-1]]), Map.edgeThickness)
        
        if (Graph.X[Parent[-1]], Graph.Y[Parent[-1]]) == start_position:
            print(Graph.goalFlag)
            Graph.goalFlag = True
            Graph.goalstate = Graph.numOfNodes()
            break
        j += 1
        pygame.display.update()
        pygame.event.clear()
            
    Map.drawPath(Graph.getPathCoords())
    pygame.display.update()
    pygame.event.clear()
    Manip.first_appearance()
    j = 0
    while (not MGraph.goalFlag) and j < 1100:
        if j % 5 == 0:
            _, _, Parent = MGraph.bias(goal)
            Manip.draw_manip(MGraph.T1[-1], MGraph.T2[-1])
        else:
            _, _, Parent = MGraph.expand()
            Manip.draw_manip(MGraph.T1[-1], MGraph.T2[-1])
        if (MGraph.T1[Parent[-1]], MGraph.T2[Parent[-1]]) == goal:
            print(MGraph.goalFlag)
            MGraph.goalFlag = True
            MGraph.goalstate = MGraph.numOfNodes()
            break

        j += 1
        pygame.display.update()
        pygame.event.clear()
    if MGraph.goalFlag:
        print('Hooray!!!')

    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)
    
main()

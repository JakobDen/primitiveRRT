import pygame
import random
import math
from manip2d_main import ManipMap, ManipGraph

def main():
    dimensions = (600, 1000)
    start_position = (500, 300)
    lengths = (50, 25)
    start_angles = (random.uniform(0, 2 * math.pi), math.pi)
    goal_position = (490, 350)
    obssize = 30
    obsnum = 80
    
    pygame.init()
    Manip = ManipMap(start_position, lengths, start_angles, goal_position ,dimensions, obssize, obsnum)
    Graph = ManipGraph(start_position, lengths, start_angles, goal_position ,dimensions, obssize, obsnum)
    goal = (Graph.theta1_goal, Graph.theta2_goal)
    obstacles = Graph.makeObs()
    Manip.drawObstacles(obstacles)
    j = 0
    while (not Graph.goalFlag) and j < 1100:
        if j % 5 == 0:
            _, _, Parent = Graph.bias(goal)
            Manip.draw_manip(Graph.T1[-1], Graph.T2[-1])
        else:
            _, _, Parent = Graph.expand()
            Manip.draw_manip(Graph.T1[-1], Graph.T2[-1])
        if (Graph.T1[Parent[-1]], Graph.T2[Parent[-1]]) == goal:
            print(Graph.goalFlag)
            Graph.goalFlag = True
            Graph.goalstate = Graph.numOfNodes()
            break

        j += 1
        pygame.display.update()
        pygame.event.clear()
    if Graph.goalFlag:
        print('Hooray!!!')

    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)
    
main()


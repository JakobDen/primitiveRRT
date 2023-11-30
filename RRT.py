import pygame
from main_RRT import RRTGraph, RRTMap

def main():
    dimensions = (600, 1000)
    start = (50, 50)
    goal = (510, 510)
    obssize = 30
    obsnum = 400
    
    pygame.init()
    Map = RRTMap(start, goal, dimensions, obssize, obsnum)
    Graph = RRTGraph(start, goal, dimensions, obssize, obsnum)
    
    obstacles = Graph.makeObs()
    Map.drawMap(obstacles)
    j = 0
    while not Graph.path_to_goal():
        if j % 10 == 0:
            X, Y, Parent = Graph.bias(goal)
            pygame.draw.circle(Map.map, Map.grey, (Graph.X[-1], Graph.Y[-1]), Map.nodeRad + 2, 0)
            pygame.draw.line(Map.map, Map.blue, (Graph.X[-1], Graph.Y[-1]), (Graph.X[Parent[-1]], Graph.Y[Parent[-1]]), Map.edgeThickness)
        else:
            X, Y, Parent = Graph.expand()
            pygame.draw.circle(Map.map, Map.grey, (Graph.X[-1], Graph.Y[-1]), Map.nodeRad + 2, 0)
            pygame.draw.line(Map.map, Map.blue, (Graph.X[-1], Graph.Y[-1]), (Graph.X[Parent[-1]], Graph.Y[Parent[-1]]), Map.edgeThickness)
        
        if (Graph.X[Parent[-1]], Graph.Y[Parent[-1]]) == goal:
            print(Graph.goalFlag)
            Graph.goalFlag = True
            Graph.goalstate = Graph.numOfNodes()
            break
        j += 1
        pygame.display.update()
            
    Map.drawPath(Graph.getPathCoords())
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)
    

main()


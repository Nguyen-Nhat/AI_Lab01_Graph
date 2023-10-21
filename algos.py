import pygame
import time
from maze import SearchSpace
from const import * 
import queue
import math
def DFS_helper(g:SearchSpace, sc:pygame.Surface, start, closed_set,fathers):
    temp = g.grid_cells[start]
    temp.set_color(YELLOW,sc)
    closed_set.add(start)
    temp.set_color(BLUE,sc)
    if(g.is_goal(temp)):
        g.start.set_color(ORANGE,sc)
        temp.set_color(PURPLE, sc)
        while fathers[start] != -1:
            current_center = g.grid_cells[start].rect.center
            father_center = g.grid_cells[fathers[start]].rect.center
            pygame.draw.line(sc, WHITE, current_center, father_center,2)
            start = fathers[start]
            g.start.set_color(ORANGE,sc)
        return True
    neighbors = g.get_neighbors(temp)
    for neighbor in neighbors: 
        if neighbor.id not in closed_set:
            fathers[neighbor.id] = start
            if DFS_helper(g,sc,neighbor.id, closed_set, fathers): 
                return True
    return False
def DFS(g: SearchSpace, sc: pygame.Surface):
    open_set = [g.start.id]
    closed_set = set() 
    fathers = [-1]*g.get_length()
    DFS_helper(g,sc,g.start.id,closed_set, fathers) 
def BFS(g: SearchSpace, sc: pygame.Surface):
    open_set = [{g.start.id, -1}]
    closed_set = set() 
    fathers = [-1]*g.get_length()
    while len(open_set) > 0 :
        id,father = open_set.pop(0)
        if id not in closed_set: 
            fathers[id] = father
            temp = g.grid_cells[id]
            temp.set_color(YELLOW, sc)
            closed_set.add(id)
            temp.set_color(BLUE,sc)
            if g.is_goal(temp):
                g.start.set_color(ORANGE,sc)
                temp.set_color(PURPLE, sc)
                while fathers[id] != -1:
                    current_center = g.grid_cells[id].rect.center
                    father_center = g.grid_cells[fathers[id]].rect.center
                    pygame.draw.line(sc, WHITE, current_center, father_center,2)
                    id = fathers[id]
                    g.start.set_color(ORANGE,sc)
                return
            neighbors = g.get_neighbors(temp)
            for neighbor in neighbors: 
                open_set.append((neighbor.id,id))
                if neighbor.id not in closed_set:
                    neighbor.set_color(RED,sc)
def calculateGValue(idParent, id) -> float:
    xParent, yParent = idParent // COLS, idParent % COLS
    x,y = id // COLS, id//COLS
    dx = abs(x - xParent)
    dy = abs(y - yParent)
    if dx + dy == 1: 
        return 1
    else:
        return math.sqrt(2) 
def UCS(g: SearchSpace, sc: pygame.Surface):
    open_set = queue.PriorityQueue()
    open_set.put((0, g.start.id))
    closed_set = set() 
    fathers = [-1]*g.get_length()
    cost = [100_000]*g.get_length()
    cost[g.start.id] = 0
    while not open_set.empty():
        distance, id = open_set.get()
        if id in closed_set:
            continue
        node = g.grid_cells[id]
        node.set_color(YELLOW, sc)
        closed_set.add(id)
        node.set_color(BLUE,sc)
        if g.is_goal(node):
            g.start.set_color(ORANGE,sc)
            node.set_color(PURPLE, sc)
            while fathers[id] != -1:
                current_center = g.grid_cells[id].rect.center
                father_center = g.grid_cells[fathers[id]].rect.center
                pygame.draw.line(sc, WHITE, current_center, father_center,2)
                id = fathers[id]
                g.start.set_color(ORANGE,sc)
            return
        neighbors = g.get_neighbors(node)
        for neighbor in neighbors:
            d = calculateGValue(node.id, neighbor.id)
            if(cost[neighbor.id] > cost[id] + d): 
                cost[neighbor.id] = cost[id] + d
                open_set.put((cost[neighbor.id], neighbor.id))
                fathers[neighbor.id] = id
                neighbor.set_color(RED,sc)
               
def calculateHValue(idNow, idDest) -> float: 
    xNow,yNow = idNow // COLS, idNow%COLS
    xDest,yDest = idDest// COLS, idDest%COLS
    return math.sqrt((xNow - xDest) * (xNow - xDest) + (yNow - yDest) * (yNow - yDest))
def AStar(g: SearchSpace, sc: pygame.Surface):
    open_set = queue.PriorityQueue()
    open_set.put((calculateHValue(g.start.id, g.goal.id), g.start.id))
    closed_set = set()
    fathers = [-1]*g.get_length()
    cost = [100_000]*g.get_length()
    cost[g.start.id] = calculateHValue(g.start.id, g.goal.id)
    while not open_set.empty() : 
        distance, id = open_set.get()
        if id in closed_set: 
            continue
        node = g.grid_cells[id]
        node.set_color(YELLOW,sc)
        closed_set.add(id)
        node.set_color(BLUE,sc)
        if g.is_goal(node):
            g.start.set_color(ORANGE,sc)
            node.set_color(PURPLE, sc)
            while fathers[id] != -1:
                current_center = g.grid_cells[id].rect.center
                father_center = g.grid_cells[fathers[id]].rect.center
                pygame.draw.line(sc, WHITE, current_center, father_center,2)
                id = fathers[id]
                g.start.set_color(ORANGE,sc)
            return
        neighbors = g.get_neighbors(node)
        for neighbor in neighbors:
            d = calculateGValue(node.id, neighbor.id) 
            temp = cost[id] - calculateHValue(id,g.goal.id) + d + calculateHValue(neighbor.id, g.goal.id)
            if(cost[neighbor.id] > temp):
                cost[neighbor.id] = temp
                open_set.put((temp,neighbor.id))
                fathers[neighbor.id] = id
                neighbor.set_color(RED,sc)
            
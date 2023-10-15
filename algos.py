import pygame
import time
from maze import SearchSpace
from const import * 
import queue
import math
def DFS_helper(g:SearchSpace, sc:pygame.Surface, start, closed_set,fathers):
    temp = g.grid_cells[start]
    temp.set_color(BLUE, sc) 
    if(g.is_goal(temp)):
        temp.set_color(GREEN, sc)
        id = start 
        while fathers[id] != -1: 
            g.grid_cells[fathers[id]].set_color(GREEN,sc)
            id = fathers[id]
        return True
    neighbors = g.get_neighbors(temp)
    for neighbor in neighbors: 
        if neighbor.id not in closed_set:
            closed_set.add(start)
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
            closed_set.add(id)
            temp = g.grid_cells[id]
            temp.set_color(BLUE,sc)
            if g.is_goal(temp):
                temp.set_color(GREEN,sc)
                while fathers[id] != -1:
                    g.grid_cells[fathers[id]].set_color(GREEN,sc)
                    id = fathers[id]
                break
            neighbors = g.get_neighbors(temp)
            for neighbor in neighbors: 
                open_set.append((neighbor.id,id))
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
        closed_set.add(id)
        node = g.grid_cells[id]
        node.set_color(BLUE,sc)
        if g.is_goal(node):
            node.set_color(GREEN, sc)
            while fathers[id] != -1: 
                g.grid_cells[fathers[id]].set_color(GREEN,sc)
                id = fathers[id]
            return
        neighbors = g.get_neighbors(node)
        for neighbor in neighbors:
            if(cost[neighbor.id] > cost[id] + 1): 
                cost[neighbor.id] = cost[id] + 1
                open_set.put((cost[neighbor.id], neighbor.id))
                fathers[neighbor.id] = id
               
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
        closed_set.add(id)
        node = g.grid_cells[id]
        node.set_color(BLUE,sc)
        if g.is_goal(node): 
            node.set_color(GREEN, sc)
            while fathers[id] != -1: 
                g.grid_cells[fathers[id]].set_color(GREEN,sc)
                id = fathers[id]
            return
        neighbors = g.get_neighbors(node)
        for neighbor in neighbors: 
            temp = cost[id] - calculateHValue(id,g.goal.id) + 1 + calculateHValue(neighbor.id, g.goal.id)
            if(cost[neighbor.id] > temp):
                cost[neighbor.id] = temp
                open_set.put((temp,neighbor.id))
                fathers[neighbor.id] = id
            
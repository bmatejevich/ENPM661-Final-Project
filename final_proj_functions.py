#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 26 18:17:19 2020

@author: brianmatejevich
"""
import numpy as np
import math

def get_min_node(queue):
    min_node = 0
    for node in range(len(queue)):
        if queue[node].cost_for_pred < queue[min_node].cost_for_pred:
            min_node = node
    return queue.pop(min_node)

def node_exists(x,y, queue):
    for node in queue:
        if node.x == x and node.y == y:
            return queue.index(node)
        else:
            return None
        
def try_move(move, current_point):
    if move == 'move_up':
        return move_up(current_point)
    if move == 'move_down':
        return move_down(current_point)
    if move == 'move_left':
        return move_left(current_point)
    if move == 'move_right':
        return move_right(current_point)
    if move == 'move_up_right':
        return move_up_right(current_point)
    if move == 'move_up_left':
        return move_up_left(current_point)
    if move == 'move_down_right':
        return move_down_right(current_point)
    if move == 'move_down_left':
        return move_down_left(current_point)

def ways_in(x,y): # a pixel with no obstacles or edges nearby can be achieved from 8 moves
    count = 0
    if y > 0: #from top
        count+=1
    if y < 200: #from bottom
        count+=1
    if x > 0: #from left
        count+=1
    if x < 200: #from right
        count+=1
    if x < 200 and y < 200: #bottom right
        count+=1
    if x < 200 and y > 0: #top left
        count+=1
    if x > 0 and y > 0: #top left
        count+=1
    if x > 0 and y < 200: #bottom right
        count+=1
    return count

def fill_pixel(img,x,y): #fill visited pixes
    img[y,x] = [255,0,0]
    return img

def backtrack(node): #create list of parent node locations
    parentList = list()
    parent = node.parent
    while parent is not None:
        parentList.append(parent)
        parent = parent.parent
    return parentList


def check_viableX(point):
    if point >= 0 and point < 200:
        return True
    else:
        print("Invalid")
        print()
        return False
    
def check_viableY(point):
    if point >= 0 and point < 200:
        return True
    else:
        print("Invalid")
        print()
        return False


def check_distance(current_point,new_point):
    x1 = current_point[0]
    y1 = current_point[1]
    x2 = new_point[0]
    y2 = new_point[1]
    d = np.sqrt((x1-x2)**2+(y1-y2)**2)
    if d <= 1* np.sqrt(2):
        #print("in range")
        return True
    else:
        #print("too far")
        return False

def get_cost_to_go(start,goal):
    x1 = start[0]
    x2 = goal[0]
    y1 = start[1]
    y2 = goal[1]
    dist = math.sqrt(((x1-x2)**2)+((y1-y2)**2))
    return dist

def increment(cost_map,agent_type):
    if agent_type == "pred":
        cost_map +=1
        cost_map = np.clip(cost_map, 0, 255)
    if agent_type == "prey":
        cost_map -=1
        cost_map = np.clip(cost_map, 0, 255)
    return cost_map

def plot_workspace(x_start,y_start,x_goal,y_goal):
    img = 255 * np.ones((200, 200, 3), np.uint8)
    img[y_start,x_start] = [0,255,0]
    img[y_goal,x_goal] = [0,0,0]
    return img

def move_up(point):
    x = point[0]
    y = point[1]
    cost = 1
    if check_viableX(x) and check_viableY(y-1):
        new_point = [x, y - 1]
        return new_point, cost
    else:
        return None, None


def move_down(point):
    x = point[0]
    y = point[1]
    cost = 1
    if check_viableX(x) and check_viableY(y+1):
        new_point = [x, y + 1]
        return new_point, cost
    else:
        return None, None


def move_left(point):
    x = point[0]
    y = point[1]
    cost = 1
    if check_viableX(x-1) and check_viableY(y):
        new_point = [x - 1, y]
        return new_point, cost
    else:
        return None, None


def move_right(point):
    x = point[0]
    y = point[1]
    cost = 1
    if check_viableX(x+1) and check_viableY(y):
        new_point = [x + 1, y]
        return new_point, cost
    else:
        return None, None


def move_up_right(point):
    x = point[0]
    y = point[1]
    cost = np.sqrt(2)
    if check_viableX(x+1) and check_viableY(y-1):
        new_point = [x + 1, y - 1]
        return new_point, cost
    else:
        return None, None


def move_up_left(point):
    x = point[0]
    y = point[1]
    cost = np.sqrt(2)
    if check_viableX(x-1) and check_viableY(y-1):
        new_point = [x - 1, y - 1]
        return new_point, cost
    else:
        return None, None


def move_down_right(point):
    x = point[0]
    y = point[1]
    cost = np.sqrt(2)
    if check_viableX(x+1) and check_viableY(y+1):
        new_point = [x + 1, y + 1]
        return new_point, cost
    else:
        return None, None


def move_down_left(point):
    x = point[0]
    y = point[1]
    cost = np.sqrt(2)
    if check_viableX(x-1) and check_viableY(y+1):
        new_point = [x - 1, y + 1]
        return new_point, cost
    else:
        return None, None

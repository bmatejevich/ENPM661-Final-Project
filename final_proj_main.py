#!/usr/bin/env python
# coding: utf-8
'''

'''
import math
import cv2
import time as t


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost_for_pred = math.inf
        self.parent = None

class Robot:
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal
        



def explore(image, robot):

    
    start_node_pos = robot.start
    goal_node_pos = robot.goal
    
    image[start_node_pos[1], start_node_pos[0]] = [0, 255, 0]
    image[goal_node_pos[1], goal_node_pos[0]] = [0, 0, 255]
    
    start_node = Node(start_node_pos[0],start_node_pos[1])
    start_node.cost_for_pred = 0

    waysIn = ways_in(goal_node_pos[0],goal_node_pos[1])
    print("Ways in", waysIn)
    
    visitedNodes = list()
    queue = [start_node]
    step_queue = []
    
    moves = ["move_up", "move_down", "move_left", "move_right",
               "move_up_right", "move_down_right", "move_up_left", "move_down_left"]
    counter = 0
    frame = 0
    
    while queue:
        current_node = get_min_node(queue)
        current_point = [current_node.x,current_node.y]
        visitedNodes.append(str(current_point))

        for move in moves:
            new_point, cost = try_move(move, current_point)
            frame +=1
            if new_point is not None and check_distance(start_node_pos,new_point):
                if new_point == goal_node_pos:
                    
                    if counter < waysIn:
                        counter += 1
                        print("Goal reached " +str(counter) + " times")

                new_node = Node(new_point[0],new_point[1])
                new_node.parent = current_node

                image = fill_pixel(image, current_node.x,current_node.y)
                image[start_node_pos[1], start_node_pos[0]] = [0, 255, 0]
                image[goal_node_pos[1], goal_node_pos[0]] = [0, 0, 255]

                # update display every 1 nodes explored
                if frame % 1 == 0:
                    img = image
                    scale_percent = 200 # percent of original size
                    width = int(img.shape[1] * scale_percent / 100)
                    height = int(img.shape[0] * scale_percent / 100)
                    dim = (width, height)
                    # resize image
                    img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
                    cv2.imshow("Map", img)
                    cv2.waitKey(1)
                
                if str(new_point) not in visitedNodes:
                    new_node.cost_for_pred = cost + new_node.parent.cost_for_pred
                    visitedNodes.append(str(new_point))
                    queue.append(new_node)
                    step_queue.append(new_node)
                else:
                    node_exist_index = node_exists(new_point[0],new_point[0], queue)
                    step_index = node_exists(new_point[0],new_point[0], step_queue)
                    if node_exist_index is not None:
                        temp_node = queue[node_exist_index]
                        if temp_node.cost_for_pred > cost + new_node.parent.cost_for_pred:
                            temp_node.cost_for_pred = cost + new_node.parent.cost_for_pred
                            temp_node.parent = current_node
                    if step_index is not None:
                        temp_step_node = step_queue[node_exist_index]
                        if temp_node.cost_for_pred > cost + new_node.parent.cost_for_pred:
                            temp_step_node.cost_for_pred = cost + new_node.parent.cost_for_pred
                            temp_step_node.parent = current_node
            else:
                continue
        if counter == waysIn:
            return new_node.parent, image
    min_node = get_min_node(step_queue)
    print(min_node.x,min_node.y+3)
    return min_node, None

#################################################
start = False
goal = False

#change these values for point/rigid robot
radius = 0
clearance = 0


while start == False:
    x_start = input("Enter robot x position : ")
    x_start = int(x_start)
    y_start = input("Enter robot y position : ") 
    y_start = 200 - int(y_start)-1
    start = check_viableY(y_start)
    if start == True:
        start = check_viableX(x_start)
       
    
while goal == False:
    x_goal = input("Enter goal x position : ") 
    x_goal = int(x_goal)
    y_goal = input("Enter goal y position : ") 
    y_goal = 200 - int(y_goal)-1
    goal = check_viableY(y_goal)
    if goal == True:
        goal = check_viableX(x_goal)
        




start = t.time()


start_node = [x_start,y_start]
goal_node = [x_goal,y_goal]



robot1 = Robot(start_node, goal_node)

workspace = plot_workspace(x_start,y_start,x_goal,y_goal)


solution, image = explore(workspace, robot1)

view_step = plot_workspace(x_start,y_start,solution.x,solution.y)
img = view_step
scale_percent = 200 # percent of original size
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)
# resize image
img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
cv2.imshow("Map", img)
cv2.waitKey(0)

print("Time to solve: " + str(t.time()-start) + " seconds")
if solution is not None:
    parent_list = backtrack(solution)
    for parent in parent_list:
        x = parent.x
        y = parent.y
        image[y, x] = [0, 255, 0]
        
        cv2.imshow("Map", image)
        cv2.waitKey(25)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("No path to goal point")





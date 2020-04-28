#!/usr/bin/env python
# coding: utf-8
import math
import cv2
import time as t
import numpy as np
from final_proj_functions import *


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost_for_pred = math.inf
        self.cost_for_prey = 0
        self.parent = None


class Robot:
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal


def pred_planner(image, robot, cost_for_pred_map):
    pred_node_pos = robot.start
    prey_node_pos = robot.goal

    image[pred_node_pos[1], pred_node_pos[0]] = [0, 255, 0]
    image[prey_node_pos[1], prey_node_pos[0]] = [0, 0, 255]

    start_node = Node(pred_node_pos[0], pred_node_pos[1])
    start_node.cost_for_pred = 0

    visitedNodes = list()
    queue = [start_node]
    step_queue = []

    moves = ["move_up", "move_down", "move_left", "move_right",
             "move_up_right", "move_down_right", "move_up_left", "move_down_left"]
    counter = 0
    frame = 0

    while queue:
        current_node = get_min_node(queue)
        current_point = [current_node.x, current_node.y]
        visitedNodes.append(str(current_point))

        for move in moves:
            new_point, cost_of_action = try_move(move, current_point)
            frame += 1

            if new_point is not None and check_distance(pred_node_pos, new_point) and check_distance(pred_node_pos,
                                                                                                     current_point):

                cost_for_pred = cost_for_pred_map[new_point[1]][new_point[0]]
                cost_to_go = get_cost_to_go(new_point, prey_node_pos)
                cost = cost_of_action + cost_for_pred + 10 * cost_to_go
                # print(new_point,prey_node_pos)
                if new_point == prey_node_pos:
                    counter += 1

                new_node = Node(new_point[0], new_point[1])
                new_node.parent = current_node

                image = fill_pixel(image, current_node.x, current_node.y)
                image[pred_node_pos[1], pred_node_pos[0]] = [0, 255, 0]
                image[prey_node_pos[1], prey_node_pos[0]] = [0, 0, 255]

                # update display every 1 nodes explored
                if frame % 1 == 0:
                    img = image
                    scale_percent = 200  # percent of original size
                    width = int(img.shape[1] * scale_percent / 100)
                    height = int(img.shape[0] * scale_percent / 100)
                    dim = (width, height)
                    # resize image
                    img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
                    cv2.imshow("Map", img)
                    cv2.waitKey(1)

                if str(new_point) not in visitedNodes:
                    new_node.cost_for_pred = cost + new_node.parent.cost_for_pred
                    visitedNodes.append(str(new_point))
                    queue.append(new_node)
                    step_queue.append(new_node)
                else:
                    node_exist_index = node_exists(new_point[0], new_point[0], queue)
                    step_index = node_exists(new_point[0], new_point[0], step_queue)
                    if node_exist_index is not None:
                        temp_node = queue[node_exist_index]
                        if temp_node.cost_for_pred > cost + new_node.parent.cost_for_pred:
                            temp_node.cost_for_pred = cost + new_node.parent.cost_for_pred
                            temp_node.parent = current_node
                    if step_index is not None:
                        print(node_exist_index)
                        temp_step_node = step_queue[step_index]
                        if temp_step_node.cost_for_pred > cost + new_node.parent.cost_for_pred:
                            temp_step_node.cost_for_pred = cost + new_node.parent.cost_for_pred
                            temp_step_node.parent = current_node
            else:
                continue
            if counter == 1:
                min_node = get_min_node(step_queue)
                print("current location " + str(min_node.x) + "," + str(200 - (min_node.y + 1)))
                print("Goal reached!")
                return min_node, True
    min_node = get_min_node(step_queue)
    print("current location " + str(min_node.x) + "," + str(200 - (min_node.y + 1)))
    return min_node, False

def prey_planner(image, robot, cost_for_prey_map):
    pred_node_pos = robot.goal
    prey_node_pos = robot.start

    image[prey_node_pos[1], prey_node_pos[0]] = [0, 0, 255]

    start_node = Node(prey_node_pos[0], prey_node_pos[1])
    start_node.cost_for_pred = 0

    visitedNodes = list()
    queue = [start_node]
    step_queue = []

    moves = ["move_up", "move_down", "move_left", "move_right",
             "move_up_right", "move_down_right", "move_up_left", "move_down_left"]
    counter = 0
    frame = 0

    while queue:
        current_node = get_min_node(queue)
        current_point = [current_node.x, current_node.y]
        visitedNodes.append(str(current_point))

        for move in moves:
            new_point, cost_of_action = try_move(move, current_point)
            frame += 1

            if new_point is not None and check_distance(prey_node_pos, new_point) and check_distance(prey_node_pos,
                                                                                                     current_point):

                cost_for_prey = cost_for_prey_map[new_point[1]][new_point[0]]
                cost_to_go = get_cost_to_go(new_point, (pred_node_pos.x, pred_node_pos.y))
                cost_p = cost_of_action +  cost_for_prey + 10 * cost_to_go

                # print(new_point,prey_node_pos)
                new_node = Node(new_point[0], new_point[1])
                new_node.parent = current_node

                if str(new_point) not in visitedNodes:
                    new_node.cost_for_prey = cost_p + new_node.parent.cost_for_prey
                    visitedNodes.append(str(new_point))
                    queue.append(new_node)
                    step_queue.append(new_node)
                else:
                    node_exist_index = node_exists(new_point[0], new_point[0], queue)
                    step_index = node_exists(new_point[0], new_point[0], step_queue)
                    if node_exist_index is not None:
                        temp_node = queue[node_exist_index]
                        if temp_node.cost_for_prey > cost_p + new_node.parent.cost_for_prey:
                            temp_node.cost_for_prey = cost_p + new_node.parent.cost_for_prey
                            temp_node.parent = current_node
                    if step_index is not None:
                        print(node_exist_index)
                        temp_step_node = step_queue[step_index]
                        if temp_step_node.cost_for_prey > cost_p + new_node.parent.cost_for_prey:
                            temp_step_node.cost_for_prey = cost_p + new_node.parent.cost_for_prey
                            temp_step_node.parent = current_node
            else:
                continue
            if counter == 1:
                min_node = get_min_node(step_queue)
                print("current location " + str(min_node.x) + "," + str(200 - (min_node.y + 1)))
                print("Goal reached!")
                return min_node
    max_node = get_max_node(step_queue)
    print("current location " + str(max_node.x) + "," + str(200 - (max_node.y + 1)))
    return max_node
#################################################
start = False
goal = False

# change these values for point/rigid robot
radius = 0
clearance = 0

# while start == False:
#     x_pred = input("Enter robot x position : ")
#     x_pred = int(x_pred)
#     y_pred = input("Enter robot y position : ")
#     y_pred = 200 - int(y_pred) - 1
#     start = check_viableY(y_pred)
#     if start == True:
#         start = check_viableX(x_pred)
#
# while goal == False:
#     x_prey = input("Enter prey x position : ")
#     x_prey = int(x_prey)
#     y_prey = input("Enter prey y position : ")
#     y_prey = 200 - int(y_prey) - 1
#     goal = check_viableY(y_prey)
#     if goal == True:
#         goal = check_viableX(x_prey)

x_pred = int(70)
y_pred = 200 - int(0) - 1
x_prey = int(180)
y_prey = 200 - int(0) - 1

start = t.time()

cost_for_pred_map = 20 * np.ones((200, 200))
cost_for_prey_map = 1 * np.ones((200, 200))
# cost_for_pred_map[98][100] = 1

start_node = [x_pred, y_pred]
prey_node = [x_prey, y_prey]

robot1 = Robot(start_node, prey_node)

workspace = plot_workspace(x_pred, y_pred, x_prey, y_prey)
cv2.imshow("Map", workspace)
cv2.waitKey(100)
print("1 move later")
min_node, goal_reached = pred_planner(workspace, robot1, cost_for_pred_map)

view_step = plot_workspace(x_pred, y_pred, min_node.x, min_node.y)
view_step[y_prey][x_prey] = [0, 0, 255]
img = view_step
scale_percent = 200  # percent of original size
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)
# resize image
img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
cv2.imshow("Map", img)
cv2.waitKey(100)
cost_for_pred_map = increment(cost_for_pred_map, "pred")
cost_for_prey_map = increment(cost_for_prey_map, "prey")
#################################################################
x = 0
while goal_reached == False:
    current_node = [min_node.x, min_node.y]
    workspace = plot_workspace(current_node[0], current_node[1], x_prey, y_prey)
    robot1 = Robot(current_node, prey_node)
    print(str(x + 2) + " moves later")
    min_node, goal_reached = pred_planner(workspace, robot1, cost_for_pred_map)

    view_step = plot_workspace(x_pred, y_pred, min_node.x, min_node.y)
    view_step[y_prey][x_prey] = [0, 0, 255]
    img = view_step
    scale_percent = 200  # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    # resize image
    img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
    cv2.imshow("Map", img)
    cv2.waitKey(100)

    cost_for_prey_map[min_node.y][min_node.x] = 255
    cost_for_pred_map[y_prey][x_prey] = 0

    cost_for_pred_map = increment(cost_for_pred_map, "pred")
    cost_for_prey_map = increment(cost_for_prey_map, "prey")

    # simulated prey movement
    if x % 2 == 0:
        robot2 = Robot(prey_node, min_node)
        max_node_prey = prey_planner(workspace, robot2, cost_for_prey_map)
        x_prey = max_node_prey.x
        y_prey = max_node_prey.y
        prey_node = [x_prey, y_prey]
    x += 1
# print(cost_for_pred_map)
# print(cost_for_prey_map)

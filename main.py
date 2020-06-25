#!/usr/bin/env python
# coding: utf-8
import math
import cv2
import time as t
import numpy as np
from final_proj_functions import *
import random

class Node:
    """node has a particular cost for the predator to move there and
    the node has a cost for the prey to move there"""
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost_for_pred = math.inf
        self.cost_for_prey = math.inf
        self.parent = None


class Robot:
    """the robot only needs start and goal points"""
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal


def pred_planner(image, robot, cost_for_pred_map):
    """path planner for the predator, plans one step per itteration"""
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
        current_node = get_min_node_pred(queue)
        current_point = [current_node.x, current_node.y]
        visitedNodes.append(str(current_point))

        for move in moves:
            new_point, cost_of_action = try_move(move, current_point)
            frame += 1

            if new_point is not None and check_distance(pred_node_pos, new_point)
            and check_distance(pred_node_pos,current_point):
                cost_for_pred = cost_for_pred_map[new_point[1]][new_point[0]]
                cost_to_go = get_cost_to_go(new_point, prey_node_pos)
                cost = cost_of_action + cost_for_pred + 1 * cost_to_go
                if new_point == prey_node_pos:
                    counter += 1
                new_node = Node(new_point[0], new_point[1])
                new_node.parent = current_node
                image = fill_pixel(image, current_node.x, current_node.y)
                image[pred_node_pos[1], pred_node_pos[0]] = [0, 255, 0]
                image[prey_node_pos[1], prey_node_pos[0]] = [0, 0, 255]

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
                        temp_step_node = step_queue[step_index]
                        if temp_step_node.cost_for_pred > cost + new_node.parent.cost_for_pred:
                            temp_step_node.cost_for_pred = cost + new_node.parent.cost_for_pred
                            temp_step_node.parent = current_node
            else:
                continue
            if counter == 1:
                min_node = get_min_node_pred(step_queue)
                print("current location " + str(min_node.x) + "," + str(200 - (min_node.y + 1)))
                print("Goal reached!")
                return min_node, True
    min_node = get_min_node_pred(step_queue)
    return min_node, False

def prey_planner(image, robot, cost_for_prey_map):
    """path planner for the prey agent, plans on step per time increment"""
    pred_node_pos = robot.goal
    prey_node_pos = robot.start

    image[prey_node_pos[1], prey_node_pos[0]] = [0, 0, 255]

    start_node = Node(prey_node_pos[0], prey_node_pos[1])
    start_node.cost_for_prey = 0

    visitedNodes = list()
    queue = [start_node]
    step_queue = []

    moves = ["move_up", "move_down", "move_left", "move_right",
             "move_up_right", "move_down_right", "move_up_left", "move_down_left"]
    counter = 0
    frame = 0

    while queue:
        current_node = get_min_node_prey(queue)
        current_point = [current_node.x, current_node.y]
        visitedNodes.append(str(current_point))

        for move in moves:
            new_point, cost_of_action = try_move(move, current_point)
            frame += 1

            if new_point is not None and check_distance(prey_node_pos, new_point)
            and check_distance(prey_node_pos,current_point):
                cost_for_prey = cost_for_prey_map[new_point[1]][new_point[0]]
                #give some random motion
                cost_r = random.randint(0,2)
                Wc = .5
                Wp = 1
                cost_to_pred = -Wp * get_cost_to_go(new_point, (pred_node_pos.x, pred_node_pos.y))
                cost_to_center = Wc * get_cost_to_go(new_point, (100, 100))
                cost_p = cost_of_action + cost_for_prey + cost_to_pred + cost_r + cost_to_center
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
                        temp_step_node = step_queue[step_index]
                        if temp_step_node.cost_for_prey > cost_p + new_node.parent.cost_for_prey:
                            temp_step_node.cost_for_prey = cost_p + new_node.parent.cost_for_prey
                            temp_step_node.parent = current_node
            else:
                continue
    min_node = get_min_node_prey(step_queue)
    return min_node
#################################################

font = cv2.FONT_HERSHEY_SIMPLEX
org = (5, 40)
fontScale = 1
color = (255, 0, 0)
thickness = 2

"""initialize pred and prey randomly, prey closer to the center"""
x_pred = random.randint(0, 199)
y_pred = 200 - random.randint(0, 200) - 1
x_prey = random.randint(50, 150)
y_prey = 200 - random.randint(50, 150) - 1

start = t.time()

cost_for_pred_map = 255 * np.ones((200, 200))
cost_for_prey_map = 1 * np.ones((200, 200))
blank_screen = 255 * np.ones((50,250,3))

start_node = [x_pred, y_pred]
prey_node = [x_prey, y_prey]

predator = Robot(start_node, prey_node)

workspace = plot_workspace(x_pred, y_pred, x_prey, y_prey)
cv2.imshow("Map", workspace)
moves_screen = blank_screen.copy()
moves_screen = cv2.putText(moves_screen, 'Moves: 0', org, font,
                   fontScale, color, thickness, cv2.LINE_AA)
cv2.imshow("Moves",moves_screen)
cv2.waitKey(10)
print("Moves Later: 1")
min_node, goal_reached = pred_planner(workspace, predator, cost_for_pred_map)

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
moves_screen = blank_screen.copy()
moves_screen = cv2.putText(moves_screen, 'Moves: 1', org, font,fontScale, color, thickness, cv2.LINE_AA)
cv2.imshow("Moves",moves_screen)
cv2.waitKey(10)
cost_for_pred_map = increment(cost_for_pred_map, "pred")
cost_for_prey_map = increment(cost_for_prey_map, "prey")

#################################################################
x = 0
while goal_reached == False:
    current_node = [min_node.x, min_node.y]
    workspace = plot_workspace(current_node[0], current_node[1], x_prey, y_prey)
    predator = Robot(current_node, prey_node)
    print("Moves Later: " + str(x + 2))
    min_node, goal_reached = pred_planner(workspace, predator, cost_for_pred_map)

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
    moves_screen = blank_screen.copy()
    moves_screen = cv2.putText(moves_screen, "Moves: " + str(x + 2), org, font,fontScale, color, thickness, cv2.LINE_AA)
    cv2.imshow("Moves",moves_screen)
    cv2.waitKey(10)

    cost_for_prey_map[min_node.y][min_node.x] = 100
    cost_for_pred_map[y_prey][x_prey] = 155

    cost_for_pred_map = increment(cost_for_pred_map, "pred")
    cost_for_prey_map = increment(cost_for_prey_map, "prey")

    pred_img = cv2.cvtColor(cost_for_pred_map.astype('uint8'), cv2.COLOR_GRAY2BGR)
    prey_img = cv2.cvtColor(cost_for_prey_map.astype('uint8'), cv2.COLOR_GRAY2BGR)
    cv2.imshow("Cost for prey",prey_img)
    cv2.imshow("Cost for pred",pred_img)
    cv2.waitKey(10)
    # simulated prey movement
    speed_of_pred = 1
    """set the speed of the predator to 1,2&3 times the prey
     to see different results"""
    if x % speed_of_pred == 0:
        prey = Robot(prey_node, min_node)
        mix_node_prey = prey_planner(workspace, prey, cost_for_prey_map)
        x_prey = mix_node_prey.x
        y_prey = mix_node_prey.y
        prey_node = [x_prey, y_prey]
    x += 1

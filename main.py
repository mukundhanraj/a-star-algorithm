# main.py
# Copyright (c) 2022 Rishabh Mukund
# MIT License
#
# Description: Main method to calcualte optimal path using a-star

import map
import astar

if __name__ == '__main__':
    map_len = 400
    map_bre = 250
    thresh = 1.5
    flag = True
    validPoints = []
    radius = 0
    clearance = 0
    thresh = 0

    radius = input('Input the radius of the robot:\n')
    radius = int(radius)
    if radius < 1:
        flag = False
        print('Radius must be positive, please try again...')

    if flag:
        clearance = input('Input the clearance:\n')
        clearance = int(clearance)
    if clearance < 0:
        flag = False
        print('clearance must be positve or 0, please try again...')

    if flag:
        validPoints = map.listOfValidPoints(map_len, map_bre, radius)

    if flag:
        start = input('Input Staring Position in format: x,y,th\n')
        start = (int(start.split(',')[0]), int(
            start.split(',')[1]), int(start.split(',')[2]))
        if not map.isPointValid(start[0:2], validPoints, clearance):
            flag = False
            print('Not a valid point, please try again...')

    if flag:
        goal = input('Input Staring Position in format: x,y,th\n')
        goal = (int(goal.split(',')[0]), int(
            goal.split(',')[1]), int(goal.split(',')[2]))
        if not map.isPointValid(goal[0:2], validPoints, clearance):
            flag = False
            print('Not a valid point, please try again...')

    if flag:
        step = input('Input the step size for robot:\n')
        step = int(step)
        if step < 1:
            flag = False
            print('not a valid step size, please try again...')

    if flag:
        print('starting')
        reached, parent_map, closed = astar.astar(
            start, goal, validPoints, clearance, step, thresh)
        if reached:
            print('reached')
            path = astar.getPath(parent_map, start, goal, closed)
            print(path)
            astar.animate(map_len, map_bre, validPoints,
                          closed, path, parent_map)
        else:
            print('the point cannot be reached')

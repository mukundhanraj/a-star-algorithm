# map.py>
#
# Copyright (c) 2022 Rishabh Mukund
# MIT License
#
# Description: definig the given map

import numpy as np


def listOfValidPoints(map_len, map_bre, radius=1):
    """
    Definition
    ---
    Method to generate a list of all valid points on a map

    Parameters
    ---
    map_len : length of map
    map_bre : breadth of map
    radius: radius of the robot (default, point robot)

    Returns
    ---
    validPoints : list of all the valid points
    """
    validPoints = []

    # Defining Circle
    xc = 300
    yc = 185
    rc = 40

    # Defining Polygon
    x1 = 36
    y1 = 185
    x2 = 115
    y2 = 210
    x3 = 80
    y3 = 180
    x4 = 105
    y4 = 100

    # Defining Hexagon
    x5 = 200
    y5 = 140.4
    x6 = 235
    y6 = 120.2
    x7 = 235
    y7 = 79.8
    x8 = 200
    y8 = 59.6
    x9 = 165
    y9 = 79.8
    x0 = 165
    y0 = 120.2

    # Slopes of the lines
    m21 = (y2 - y1) / (x2 - x1)
    m32 = (y3 - y2) / (x3 - x2)
    m43 = (y4 - y3) / (x4 - x3)
    m14 = (y1 - y4) / (x1 - x4)

    m65 = (y6 - y5) / (x6 - x5)
    m87 = (y8 - y7) / (x8 - x7)
    m98 = (y9 - y8) / (x9 - x8)
    m50 = (y5 - y0) / (x5 - x0)

    rob = np.array([], np.int0)
    for x in range(-radius, radius+1):
        for y in range(-radius, radius+1):
            if np.linalg.norm([x, y]) > radius:
                rob = np.append(rob, [x, y])
    rob = np.reshape(rob, (-1, 2))

    for xm in range(map_len + 1):
        for ym in range(map_bre + 1):
            flag = False
            for xr, yr in rob:
                x = xm + xr
                y = ym + yr
                if ((x - xc)**2 + (y - yc)**2) <= rc**2:
                    flag = True
                if (y-y1) <= (m21*(x-x1)) and (y-y2) >= (m32*(x-x2)) and \
                   (y-y4) >= (m14*(x-x4)):
                    flag = True
                if (y-y1) <= (m21*(x-x1)) and (y-y3) <= (m43*(x-x3)) and \
                   (y-y4) >= (m14*(x-x4)):
                    flag = True
                if x <= x6 and x >= x9 and y >= y9 and y <= y6:
                    flag = True
                if y >= y6 and (y-y5) <= (m65*(x-x5)) and \
                        (y-y0) <= (m50*(x-x0)):
                    flag = True
                if y <= y9 and (y-y7) >= (m87*(x-x7)) and \
                        (y-y8) >= (m98*(x-x8)):
                    flag = True
                if flag:
                    break
            if not flag:
                validPoints.append((x, y))
    return validPoints


def isPointValid(point, validPoints, clearance, radius = 1):
    """
    Definition
    ---
    Method to check if point is valid and clear of obstacles

    Parameters
    ---
    point : node of intrest
    validPoints : list of all valid points
    clearance : minimum distance required from obstacles
    radius: radius of the robot (default, point robot)

    Returns
    ---
    bool : True if point is valid, False othervise
    """
    for i in range(-clearance - radius, clearance + radius):
        for j in range(-clearance - radius, clearance + radius):
            if not (point[0] + i, point[1] + j) in validPoints:
                return False
    return True

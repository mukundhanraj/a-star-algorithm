# map.py>
#
# Copyright (c) 2022 Mukundhan Rajendiran
# MIT License
#
# Description: definig the given map


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
    rc = 40 + radius

    # Defining Polygon
    x1 = 36 - radius
    y1 = 185
    x2 = 115 + radius
    y2 = 210 + radius
    x3 = 80 + radius
    y3 = 180
    x4 = 105 + radius
    y4 = 100 - radius

    # Defining Hexagon
    x5 = 200
    y5 = 140.4 + radius
    x6 = 235 + radius
    y6 = 120.2 + radius
    x7 = 235 + radius
    y7 = 79.8 - radius
    x8 = 200
    y8 = 59.6 - radius
    x9 = 165 - radius
    y9 = 79.8 - radius
    x0 = 165 - radius
    y0 = 120.2 + radius

    # Slopes of the lines
    m21 = (y2 - y1) / (x2 - x1)
    m32 = (y3 - y2) / (x3 - x2)
    m43 = (y4 - y3) / (x4 - x3)
    m14 = (y1 - y4) / (x1 - x4)

    m65 = (y6 - y5) / (x6 - x5)
    m87 = (y8 - y7) / (x8 - x7)
    m98 = (y9 - y8) / (x9 - x8)
    m50 = (y5 - y0) / (x5 - x0)

    for x in range(radius, map_len + 1 - radius):
        for y in range(radius, map_bre + 1 - radius):
            if ((x - xc)**2 + (y - yc)**2) <= rc**2:
                continue
            if (y-y1) <= (m21*(x-x1)) and (y-y2) >= (m32*(x-x2)) and \
                    (y-y4) >= (m14*(x-x4)):
                continue
            if (y-y1) <= (m21*(x-x1)) and (y-y3) <= (m43*(x-x3)) and \
                    (y-y4) >= (m14*(x-x4)):
                continue
            if x <= x6 and x >= x9 and y >= y9 and y <= y6:
                continue
            if y >= y6 and (y-y5) <= (m65*(x-x5)) and (y-y0) <= (m50*(x-x0)):
                continue
            if y <= y9 and (y-y7) >= (m87*(x-x7)) and (y-y8) >= (m98*(x-x8)):
                continue
            validPoints.append((x, y))
    return validPoints


def isPointValid(point, validPoints, clearance):
    """
    Definition
    ---
    Method to check if point is valid and clear of obstacles

    Parameters
    ---
    point : node of intrest
    validPoints : list of all valid points
    clearance : minimum distance required from obstacles

    Returns
    ---
    bool : True if point is valid, False othervise
    """
    for i in range(-clearance, clearance):
        for j in range(-clearance, clearance):
            if not (point[0] + i, point[1] + j) in validPoints:
                return False
    return True

# This program generates a simple rapidly
# exploring random tree (RRT) in a rectangular region.
#
# Written by Steve LaValle
# May 2011
# Link to Original Code: http://msl.cs.illinois.edu/~lavalle/sub/rrt.py
#
# Modified by Rumaisa Abdulhai
# November 2019

import sys, random, math, pygame
from pygame.locals import *
from math import sqrt,cos,sin,atan2

#############
# CONSTANTS #
#############

XDIM = 640
YDIM = 480
WINSIZE = [XDIM, YDIM]
screen = pygame.display.set_mode(WINSIZE)

EPSILON = 7.0
NUMNODES = 5000
GOAL_RANGE = 15.0

white = 255, 240, 200
black = 20, 20, 40
red = 240,50,50

ob_1 = [150, 250, 30, 40]
ob_2 = [500, 250, 40, 30]
ob_3 = [300, 350, 20, 30]
ob_4 = [350, 150, 60, 40]

obs_list = [ob_1,ob_2,ob_3,ob_4]

start = ( XDIM/2.0 , YDIM/2.0 ) # nodes.append((0.0,0.0)) # Start in the corner
goal = ( 200 , 300 )
nodes = []
nodes.append(start) # Start in the center

nn = nodes[0]

###########
# METHODS #
###########

def main():
    global nn
    pygame.init()
    pygame.display.set_caption('RRT with Collision Detection')
    screen.fill(black)
    createObstacles()
    
    for i in range(NUMNODES):
        rand = generateRandomPoint()

        for p in nodes:
            if dist(p,rand) < dist(nn,rand):
                nn = p
        new_node = step_from_to(nn,rand)

        if isCollision(nn, new_node) == True:
            print("COLLISION DETECTED. SKIPPING POINT", new_node)
            continue

        nodes.append(new_node)
        pygame.draw.line(screen,white,nn,new_node)
        pygame.display.update()

        # withinGoal(nn)

        for e in pygame.event.get():
	        if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
	            sys.exit("Leaving because you requested it.")

def dist(p1,p2):
    """
    This method returns the euclidean distance between two nodes

    Parameters:
    -----------
    p1: first specified node,
    p2: second specified node
    """
    return sqrt( (p1[0]-p2[0]) ** 2 + (p1[1]-p2[1]) ** 2 )

def step_from_to(p1,p2):
    """
    This method returns a closer node within EPSILON units 
    if the distance from p1 to p2 is large

    Parameters:
    -----------
    p1: first specified node,
    p2: second specified node
    """
    if dist(p1,p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + EPSILON * cos(theta), p1[1] + EPSILON * sin(theta)
 
def generateRandomPoint():
    """
    This method generates a random point on the screen
    """
    return random.random() * XDIM, random.random() * YDIM

def createObstacles():
    """ 
    This method creates obstacles
    """
    for ob in obs_list:
        pygame.draw.rect(screen, red, pygame.Rect(ob))

def isCollision(nn, new_node):
    """
    This method checks to see if the line connecting the close node and nearest node will pass through an obstacle. Returns true if there is a collision and false if there is no collision.
    """
    start_pos = nn
    end_pos = new_node

    x1 = start_pos[0]
    y1 = start_pos[1]

    x2 = end_pos[0]
    y2 = end_pos[1]

    m = (y2-y1) / (x2-x1)
    b = y2 - m * x2

    points = []
    start_x = 0

    if x1 > x2:
        start_x = x2
    else:
        start_x = x1

    range_x = abs(x1-x2)

    for i in range(int(start_x), int(start_x) + int(range_x) + 1):
        j = m * i + b
        point = int(i), int(j)
        points.append(point)

    for point in points:
        for ob in obs_list:
            rect = pygame.Rect(ob)
            if rect.collidepoint(point) == True:
                return True
    return False

def withinGoal(nn):
    """
    This methods checks if the goal has been reached

    Parameters:
    -----------
    nn: the current nearest node
    """
    if dist(goal,nn) <= GOAL_RANGE:
        print("found!")
    else:
        print("not yet, dist is " , dist(goal,nn))

if __name__ == '__main__':
    main()
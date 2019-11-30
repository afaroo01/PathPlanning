# This program generates a simple rapidly
# exploring random tree (RRT) in a rectangular region.
#
# Written by Steve LaValle
# May 2011
# Link to Original Code: http://msl.cs.illinois.edu/~lavalle/sub/rrt.py
#
# Modified by Rumaisa Abdulhai
# November 2019

                                                            ############### BEG OF CLASS ###############
class Node():

    ###############
    # CONSTRUCTOR #
    ###############

    def __init__(self, parent = None, position = None):

        """
        parent: The node that precedes the current node
        position: The x and y coordinate of the current node
        """
        
        self.parent = parent
        self.position = position

                                                            ############### END OF CLASS ###############

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
GOAL_RANGE = 5.0

white = 255, 240, 200
black = 20, 20, 40
red = 240, 50, 50
blue = 0, 0, 255

ob_1 = [150, 250, 30, 40]
ob_2 = [500, 250, 40, 30]
ob_3 = [300, 350, 20, 30]
ob_4 = [350, 150, 60, 40]

obs_list = [ob_1,ob_2,ob_3,ob_4]

start = ( XDIM/2.0 , YDIM/2.0 ) # start = ( 0,0 )
goal = (500, 400)

start_node = Node(None, start)
goal_node = Node(None, goal)

nodes = []
nodes.append(start_node) # Start in the center

nn = nodes[0]

###########
# METHODS #
###########

def main():

    global nn
    pygame.init()
    pygame.display.set_caption('RRT with Collision Detection- Rumaisa Abdulhai')
    screen.fill(black)
    createObstacles()
    pygame.draw.circle(screen, blue, (int(start[0]),int(start[1])), 7)
    
    gameExit = False

    while not gameExit:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit("Leaving because you requested it.")
                gameExit = True
            if event.type == pygame.KEYDOWN:
                if event.key == ord('q'):
                    sys.exit("Leaving because you requested it.")
                    gameExit = True

        rand_node = Node(None, generateRandomPoint())

        for p in nodes:
            if dist( p.position, rand_node.position ) < dist( nn.position, rand_node.position ):
                nn = p

        new_node = Node( nn, step_from_to(nn,rand_node) )

        if isCollision(nn.position, new_node.position) == True:
            print("COLLISION DETECTED. SKIPPING POINT", new_node.position)
            continue
        
        nodes.append(new_node)
        pygame.draw.line(screen, white, nn.position, new_node.position)
        withinGoal(nn)

        pygame.display.update()
        
def get_path(final_node):
    """
    This method gets the list of coordinates of the final path

    Parameters:
    -----------
    final_node: The last node close to the goal
    """
    path = []
    final = final_node
    while final is not None: # goes all the way back to the start node whose parent is none
        path.append(final.position)
        final = final.parent

    return path[::-1] # Return reversed path

def draw_path(final_node):
    """
    This method draws the final path from the start point to the goal point in blue

    Parameters:
    -----------
    final_node: The last node close to the goal
    """
    points = get_path(final_node)

    for i in range(1,len(points)):
        pygame.draw.line(screen, blue, points[i], points[i-1],3)

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
    if dist(p1.position,p2.position) < EPSILON:
        return p2.position
    else:
        theta = atan2(p2.position[1]-p1.position[1],p2.position[0]-p1.position[0])
        return p1.position[0] + EPSILON * cos(theta), p1.position[1] + EPSILON * sin(theta)
 
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

def isCollision(nn_pos, new_node_pos):
    """
    This method checks to see if the line connecting the close node
    and nearest node will pass through an obstacle. Returns true if there 
    is a collision and false if there is no collision.
    """
    start_pos = nn_pos
    end_pos = new_node_pos

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
    This methods checks if the goal has been reached.

    Parameters:
    -----------
    nn: the current nearest node
    """
    if dist(goal, nn.position) <= GOAL_RANGE:
        print("found!")
        pygame.draw.circle(screen, blue, (int(goal[0]),int(goal[1])), 7)
        draw_path(nn)
    else:
        print("not yet, dist is " , dist(goal,nn.position))

if __name__ == '__main__':
    main()

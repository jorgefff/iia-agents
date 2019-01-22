
# This module defines data structures to represent the game state.
#
# JMR@ua.pt 2017

from collections import namedtuple
from collections import ChainMap
from collections import deque
#from enum import Enum
import logging
import random

class Point(namedtuple("Point", ['x', 'y'])):
    """A Point is a tuple of coordinates inside a 2D world.
    
    It may be used to store absolute coords (points),
    but also to store point differences (vectors),
    or even (width, height) dimensions.
    
    Examples:
    """
    
    __slots__ = ()

    def __add__(self, other):
        return Point(self.x+other[0], self.y+other[1])
    
    def __sub__(self, other):
        """Return point difference."""
        return Point(self.x-other[0], self.y-other[1])
    
    def __abs__(self):
        """Return positive (width, height) of a vector."""
        return Point(abs(self.x), abs(self.y))

    def __str__(self):
        """String representation of a point.
        Make it the same as a tuple's, e.g.: (1, 2).
        But repr(p) == 'Point(1, 2)'.
        """
        return str(tuple(self))

class World:
    """A World object contains a view of the game world.

    This world consists of cells in a toroidal surface with a given size.
    The position of each cell is a 2D Point with integer coordinates.
    
    It includes methods to query and manipulate cell contents,
    but also useful methods to manipulate points and compute distances
    in the toroidal world.
    """
    
    def __init__(self, size, seed=None):
        logging.debug("Creating World(size={!r}, seed={!r})".format(size, seed))
        self.rnd = random.Random(seed)  # random generator to use in this world
        self.size = size
        self.walls = {}
        self.food = {}
        self.foodQueue = {t: deque() for t in FOODTYPES} # queues of foods of each type
        self.bodies = {}
        self.cells = ChainMap(self.bodies, self.food, self.walls)
        
        self.foodfield = []
        self.playerfield = []
    
    def put(self, pos, content):
        assert isinstance(pos, Point)
        pos = self.normalize(pos)
        if content in WALL:
            self.walls[pos] = content
        elif content in FOODTYPES:
            self.food[pos] = content
            self.foodQueue[t].append(p)
        else:
            self.bodies[pos] = content
    
    def get(self, pos, default=None):
        return self.cells.get(pos, default)
    
    
    # Methods to manipulate points and coords in the world
    def normalize(self, p):
        """Normalize point p. (Wrap coords around this toroidal world.)"""
        return Point(p[0]%self.size.x, p[1]%self.size.y)
    
    def point(self, x, y):
        """Create a normalized Point from separate coords."""
        p = (x, y)
        return self.normalize(p)
    
    def dist(self, p1, p2):
        """Distance between points in this world. (Manhattan distance.)"""
        d = abs(self.normalize(p2) - self.normalize(p1))
        return min(d.x, self.size.x-d.x) + min(d.y, self.size.y-d.y)
    
    def translate(self, p, d):
        """Point obtained by adding vector d to point p."""
        p = self.normalize(p)   # not needed if isinstance(p, Point)
        return self.normalize(p+d)

    def randCoords(self):
        """Return a random point in this world."""
        return Point(self.rnd.randrange(0,self.size.x), self.rnd.randrange(0,self.size.y))

    def generatePos(self, forbiden={}, preferred={}):
        """Generate a position guaranteed not to be in forbiden.
        Chosen randomly from the preferred positions, or from the full range.
        """
        preflist = [p for p in preferred if p not in forbiden]
        if len(preflist) > 0:
            pos = self.rnd.choice(preflist)
        else:
            while True:
                pos = self.randCoords()
                if pos not in forbiden: break
        assert pos not in forbiden
        return pos

    def generatePlayerBody(self, t):
        body = []
        while True: ## TODO: possibility of infinite loop?
            body.append(self.generatePos(forbiden=self.cells, preferred=self.playerfield))
            neighboors = [self.translate(body[0],d) for d in DIRECTIONS]
            body.append(self.generatePos(forbiden=self.cells, preferred=neighboors))
            if len(body)==2: break
        for p in body:
            self.put(p, t)
        return body

    def generateFood(self, t):
        """Create food of type t and return its position."""
        p = self.generatePos(forbiden=self.cells, preferred=self.foodfield)
        self.food[p] = t
        self.foodQueue[t].append(p)
        return p
    
    def eatFood(self, p):
        """Eat the food in position p and return its type."""
        t =  self.food.pop(p)
        self.foodQueue[t].remove(p)
        return t
    
    def moveFood(self, t):
        """Move a piece of food of type t."""
        p = self.foodQueue[t].popleft()  # food that has not moved for the longest
        t2 = self.food.pop(p)
        assert t2 == t
        newpos = [self.translate(p, d) for d in ACTIONS]
        #newpos = {self.translate(p, d): t for d in ACTIONS}
        assert p not in self.cells # current position must be free now
        assert p in newpos         # current position is always a valid option
        p2 = self.generatePos(forbiden=self.cells, preferred=newpos)
        self.food[p2] = t
        self.foodQueue[t].append(p2)
        logging.debug("Moving {}->{}".format(p, p2))
                        
    def loadField(self, pxarray):
        for x in range(len(pxarray)):
            for y in range(len(pxarray[x])):
                p = pxarray[x][y] & 0xFFFFFF
                if p == 0x00F900:   #food
                    self.foodfield.append(Point(x,y))
                elif p == 0xFF2600: #player
                    self.playerfield.append(Point(x,y))
                elif p == 0xAA7942: #wall
                    self.walls[Point(x, y)] = WALL
                elif p == 0xFFFFFF: #empty
                    pass
                else:   # not in [0xFF000000, 0]: #oldwall, oldempty 
                    logging.error("UNKNOWN color: {:02X}".format(p))
    
    def saveField(self, pxarray):
        pxarray[:,:] = 0xFFFFFF
        for pos in self.walls:
            pxarray[pos] = 0xAA7942
        for pos in self.foodfield:
            pxarray[pos] = 0x00F900
        for pos in self.playerfield:
            pxarray[pos ] = 0xFF2600
        
    def generateWalls(self, level):
        for i in range(1,level+1):
            lo = self.randCoords() #last wall
            self.walls[lo] = WALL
            for j in range(1, self.rnd.randint(1,level)):
                d = self.rnd.choice(DIRECTIONS[1:3])
                lo = self.translate(lo, d)
                self.walls[lo] = WALL

# CONSTANTS:

## Colours
Black = (0,0,0)
White = (255,255,255)
Red = (255,0,0)
Lime = (0,255,0)
Blue = (0,0,255)
Yellow = (255,255,0)
Cyan = (0,255,255)
Magenta = (255,0,255)
Silver = (192,192,192)
Gray = (128,128,128)
Maroon = (128,0,0)
Olive = (128,128,0)
Green = (0,128,0)
Purple = (128,0,128)
Teal = (0,128,128)
Navy = (0,0,128)
COLORS = [Red, Lime, Blue, Yellow, Cyan, Magenta, Silver, Gray, Maroon, Olive, Green, Purple, Teal, Navy]

WALLCOLOR = (139,69,19)
FOODCOLOR = {'M': Red, 'S': Blue}

## Actions and Directions
Up = Point(0,-1)
Down = Point(0,1)
Left = Point(-1,0)
Right = Point(1,0)
Stay = Point(0,0)       # Staying put
DIRECTIONS = [Up, Down, Right, Left]    # Does not contain Stay!
ACTIONS = [Stay] + DIRECTIONS           # Stay is always ACTIONS[0]

## Cell contents
WALL = 'W'
FOODTYPES = list(FOODCOLOR.keys())
#BODYTYPES = ['P0', 'P1'] # = agent names


## TESTS

# This code is just for testing the module.
# It is executed only if you run this as a script,
# but not if you import the module.
if __name__ == "__main__":
    
    p = Point(1,2)                  # Create a Point
    assert p == (1,2)               # You can compare Points and tuples
    assert p.x == 1 and p.y == 2    # Access coords by name
    assert p[0] == 1 and p[1] == 2  # Access coords by index
    
    q = Point(4,1)
    d = q - p                       # Point - Point = vector
    assert d == Point(3,-1)
    assert p + d == q               # Point + vector = Point
    
    w = World(Point(10,10))         # Create a toroidal 10x10 world
    p = w.point(0,9)                # A point 9 cells down from the origin = bottom left corner
    q = w.point(-1,18)              # A point 1 cell left, 18 down from the origin
    assert q == Point(9,8)          # It's the same as 9 right, 8 up in the toroidal world
    d = w.dist(p, q)                # Distance between p and q in the toroidal world
    assert d == 1+1                 # It's 2 because q is just 1 step left + 1 step up from p    
    print(p, q, d)
    r = w.translate(p, (-1,+1))     # r is 1 step left, 1 down from bottom left
    assert r == Point(9,0)          # It's the same as top right
    print(r)
    

# LongLife: a cooperating agents game
# v1.5  (This is the version used to score the agents on the first submission.)
#
# Changes since 1.4:
# * Timeout feature: agent.chooseAction is interrupted if it takes longer
#   than the nutrients would allow.
# * Randomly generated maps are saved to "currentmap.bmp".
# * New -f/--fps option in start.py to set the speed of visualization.
# * Minor improvements and cleanup.
#
# Features changed since 1.3:
# * Agents receive a world only with walls and size filled in.
#   Remaining world fields, such as foodfield, etc., are left empty.
# * It is now possible to set random generator seeds.
#   (You need to set PYTHONHASHSEED and LONGLIFESEED environment variables.)
#   This allows repeating initial conditions and often the complete game.
#
# Features changed since 1.2:
# * Redistribution of nutrients only happens between _live_ players.
#
# Features changed since v1.0:
# * Messages pass from P0 to P1 and from P1 to P0 always with half-cycle delay.
# * Nutrient redistribution may happen after each player acts.
# * Food is moved between player turns.
# * There's a tentative calibration of the timeslot.
#
# Initially based on the code provided by http://www.virtualanup.com at https://gist.githubusercontent.com/virtualanup/7254581/raw/d69804ce5b41f73aa847f4426098dca70b5a1294/snake2.py
# Modified into the ia-iia-snake game by Diogo Gomes <dgomes@av.it.pt> 2016
# Transformed into the ia-iia-longlife game by JM Rodrigues <jmr@ua.pt> 2017

import sys
import logging
import copy
from collections import namedtuple, ChainMap, Counter
import pygame
from pygame.locals import *
import time

from world import *
#from agent import Agent


class Player:
    def __init__(self, name, body, world, AgentClass, seed=None):
        self.name = name
        self.body = body[:]
        self.world = world
        self.age = 0
        
        agentWorld = World(world.size, seed=seed)
        agentWorld.walls.update(world.walls)
        self.agent = AgentClass(name, body[:], agentWorld)
        
        self.nutrients = {}
        self.nutrients['M'] = 1000
        self.nutrients['S'] = 1000
        self.timespent = 0.0    # cpu time spent in previous cycle (s)
        self.horizon = 20
        
        self.outbox = b""
        
    def filterVision(self, env):
        """Return only the parts of the environment which are visible."""
        return {p:v for p,v in env.items() if self.world.dist(p, self.body[0]) <= self.horizon}  

    def transferInfo(self):
        """Transfer proprioception from player into corresponding agent."""
        self.agent.age = self.age
        self.agent.body = copy.deepcopy(self.body)
        self.agent.nutrients = copy.deepcopy(self.nutrients)
        self.agent.timespent = self.timespent
    
    @staticmethod
    def redistributeNutrients(players):
        """Redistributes nutrients between players in order to equalize their stocks.
        For example: {'S':1, 'M':8} and {'S':8, 'M':2} -> {'S':4, 'M':5} and {'S':5, 'M':5}.
        """
        for t in FOODTYPES:
            # TODO: allPlayers? even if dead?
            total = sum(p.nutrients[t] for p in players)
            (m, r) = divmod(total, len(players))    # find integer mean and remainder
            sortedPlayers = sorted(players, key=lambda p: p.nutrients[t], reverse=True)
            for p in sortedPlayers:
                p.nutrients[t] = m + (r>0)  # add 1 to the first r players
                r -= 1
            # Test postcondition:
            assert total == sum(p.nutrients[t] for p in players), "Total should be conserved"
            n = [p.nutrients[t] for p in players]
            assert max(n) - min(n) <= 1, "Differences should be at most 1" 
        return
    
    def __repr__(self):
        return "({}, age={}, nutrients={}, head={})".format(self.name, self.age, self.nutrients, self.body[0])


class AgentGame:
    def __init__(self, AgentClass,
            width=60, height=40,
            filename=None, walls=15,
            foodquant=4, timeslot=0.020, calibrate=False,
            visual=False, fps=25, tilesize=20,
            seeds=(None, None)):
        
        logging.info("Original timeslot: {:.6f} s".format(timeslot))
        if calibrate:
            # Call 3 times and choose minimum (first time is usually higher)
            TCAL = min([self.calibrationTime(),
                        self.calibrationTime(),
                        self.calibrationTime()])
            timeslot *= TCAL/0.068
        logging.info("Adjusted timeslot: {:.6f} s".format(timeslot))

        if filename != None:
            logging.info("Loading {} ...".format(filename))
            image = pygame.image.load(filename)
            pxarray = pygame.PixelArray(image)
            width = len(pxarray)
            height = len(pxarray[0])
        
        ## Create the game world view:
        self.world = World(Point(width, height), seed=seeds[0])
        # Set seed for static generator:
        random.seed(seeds[1])
        
        self.foodquant = foodquant  # quantity of each kind of food
        self.timeslot = timeslot    # time in seconds afforded per unit of S nutrient (Confused?)
        self.bytesPerS = 10         # bytes of communication afforded per unit of S nutrient
        
        ## Fill the walls
        if filename != None:
            self.world.loadField(pxarray)
        else:
            self.world.generateWalls(walls)
            # Save a copy of the current field
            pxarray = pygame.PixelArray(pygame.Surface(self.world.size))
            self.world.saveField(pxarray)
            pygame.image.save(pxarray.surface, "currentmap.bmp")
                
        self.fps = fps      # Frames per second
        self.tilesize = tilesize    # tile size
        
        if visual: 
            #create the window and do other stuff
            pygame.init()
            self.screen = pygame.display.set_mode(((self.world.size.x)*self.tilesize,(self.world.size.y)*self.tilesize+25), pygame.RESIZABLE)
            pygame.display.set_caption('LongLife Cooperating Agents Game')
            #load the font
            self.font = pygame.font.Font(None, 30)
        else:
            self.screen = None
            self.fps = 0         # no need to slow down when not displaying
        
        ## Create players
        self.allPlayers = []
        for name in ["P0", "P1"]:
            body = self.world.generatePlayerBody(name)
            player = Player(name, body, self.world, AgentClass, seed=random.randrange(2**60))
            self.allPlayers.append(player)
        self.livePlayers = self.allPlayers[:]   # list of live players
        self.deadPlayers = []                   # list of dead players
    
        ## Generate food
        for t in FOODTYPES:
            foodcount = 0
            while foodcount < self.foodquant:
                self.world.generateFood(t)
                foodcount += 1
        
    def calibrationTime(self):
        """Run a calibration loop and return the time it takes to complete."""
        mockworld = World(Point(60,40), 20171106)
        mockworld.generateWalls(15)
        origin = Point(0,0)
        p = mockworld.randCoords()
        a = Stay
        t = time.process_time()
        t2 = time.perf_counter()
        #t3 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID) # Not on Windows
        for k in range(10000):
            p = mockworld.translate(p, a)
            d = mockworld.dist(p, origin)
            if p in mockworld.walls:
                a = mockworld.rnd.choice(ACTIONS[1:])
            else:
                mockworld.put(p, 'x')
        t = time.process_time() - t
        t2 = time.perf_counter() - t2
        #t3 = time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID) - t3
        logging.debug("Calibration(cpu,perf) {:.6f} {:.6f}".format(t, t2))
        return t
    
    def killPlayer(self, player):
        #for t in player.nutrients:
        #    player.nutrients[t] = 0
        self.livePlayers.remove(player)
        self.deadPlayers.append(player)

    def executeAction(self, player, action):
        
        ## Check action
        if action not in ACTIONS:
            self.killPlayer(player)
            logging.error("{} invalid action: {} -> DEAD".format(player.name, action))
            return
        
        ## Check message
        if not isinstance(player.outbox, bytes):
            self.killPlayer(player)
            logging.error("{} invalid message: {} -> DEAD".format(player.name, player.outbox))
            player.outbox = b""
            return
        
        ## Account for nutrients
        costT = 1 + int(player.timespent/self.timeslot)   # Cost of Thinking
        costC = (len(player.outbox)+self.bytesPerS-1)//self.bytesPerS   # Cost of Communicating
        costA = 1 + player.age//1000       # Cost of Acting: increases with age
        # Thinking & Communicating consume S nutrients:
        player.nutrients['S'] -= costT + costC
        # Acting consumes M nutrients:
        player.nutrients['M'] -= costA
        logging.info("{} consumes {} units of S <= spent {:.4f} s thinking.".format(player.name, costT, player.timespent))
        logging.info("{} consumes {} units of S <= spent {} bytes communicating.".format(player.name, costC, len(player.outbox)))
        logging.info("{} consumes {} units of M <= chose action {}.".format(player.name, costA, action))
        
        if player.nutrients['S'] <= 0 or player.nutrients['M'] <= 0:
            self.killPlayer(player)
            logging.info("{} run out of nutrients -> DEAD".format(player.name))
            return
        
        ## Move (or stay put)
        if action != Stay:
            # New head position
            head = player.body[0]    ## current head of player
            head = self.world.translate(head, action)
                    
            # Check if the player crashed
            if head in self.world.walls:    # hit a wall
                self.killPlayer(player)
                logging.info("{} crashed against wall -> DEAD".format(player.name))
                return
            if head in self.world.bodies:   # hit a body
                self.killPlayer(player)
                logging.info("{} crashed against a body -> DEAD".format(player.name))
                return
            # Update the body
            tail = player.body[-1]   # tail tip (may be needed after eating)
            player.body = [head] + player.body[:-1]
            self.world.bodies[head] = player.name
            self.world.bodies.pop(tail)
            if head in self.world.food: # eat food
                ## remove the food
                t = self.world.eatFood(head)
                self.world.generateFood(t)
                ## absorb nutrients (but not indefinitely)
                player.nutrients[t] = min(player.nutrients[t]+100, 2000)
                ## grow body
                #player.body.append(tail)
                #self.world.bodies[tail] = player.name
        
        self.checkBodies()  # check bodies invariant

    def checkBodies(self):
        c = 0
        for player in self.allPlayers:
            for pos in player.body:
                c += 1
                assert pos in self.world.bodies
                assert self.world.bodies[pos] == player.name
        assert c == len(self.world.bodies), (c, self.world.bodies,)
    
    
    def getEvents(self):
        ## Get events
        if self.screen != None:
            self.world.keys = []
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == pygame.KEYDOWN and event.key == K_ESCAPE): #close window or press ESC
                    pygame.quit();
                    raise KeyboardInterrupt("ESC pressed")
                elif event.type == pygame.KEYDOWN:
                    ## Other key down events will be available to every agent (may be used for testing)
                    self.world.keys.append(event.key)
                elif event.type == pygame.VIDEORESIZE:
                    self.tilesize = int(min(event.w/self.world.size.x, event.h/self.world.size.y))
                    self.screen = pygame.display.set_mode((self.world.size.x*self.tilesize, self.world.size.y*self.tilesize+25), pygame.RESIZABLE)
                    print(event, self.tilesize)
    
    def show(self):
        # Show all the game contents in the screen
        if self.screen != None:
            T = self.tilesize
            self.screen.fill((0,0,0))
            
            # Show walls
            for wall in self.world.walls:
                pygame.draw.rect(self.screen, WALLCOLOR, (wall[0]*T, wall[1]*T, T, T), 0)
            
            # Show food
            for f in self.world.food:
                pygame.draw.ellipse(self.screen, FOODCOLOR[self.world.food[f]], (f[0]*T, f[1]*T, T, T), 0)
            
            ## Show players
            for player in self.allPlayers:
                f = 0.25+0.75*player.nutrients['S']/2000
                (r,g,b) = FOODCOLOR['S']
                head_color = (int(r*f), 64, int(b*f))
                f = 0.25+0.75*player.nutrients['M']/2000
                (r,g,b) = FOODCOLOR['M']
                color = (int(r*f), 64, int(b*f))
                #head + rest of body
                pygame.draw.rect(self.screen, head_color, (player.body[0][0]*T, player.body[0][1]*T, T, T), 0)
                for part in player.body[1:]:
                    pygame.draw.rect(self.screen, color, (part[0]*T, part[1]*T, T, T), 0)
            
        # Show stats
        line = "  ".join(str(p) for p in self.allPlayers)
        
        logging.info("Stats: "+line)
        
        if self.screen != None:
            ## Show stats
            text = self.font.render(line, 1, (255,255,255))
            textpos = text.get_rect(x=0, y=(self.world.size.y)*self.tilesize)
            self.screen.blit(text, textpos)
        
        if len(self.livePlayers) == 0:
            line = "GAME OVER"
            logging.info(line)
            if self.screen != None:
                text=self.font.render(line, 1, (255,128,0), (0,0,0))
                textpos = text.get_rect(centerx=self.screen.get_width()/2, centery=self.screen.get_height()/2)
                self.screen.blit(text, textpos)
    
    def start(self):
        clock = pygame.time.Clock()
        # Single mailbox for passing messages between players:
        mailbox = b""
        
        # Discriminator for spreading food movement between player turns
        #   DISC = p*m - f*n,
        # where
        #   p = number of turns played, f = number of foods moved,
        #   n = number of players, m = number of foods to move per n turns
        # So,
        #   DISC increases m for each turn played,
        #   DISC decreases n for each food moved.
        # Note that:
        #   DISC > 0  <=>  p*m > f*n  <=>  p/n > f/m,
        # which means "number of foods is lower than it should"
        DISC = 0
        
        ## Main loop
        while len(self.livePlayers) > 0:
            clock.tick(self.fps)
            
            self.getEvents()
            
            #game logic is updated in the code below
            
            ## Update players
            n = len(self.livePlayers)   # number of players (and turns)
            for player in self.livePlayers:
                player.age += 1           # lived another tick!
                
                ## Move some foods (only this turn's share)
                assert len(self.world.foodQueue['M']) == self.foodquant
                DISC += self.foodquant  # another turn, increase m
                while DISC > 0:
                    self.world.moveFood('M')
                    DISC -= len(self.livePlayers)  # another movement, decrease n
                
                ## Transfer info to agent
                player.transferInfo()
                vision = namedtuple('Vision', ['bodies', 'food'])
                vision.bodies = player.filterVision(self.world.bodies)
                vision.food = player.filterVision(self.world.food)
                
                ## Call the AGENT (and measure time and catch errors)
                timeout = player.nutrients['S']*self.timeslot + 1.0
                s = time.process_time()
                try:
                    setDeadline(s, timeout, time.process_time)
                    action, player.outbox = player.agent.chooseAction(vision, mailbox)
                except Exception as e:
                    action, player.outbox = None, b""  # default if agent fails => Die
                    logging.exception(e)
                finally:
                    unsetDeadline()
                f = time.process_time()
                
                player.timespent = f - s
                logging.debug("{}.timespent = {:.4f} s".format(player.name, player.timespent))
                ## Execute the action and update the game
                self.executeAction(player, action)
                # Pass message
                mailbox = player.outbox
                
                ## If heads touch, redistribute nutrients
                # DONE: now done after each player!
                # DONE: now works for N players!
                # DONE: now only works on live players!
                neighbors = [p for p in self.livePlayers if self.world.dist(player.body[0], p.body[0]) <= 1]
                if len(neighbors) > 1:
                    logging.info("Rendez-vous {}".format(neighbors))
                    logging.debug("Before redistribution: {}".format(self.allPlayers))
                    Player.redistributeNutrients(neighbors)
                    logging.debug("After redistribution: {}".format(self.allPlayers))
                    
            ## Show the game state
            self.show()
            
            if self.screen != None:
                pygame.display.update()

        while self.screen != None:
            # Pygame BUG?: This seems to busy-wait (100% CPU)! Why?
            event = pygame.event.wait()
            if event.type == QUIT or (event.type == pygame.KEYDOWN and event.key == K_ESCAPE): #close window or press ESC
                pygame.quit(); 
                break
        
        score = sum([p.age for p in self.allPlayers])
        return score


## The timeout feature. (NOT VERY PORTABLE! May work only on UNIX systems.)

import signal
import math

def timeoutHandler(signum, frame):
    """Handler for SIGALRM.
    Funtion keeps 3 important attributes: .clock, .startTime, .limit."""
    currTime = timeoutHandler.clock()
    elapsed = currTime - timeoutHandler.startTime
    logging.debug("timeoutHandler elapsed={} limit={}.".format(elapsed, timeoutHandler.limit))
    if elapsed < timeoutHandler.limit:
        logging.debug("timeoutHandler woke up too early! Reseting alarm.")
        signal.alarm(int(math.ceil(timeoutHandler.limit - elapsed)))
    else:
        logging.debug("timeoutHandler interrupting.")
        # reset previous handler
        unsetDeadline()
        raise TimeoutError("Timeout reached.")

def setDeadline(start, limit, clock):
    """Set a deadline for after timeout seconds (int>0), measured using clock function."""
    logging.debug("setDeadline start={} limit={} clock={}.".format(start, limit, clock))
    ## Set a timeout
    timeoutHandler.prevHandler = signal.signal(signal.SIGALRM, timeoutHandler)
    timeoutHandler.startTime = start
    timeoutHandler.limit = limit
    timeoutHandler.clock = clock
    #currTime = timeoutHandler.clock()
    signal.alarm(int(math.ceil(limit)))
    return start

def unsetDeadline():
    """Remove a pending deadline."""
    signal.alarm(0)
    signal.signal(signal.SIGALRM, timeoutHandler.prevHandler)


## ATTEMPTS at making the timeout feature as a context manager.
## NOT WORKING YET!
from contextlib import contextmanager

# NOT WORKING
@contextmanager
def timelimit(limit, clock):
    """Run block subject to a time limit as measured by a given clock function."""
    try:
        logging.debug("setDeadline start={} limit={} clock={}.".format(start, limit, clock))
        ## Set a timeout
        timeoutHandler.prevHandler = signal.signal(signal.SIGALRM, timeoutHandler)
        timeoutHandler.startTime = start
        timeoutHandler.limit = limit
        timeoutHandler.clock = clock
        signal.alarm(int(math.ceil(limit)))
        start = clock()
        yield start
    finally:
        signal.signal(signal.SIGALRM, timeoutHandler.prevHandler)
        signal.alarm(0)
        


def handler(signum, frame):
    raise TimeoutError("time limit reached")

# NOT WORKING
@contextmanager
def timelimit2(limit, clock):
    """Run block subject to a time limit as measured by a given clock function."""
    signal.alarm(0)                                         # cancel any pending alarm
    prevHandler = signal.signal(signal.SIGALRM, handler)    # register our handler
    start = clock()
    logging.debug("timelimit: start={} limit={} clock={}.".format(start, limit, clock))
    elapsed = 0.0
    try:
        logging.debug("timelimit: {}<{}: setting alarm.".format(elapsed, limit))
        signal.alarm(int(math.ceil(limit - elapsed)))
        yield start              # execute block
        limit = elapsed     # concluded within limit => force stop
    except TimeoutError as e:       # alarm expired (real time)
        elapsed = clock() - start   # how much time passed in our clock?
    finally:
        logging.debug("timelimit: {}<{}: setting alarm.".format(elapsed, limit))
        signal.alarm(0)
        signal.signal(signal.SIGALRM, prevHandler)
        

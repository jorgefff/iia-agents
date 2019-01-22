import logging
from world import *

class Agent:
    def __init__(self, name, body, world):
        self.name = name
        self.world = world      # My representation of the world
        # world.size and world.walls are filled, remaining fields are empty!
        
        self.body = body        # List of body positions. body[0] is the head
        self.nutrients = {}     # Dict representing nutrient stock
        self.age = 0            # number of cycles this agent has lived so far
        self.timespent = 0.0    # seconds spent by previous call to chooseAction
        
    def chooseAction(self, vision, msg):
        """Analyse input vision and msg, and choose an action to do and msg to post."""
        # This is the brain of the agent. It should be reimplemented on any subclass.
        # It may use the fields (.name, .world, .body, .nutrients, .age, .timespent)
        # and the parameters (vision and msg) to decide what to do.
        # It must return an action (one of the ACTIONS),
        # and a message (a possibly empty bytes object) to send to the other agent.
        # (You may want to use pickle.dumps / pickle.loads to convert
        # objects to bytes and back.)
        
        action = Stay
        msg = b""       # an empty message costs nothing
        return action, msg

from agent import *
import random

# This is an example of an agent to play the LongLife game.
# It is not very intelligent: it simply avoids crashing onto walls or bodies.

class Agent1(Agent):
    def __init__(self, name, body, world):
        super().__init__(name, body, world)

    def chooseAction(self, vision, msg):
        # This is the brain of the agent
        # You can use self.name, .world, .body, .nutrients, .age, .timespent
        # and the parameters, vision and msg, to decide what to do.
        # You must return an action and a message (possibly empty).
        
        head = self.body[0]
        
        #get the list of valid actions for us
        validact = ACTIONS[:1]   # staying put is always valid
        for act in ACTIONS[1:]:
            newpos = self.world.translate(head, act)
            if newpos not in self.world.walls and newpos not in vision.bodies:
                validact.append(act)
        
        action = random.choice(validact)
        msg = b""
        return action, msg

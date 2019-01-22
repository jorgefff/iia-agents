
from agent import *
import random

class StudentAgent(Agent):
    def __init__(self, name, body, world):
        super().__init__(name, body, world)
        self.goal = None                # Selected food
        self.path = []                  # Path to selected food
        self.other_agent = []           # Position of other agent
        self.vision = None              # Global access to vision 
        self.other_agent_alive = True   # Other agent is alive
        
        self.goal_dist_update = 4       # Distance from goal where it updates path again        
        self.check_for_food = 5         # Number of cycles before it checks for food nearby
        self.chk_cycle = 0              # Counts cycles to check for food LEAVE AT ZERO
        self.dont_freeze = 3            # Max number of cycles it waits for other agent to pass
        
        self.food_memory = set()        # Memory of food positions
        self.chk_mem_limit = 20         # Number of positions it checks for optimal distance
        
        # Pre-processing:
        # Dead ends
        self.dead_ends = set()      # List of dead ends
        self.calcdead_ends()        # Finds dead ends
        self.tunnel_deadend()       # Finds dead end tunnels
        # Tunnels
        self.in_tunnel = False      # This agent is currently in a tunnel
        self.tunnels = dict()       # Initialize tunnels dictionary
        self.find_tunnels()         # Finds one way tunnels
        self.busy_tunnel = None     
        
        
        
    def chooseAction(self, vision, msg):
        # You can use self.name, .world, .body, .nutrients, .age, .timespent
            
        path = self.path        # Path
        head = self.body[0]     # Own head
        self.vision = vision    # Global access to vision
        dont_move = False
        
        if self.other_agent_alive and msg != b"":
            self.read_msg(msg)
        
        # Check if path needs updating
        update_path = False
        if not path \
        or self.tunnels.get(path[0],0) == self.busy_tunnel:
            update_path = True
        
        # Priority check
        if path and path[0] in vision.bodies:            
            if self.other_agent_alive \
            and self.name == 'P0':
                dont_move = True
            else:
                update_path = True
            
        # Check if goal needs updating
        update_goal = False
        if len(path) <= self.goal_dist_update:
            update_goal = True
        
        # Periodically checks for nearby food
        self.chk_cycle = (self.chk_cycle + 1) % self.check_for_food
        if self.chk_cycle == 0:
            if not update_path:
                limit = len(path)
                new_goal = self.find_food()
                # Checks if new goal is a better choice
                if new_goal:
                    new_path = self.find_path(new_goal,limit)
                    if new_path:
                        path.clear()
                        path.extend(new_path)
                        dont_move = False
                        update_goal = False
        
        # Update goal
        if update_goal:
            self.goal = self.find_food()
            if self.goal:
                update_path = True
            # No goal, check previous food positions
            elif self.food_memory:
                self.goal = self.food_from_memory()
                update_path = True
            # No previous positions, rand move
            if not self.goal:
                path.clear()            
        
        # Update path
        if update_path and self.goal:
            path.clear()
            path.extend(self.find_path(self.goal))
            dont_move = False
            
        
        if not dont_move:
            # Follow path 
            if path:
                new_pos = path.pop(0)
            # If no path, make random move
            else:
                new_pos = self.random_move()
            action = self.direction(head, new_pos)            
        else: 
            # Agent 0 gives passage
            # If stopped for 3 cycles, rand move
            self.dont_freeze -= 1
            if self.dont_freeze == 0:
                self.dont_freeze = 3
                path.clear()
                new_pos = self.random_move()
                action = self.direction(head, new_pos)
            else:
                action = (0,0)
                new_pos = head            
        
        # Inform other agent about enter/exit tunnel
        msg = b""
        if self.other_agent_alive:
            msg = self.name
            # Enter tunnel
            if not self.in_tunnel and self.tunnels.get(new_pos):
                self.in_tunnel = True
                msg += "i%i%s%i" % (new_pos.x, ",", new_pos.y)
            # Exit tunnel
            elif self.in_tunnel and not self.tunnels.get(new_pos):
                self.in_tunnel = False
                msg += "o"
            else:
                msg = ""
            msg = msg.encode()
        
        #print(self.name,":", head)
        #print("goal:",self.goal)
        #print("path:",[new_pos]+path)
        #print("going:", action)        
        #print("-----------------------------")
        
        return action, msg
    
    
    
    # Reads Messages from other agent
    def read_msg(self,msg):
        msg = msg.decode('utf-8')
        if msg[0:2] == self.name:
            self.other_agent_alive = False
        else:
            # remove message ID
            msg = msg[2:]
            # "out" Leaving a tunnel
            if msg == 'o':
                self.busy_tunnel = None
            # "in" Entering a tunnel
            elif msg[0] == 'i':
                x,y = list(map(int,msg.lstrip('i').split(',')))
                pos = Point(x,y)
                self.busy_tunnel = self.tunnels.get(pos)
        
    
    # Finds all tunnels that arent dead ends
    def find_tunnels(self):
        walls = self.world.walls
        nm=self.world.normalize
        tunnel_units = dict()
        tunnels = self.tunnels
        
        # Find tunnel_units        
        for w in walls:
            # Horizontal tunnel
            if nm((w.x, w.y+2)) in walls and nm((w.x, w.y+1)) not in walls:
                tunnel_units[nm(Point(w.x, w.y+1))] = 'h'
            if nm((w.x, w.y-2)) in walls and nm((w.x, w.y-1)) not in walls:
                tunnel_units[nm(Point(w.x, w.y-1))] = 'h'
            # Vertical tunnel
            if nm((w.x+2, w.y)) in walls and nm((w.x+1, w.y)) not in walls:
                tunnel_units[nm(Point(w.x+1, w.y))] = 'v'
            if nm((w.x-2, w.y)) in walls and nm((w.x-1, w.y)) not in walls:
                tunnel_units[nm(Point(w.x-1, w.y))] = 'v'            
        
        # Connect tunnel units into tunnels
        t_id = 1
        for unit in tunnel_units:
            if unit not in tunnels and unit not in self.dead_ends:
                self.connect_tunnels(unit, tunnel_units, t_id)
                t_id += 1
                
        
    def connect_tunnels(self, unit, tunnel_units, t_id, previous=None):
        if unit and unit not in self.tunnels:
            walls = self.world.walls
            this = tunnel_units.get(unit,'c')
            nm=self.world.normalize
            # Horizontal
            if this == 'h':
                self.tunnels[unit] = t_id
                self.connect_tunnels(nm((unit.x+1,unit.y)), tunnel_units, t_id, this)
                self.connect_tunnels(nm((unit.x-1,unit.y)), tunnel_units, t_id, this)
            # Vertical
            elif this == 'v':
                self.tunnels[unit] = t_id
                self.connect_tunnels(nm((unit.x,unit.y+1)), tunnel_units, t_id, this)
                self.connect_tunnels(nm((unit.x,unit.y-1)), tunnel_units, t_id, this)
            # Either connection between 2 tunnels, or end
            else:
                if (previous == 'h') \
                and (nm((unit.x+1,unit.y)) in walls or nm((unit.x-1,unit.y)) in walls) \
                and ((nm((unit.x,unit.y+1)) in tunnel_units) ^ (nm((unit.x,unit.y-1)) in tunnel_units)):
                    self.tunnels[unit] = t_id
                    self.connect_tunnels(nm((unit.x,unit.y+1)), tunnel_units, t_id, this)
                    self.connect_tunnels(nm((unit.x,unit.y-1)), tunnel_units, t_id, this)
                elif (previous == 'v') \
                and (nm((unit.x,unit.y+1)) in walls or nm((unit.x,unit.y-1)) in walls) \
                and ((nm((unit.x+1,unit.y)) in tunnel_units) ^ (nm((unit.x-1,unit.y)) in tunnel_units)):
                    self.tunnels[unit] = t_id
                    self.connect_tunnels(nm((unit.x+1,unit.y)), tunnel_units, t_id, this)
                    self.connect_tunnels(nm((unit.x-1,unit.y)), tunnel_units, t_id, this)
                    
    
    
    # Translates two positions to a direction / action
    def direction(self,src,dst):
        max_x,max_y = self.world.size
        x = dst[0]-src[0]
        y = dst[1]-src[1]
        if abs(x) > 1:
            x = -x // abs(x)
        elif abs(y) > 1:
            y = -y // abs(y)
        return Point(x,y)
    
    # Random (safe) move
    def random_move(self):
        bodies = self.vision.bodies
        walls = self.world.walls
        dead_ends = self.dead_ends
        head = self.body[0]
        tunnels = self.tunnels
        busy_tunnel = self.busy_tunnel
        translate = self.world.translate
        
        valid_pos = []
        for act in ACTIONS[1:]:
            new_pos = translate(head, act)
            if new_pos not in bodies \
            and  new_pos not in walls \
            and new_pos not in dead_ends \
            and tunnels.get(new_pos,0) != busy_tunnel:
                valid_pos.append(new_pos)
        
        if not valid_pos:
            valid_pos = [head]
            
        pos = random.choice(valid_pos)
        return pos    
    
    # Finds other agent
    def find_other(self):
        self.other_agent = [pos[0]
                            for pos in self.vision.bodies.items()
                            if not pos[1] == self.name]        
    
    # Finds food
    def find_food(self): 
        nutrients = self.nutrients
        self.find_other()
        tunnels = self.tunnels
        busy_tunnel = self.busy_tunnel
        other_agent = (self.other_agent[0] if self.other_agent else None)
        head = self.body[0]
        distf = self.world.dist
        
        # Food in valid position
        valid_food = {pos:f_type 
                      for (pos,f_type) in self.vision.food.items() 
                      if  pos not in self.dead_ends
                      and tunnels.get(pos,0) != busy_tunnel}
        
        # No choice
        if len(valid_food) == 0: # and len(discarded do outro) == 0
            return None
        
        # Update food memory
        self.food_memory = self.food_memory.union(valid_food.keys())        
        
        # Other agent in line of sigh
        discarded_food = set()
        if self.other_agent_alive and other_agent:
            for f_pos in valid_food.keys():
                d_to_food = distf(head, f_pos)
                d_other_to_food = distf(other_agent, f_pos)                
                if d_to_food > d_other_to_food:
                    discarded_food.add(f_pos)
        remove = valid_food.pop
        for discard in discarded_food:
            remove(discard)

        # Calc ratio of nutrients
        if 'S'  in valid_food.values() \
        and 'M' in valid_food.values():
            both_types = True
            s_ratio = nutrients['S'] / nutrients['M']
            m_ratio = nutrients['M'] / nutrients['S']
        else:
            both_types = False
            
        # Find nearest
        dist = 100
        selected = None
        for pos,f_type in valid_food.items():
            new_dist = distf(head, pos)
            # Calc distance based on nutrients needed
            if both_types:
                if f_type == 'S':
                    new_dist *= s_ratio
                else:
                    new_dist *= m_ratio
            if new_dist < dist:
                dist = new_dist
                selected = pos
                
        return selected
    
    
    # Returns position of previously found food
    def food_from_memory(self):
        head = self.body[0]
        distf = self.world.dist
        selected = None
        dist = 100
        chk_count = 0
        chk_limit = self.chk_mem_limit
        for f in self.food_memory:
            new_dist = distf(head,f)
            if new_dist > 20 and new_dist < dist:
                selected = f
                dist = new_dist
                chk_count += 1
                if chk_count == chk_limit:
                    break
        return selected
    
    # Find path to goal
    def find_path(self, goal, limit=None):
        # Local access to global vars/funcs
        self.find_other()
        walls = self.world.walls
        dead_ends = self.dead_ends
        heuristic = self.world.dist
        translate = self.world.translate
        head = self.body[0]
        other_agent = self.other_agent
        tunnels = self.tunnels
        busy_tunnel = self.busy_tunnel
        
        # Initializing
        start = Node(head, heuristic(head,goal))
        goal_node = None
        # List of nodes to be expanded
        open_nodes = [start]
        # Once a node is expanded, it goes in this list
        visited = set()
        # Main cycle, finds path to goal
        found = False
        while open_nodes:
            if limit and len(visited) >=  limit:
                break
            # Pick/remove first
            node = open_nodes.pop(0)
            visited.add(node)
            if node.pos == goal:
                found = True
                goal_node = node
            # Update tail position
            if node.parent == None:
                tail = self.body[1]
            else:
                tail = node.parent.pos
            # Check surrounding positions
            for act in ACTIONS[1:]:
                # Create new possible node
                p = translate(node.pos, act)
                h = heuristic(p, goal)
                new_node = Node(p,h,node)
                # Check if position is valid            
                # Can't go into: tail, other agent, visited positions, walls, dead ends, busy tunnels
                if  new_node.pos != tail \
                and new_node not in visited \
                and new_node.pos not in walls \
                and new_node.pos not in dead_ends \
                and tunnels.get(new_node.pos,0) != busy_tunnel \
                and new_node.pos not in other_agent:
                    # Check if it's already in open_nodes
                    if new_node in open_nodes:
                        # Check if it has better cost
                        change_node = open_nodes[open_nodes.index(new_node)]
                        if new_node.cost < change_node.cost:
                            change_node.new_parent(node)
                    # Else insert in open_nodes
                    elif not found:
                        # Find position based on heuristic
                        inserted = False
                        for op in open_nodes:
                            if new_node.heur < op.heur:
                                open_nodes.insert( open_nodes.index(op), new_node)
                                inserted = True
                                break
                        if not inserted:
                            open_nodes.append(new_node)
        # Secondary cycle, creates list of moves from start to path
        node = goal_node
        path = []
        #insert = path.insert
        if found:
            while node != start:
                path.insert(0, node.pos)
                node = node.parent
            
        return path
    
    #Tries to avoid dead_ends, includes dead_ends inside the walls
    def calcdead_ends(self):
        walls = self.world.walls
        for a in walls:
            if((a.x+1,a.y+1) in walls) and ((a.x+1,a.y-1) in walls) and ((a.x+1,a.y) not in walls):
                self.dead_ends.add((a.x+1,a.y))
            elif((a.x-1,a.y+1) in walls) and ((a.x-1,a.y-1) in walls) and ((a.x-1,a.y) not in walls):
                self.dead_ends.add((a.x-1,a.y))
            elif((a.x+1,a.y+1) in walls) and ((a.x-1,a.y+1) in walls) and ((a.x,a.y+1) not in walls):
                self.dead_ends.add((a.x,a.y+1))
            elif((a.x+1,a.y-1) in walls) and ((a.x-1,a.y-1) in walls) and ((a.x,a.y-1) not in walls):
                self.dead_ends.add((a.x,a.y-1))
        #print(self.dead_ends)
        return None

    #in progress
    def tunnel_deadend(self):
        tmp_de=set()
        walls = self.world.walls
        for a in self.dead_ends:
            #verifica em que direção é o tunel , e depois enquanto houver paredes dos lados adiciona essa posições aos dead_ends
            #y+
            yp=a[1]
            yp+=1
            while ((((a[0]+1,yp) in walls and (a[0]-1,yp)) in walls) \
                    or ((a[0]+1,yp) in walls and (a[0],yp+1) in walls) \
                    or ((a[0]-1,yp) in walls and (a[0],yp+1) in walls)) \
                    and ((a[0],yp) not in walls):
                tmp_de.add((a[0],yp))
                yp+=1
            #y-
            yn=a[1]
            yn-=1
            while ((((a[0]+1,yn) in walls and (a[0]-1,yn)) in walls) \
                    or ((a[0]+1,yn) in walls and (a[0],yn-1) in walls) \
                    or ((a[0]-1,yn) in walls and (a[0],yn-1) in walls)) \
                    and ((a[0],yn) not in walls):
                tmp_de.add((a[0],yn))
                yn-=1
            #x+
            xp=a[0]
            xp+=1
            while ((((xp,a[1]+1) in walls and (xp,a[1]-1)) in walls) \
                    or ((xp,a[1]+1) in walls and (xp+1,a[1]) in walls) \
                    or ((xp,a[1]-1) in walls and (xp+1,a[1]) in walls)) \
                    and ((xp,a[1]) not in walls):
                tmp_de.add((xp,a[1]))
                xp+=1
            #x-
            xn=a[0]
            xn-=1
            while ((((xn,a[1]+1) in walls and (xn,a[1]-1)) in walls) \
                    or ((xn,a[1]+1) in walls and (xn-1,a[1]) in walls) \
                    or ((xn,a[1]-1) in walls and (xn-1,a[1]) in walls)) \
                    and ((xn,a[1]) not in walls):
                tmp_de.add((xn,a[1]))
                xn-=1

        size_tunnels = len(self.dead_ends)
        self.dead_ends.update(tmp_de)
        if(size_tunnels != len(self.dead_ends)):
            self.tunnel_deadend()

        #print(self.dead_ends)
        #print(tmp_de)
        #quit()
        return None
    
    
# Node used in search algorithm
# node.pos
# node.parent
# node.heur
# node.cost
class Node:
    def __init__(self,pos,heur,parent=None):
        self.pos = pos
        self.parent = parent
        self.heur = heur
        if parent == None:
            self.cost = 0
        else:
            self.cost = 1 + parent.cost
    
    # To be searchable in a list
    def __eq__(self,other):
        if  isinstance(other, Node) \
        and self.pos[0] == other.pos[0] \
        and self.pos[1] == other.pos[1]:
            return True
        else:
            return False
        
    # To be searchable in a set
    def __hash__(self):
        return hash(self.pos)
    
    # Updates parent node
    def new_parent(self,new_parent):
        self.parent = new_parent
        self.cost = 1 + new_parent.cost
    
    # Debugging purposes
    def pr(self):
        print("NODE: pos:",self.pos," heur:",self.heur," cost:",self.cost)
    
    def __str__(self):
        return "("+self.pos[0]+","+self.pos[1]+")->"+self.parent
    
    
    

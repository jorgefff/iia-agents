# Agents

Done for the Introduction to Artificial Intelligence class in Aveiro's University

## Context:

There are 2 agents, with limited vision, with 2 types of resources. One is used when the agent moves, the other when he spends time "thinking".  
The agents can communicate, but communication costs resources.  
They die if they collide with walls or each other, or if they run out of resources.  
The goal is to last as long as possible. Resource usage increases over time.

## Main techniques used:

Finding the path to food is done with an algorithm based on A*.  
There is a "food memory", when agents can't see food they go where they found it before.  
Each agent informs the other when they enter single occupancy tunnels.  

## Requirements to run:

To run the game you need:

* Python >= 3.5
* pygame >= 1.9.2b6

## Usage examples:

### Intended use
python3 start.py -s StudentAgent
### play the game with basic agent on a random world  
python3 start.py
### use a specified world map  
python3 start.py -m mapa4.bmp
### show a log of information messages (and above)  
python3 start.py -d 1
### run fast without video, show debug log  
python3 start.py -d 0 -v


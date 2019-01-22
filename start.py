# Run this script to start the game.
# Examples:
# python3 start.py                  # play the game with Agent1 on a random world.
# python3 start.py -m mapa4.bmp     # use a specified world map.
# python3 start.py -s StudentAgent  # use another agent.
# python3 start.py -d 1             # show a log of information messages (and above).
# python3 start.py -d 0 -v          # run fast without video, show debug log

from game import *
from agent1 import Agent1
import importlib
import logging
import sys
import os
import getopt
import string

# Alphabet for generated seeds:
ALPHABET = string.ascii_uppercase + string.ascii_lowercase + string.digits

USAGE = \
"""start.py [-h/--help
          -s/--student-agent AgentName
          -m/--map <mapfile>
          -v/--no-video
          -f/--fps <FPS>
          -c/--calibrate
          -d/--debug Level(0--4)
"""


def main(argv):

    levels = [logging.DEBUG, logging.INFO, logging.WARNING, logging.ERROR, logging.CRITICAL]
    
    hashseed = os.environ.get("PYTHONHASHSEED", "random")
    random.seed(None)
    seedstr = ''.join(random.SystemRandom().choice(ALPHABET) for _ in range(20))
    seedstr = os.environ.get("LONGLIFESEED", seedstr)
    
    inputfile = None
    visual = True
    fps = 25
    studentAgent = Agent1
    debug = 2
    calibrate = False
    
    try:
        opts, args = getopt.getopt(argv,"hm:s:vf:cd:", ["help","map=", "student-agent=", "no-video", "fps=", "calibrate", "debug="])
    except getopt.GetoptError as e:
        print(e)
        print(USAGE)
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print(USAGE)
            sys.exit()
        elif opt in ["-m", "--map"]:
            inputfile = arg
        elif opt in ["-s", "--student-agent"]:
            classmodule = importlib.import_module(arg.lower())
            studentAgent = getattr(classmodule, arg)
        elif opt in ["-v", "--no-video"]:
            visual = False 
        elif opt in ["-f", "--fps"]:
            fps = int(arg)
        elif opt in ["-c", "--calibrate"]:
            calibrate = True
        elif opt in ["-d", "--debug"]:
            debug = int(arg)
        
    logging.basicConfig(format='%(levelname)s:\t%(message)s', level=levels[debug]) 
    
    try:
        # Try to pin process to a single CPU (to have more predictable times)
        aff = os.sched_getaffinity(0)
        logging.debug("Original affinity: {}".format(aff))
        cpu = aff.pop()
        os.sched_setaffinity(0, [cpu])  # TODO: does not seem to make a difference...
        aff = os.sched_getaffinity(0)
        logging.debug("New affinity: {}".format(aff))
    except:
        # Operating System may not support setaffinity.  Not a big deal.
        logging.debug("Could not set CPU affinity for the process.")
    
    try:
        print("Launching game.  PYTHONHASHSEED={} LONGLIFESEED={}".format(hashseed, seedstr))
        logging.info("Launching game.  PYTHONHASHSEED={} LONGLIFESEED={}".format(hashseed, seedstr))
        logging.info("cwd={!r} argv={!r} path={!r}".format(os.getcwd(), sys.argv, sys.path))
        game = AgentGame(AgentClass=studentAgent,
            width=60, height=40,
            filename=inputfile, walls=15,
            foodquant=4, timeslot=0.020, calibrate=calibrate,
            visual=visual, fps=fps, tilesize=20,
            seeds=(seedstr[::2], seedstr[1::2]))
        score = game.start()
        print("Score:", score)
    except Exception as e:
        logging.exception(e)
        sys.exit(1)
    
if __name__ == "__main__":
    main(sys.argv[1:])


import matplotlib
matplotlib.use('Agg') # No need to show it
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter  # For gif

import pdb # Use this for debugging python!
import argparse

###############################################################
################### Util Functions Below ######################

def readMap(mapfile):
    """ Input: mapfile path
    Output: 2D numpy array with binary values based on the map
    """
    with open(mapfile) as f:
        line = f.readline()  # e.g. "height 50"
        height = int(line.split(' ')[1])
        line = f.readline()  # e.g. "width 50"
        width = int(line.split(' ')[1])

        mapdata = np.array([line.rstrip().split(" ") for line in f])

    mapdata.reshape((width,height))
    mapdata = mapdata.astype(int)
    return mapdata

###############################################################
################### Main Functions Below ######################

def createSingleFrame(i, mapData, allPoses, includePrevious):
    """ Input: frameIndex (int)
        mapData (2D numpy)
        allPoses (list of numpy or 2D numpy)
        includePrevious (boolean)
    Output:
        Returns matplotlib artists for gif creation.
    Note: Feel free to modify this function based off your needs.
    """
    if not includePrevious:
        plt.clf()
    curPose = allPoses[i]
    artists = []  # Need to save plots into artists for animating

    ## Show obstacles, need to transpose as C++ map is transposed as well
    artists.append(plt.imshow(1-mapData.T, cmap=plt.cm.gray))

    ## Base position
    xBase = mapData.shape[0]//2
    yBase = 0
    linkLength = 10

    xs, ys = [xBase], [yBase]
    for i in range(0, len(curPose)):
        xs.append(xs[i] + linkLength*np.cos(curPose[i]))
        ys.append(ys[i] + linkLength*np.sin(curPose[i]))
    xs = np.array(xs)
    ys = np.array(ys)
    artists.append(plt.plot(xs, ys, color="g", linewidth="2"))
    artists.append(plt.plot(xs[0], ys[0], 'go', markersize=10, label='Base Position'))
    artists.append(plt.plot(xs[-1], ys[-1], 'ro', markersize=5, label='End Effector Position'))
    return artists

def viz():
    """ Main function for creating a gif give the solution file path and gif args.
    Note: Feel free to change anything in this function or file. This was made
    solely for your convience. :-)
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("fileWithSolution", help="filepath with solution", type=str, default="output.txt")
    parser.add_argument("--gifFilepath", help="filepath for gif", type=str, default="output.gif", required=False)
    parser.add_argument("--fps", help="frames per second", type=int, default=4, required=False)
    parser.add_argument("--incPrev", help="include previous poses (1), else don't (0). Note this slows gif creation speed",
                                        type=int, default=0, required=False)
    args = parser.parse_args()
    assert(args.gifFilepath.endswith(".gif"))  # Make sure it ends with a .gif extension
    assert(args.incPrev == 0 or args.incPrev == 1)  # 0 is don't include, 1 is include

    with open(args.fileWithSolution) as f:
        line = f.readline().rstrip()  # filepath of the map
        mapData = readMap(line)

        solution = []
        for line in f:
            solution.append(line.split(",")[:-1]) # :-1 to drop trailing comma
        solution = np.asarray(solution).astype(float)

    numFrames = len(solution)

    fig = plt.figure()
    ani = FuncAnimation(fig, createSingleFrame, repeat=False,
        frames=numFrames, fargs=(mapData, solution, args.incPrev))    
    ani.save(args.gifFilepath, dpi=300, writer=PillowWriter(fps=args.fps))

if __name__ == "__main__":
    viz()
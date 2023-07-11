from visualization import *
# from datetime import datetime
import os
import subprocess
import glob
from utils import *
from main import argument_parser


if __name__ == '__main__':
    args, parser = argument_parser()
    rigidObjectList = get_non_articulated_objects()
    isArticulatedObj = False if args.object in rigidObjectList else True
    
    # get folders of the same task
    folderList = []
    # path = './results/ICRA2024/Scenario01Fish-Hook'
    # path = './results/ICRA2024/Scenario02Starfish-HandAndBowl'
    # path = 'results/ICRA2024/Scenario03Mask-Ear'
    # path = 'results/ICRA2024/Scenario04Fish-Shovel'
    path = 'results/ICRA2024/Scenario05Rubic-Bimanual'
    # path = 'results/ICRA2024/Scenario06Bag-Gripper'
    os.chdir(path)
    for folderName in glob.glob(args.scenario + "*"):
        folderList.append(folderName + '/')
    
    # save per-frame pics
    numframes = 401 # starfish
    xmax = 44
    # numframes = 141 # starfish
    # xmax = 35
    # numframes = int(450*17000/18000) # starfish
    # xmax = 84
    # numframes = 472-15+1 # starfish
    # xmax = 84
    # numframes = 163-6+1 # mask
    # xmax = numframes
    # numframes = 399 # shovel
    # xmax = 70
    for i in range(numframes):
        _, ax = plt.subplots(figsize=(6,6))
        plot_escape_energy_from_multi_csv(ax, folderList, isArticulatedObj, axvline=[i*xmax/(numframes-1)])
        plt.savefig("file%03d.png" % i, dpi=300)

    # produce videos
    subprocess.call([
        'ffmpeg', '-framerate', '24', '-i', 'file%03d.png', '-r', '24', '-pix_fmt', 'yuv420p',
        'soft-fixture.mp4'
    ])
    for file_name in glob.glob("file*"):
        os.remove(file_name)
    os.chdir('./../')


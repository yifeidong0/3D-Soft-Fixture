import os.path as osp
import sys
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))
from pbOmplInterface import PbOMPL
from cagingSearchAlgo import RigidObjectCaging, ArticulatedObjectCaging
# from runScenario import runScenario
import pybullet as p
import matplotlib.pyplot as plt
from utils import argument_parser, get_non_articulated_objects

if __name__ == '__main__':
    args = argument_parser()
    basePosBounds=[[-5,5], [-5,5], [-3,5]] # searching bounds

    # create caging environment and items in pybullet
    if args.object in get_non_articulated_objects():
        eps_thres = 1e-2 # threshold of loop terminating
        env = RigidObjectCaging(args, eps_thres)
    else:
        env = ArticulatedObjectCaging(args)

    # set searching bounds and add obstacles
    env.add_obstacles()
    env.pb_ompl_interface = PbOMPL(env.robot, args, env.obstacles)
 
    # Choose from different searching methods
    if args.search == 'BoundShrinkSearch':
        useBisecSearch = True # True: bisection search; False: Conservative search
        env.bound_shrink_search(useBisecSearch)
        escape_energy, z_thres = env.visualize_bound_shrink_search(useBisecSearch) # visualize
        print('final z threshold: {}, escape energy: {}'.format(z_thres, escape_energy))

    elif args.search == 'EnergyMinimizeSearch':
        numInnerIter = 3
        env.energy_minimize_search(numInnerIter)
        env.visualize_energy_minimize_search()
        print('Energy costs of current obstacle and object config: {}'.format(env.sol_final_costs))

    # shut down pybullet (GUI)
    p.disconnect()

    # TODO: comapare the results with ground truth (Open3d OBB - donut model)
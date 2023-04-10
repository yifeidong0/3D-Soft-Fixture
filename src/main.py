import os.path as osp
import sys
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))
from pbOmplInterface import PbOMPL
from cagingSearchAlgo import *
# from runScenario import runScenario
import pybullet as p
import matplotlib.pyplot as plt
from utils import argument_parser, get_non_articulated_objects

if __name__ == '__main__':
    args, parser = argument_parser()
    # basePosBounds=[[-5,5], [-5,5], [-3,5]] # searching bounds

    # create caging environment and items in pybullet
    if args.object in get_non_articulated_objects():
        env = RigidObjectCaging(args)
    elif args.object == 'Fish':
        env = ArticulatedObjectCaging(args)
    elif args.object == 'Band':
        numCtrlPoint = 6
        # start = [-0.2,-0.2,1.8, 0.2,-0.2,1.8, 0,0.4,1.8] 
        start = generate_circle_points(numCtrlPoint, rad=.8, z=0.98)
        goal = [0,0,2.18] * numCtrlPoint
        env = ElasticObjectCaging(args, numCtrlPoint, start, goal)

    # set searching bounds and add obstacles
    env.add_obstacles(scale=[.1]*3, pos=[0,0,0], qtn=p.getQuaternionFromEuler([1.57, 0, 0]))
    env.pb_ompl_interface = PbOMPL(env.robot, args, env.obstacles)
 
    # Choose from different searching methods
    if args.search == 'BoundShrinkSearch':
        useGreedySearch = False # True: bisection search; False: Conservative search
        env.bound_shrink_search(useGreedySearch)
        env.visualize_bound_shrink_search(useGreedySearch) # visualize
        # print('final z threshold: {}, escape energy: {}'.format(z_thres, escape_energy))

    elif args.search == 'EnergyMinimizeSearch':
        numInnerIter = 1
        env.energy_minimize_search(numInnerIter)
        env.visualize_energy_minimize_search()
        print('Energy costs of current obstacle and object config: {}'.format(env.sol_final_costs))

    # shut down pybullet (GUI)
    p.disconnect()

    # TODO: comapare the results with ground truth (Open3d OBB - donut model)
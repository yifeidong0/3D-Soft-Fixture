"""
Title: escape energy analysis with fixed obstacles in a single frame.
Author: Yifei Dong
Date: 14/07/2023
Description: The script provides an interface of running our iterative or energy-biased search algorithms 
and analyzing the escape energy within a single frame of various soft fixture cases.
"""
import os.path as osp
import sys
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))
from cagingSearchAlgo import *
import pybullet as p
from utils import *

if __name__ == '__main__':
    args, parser = argument_parser()

    if args.object == 'Ring':
        start = [0.5798,0.0094,1.391,0,0,0]
        goal = [1.5,0,1.5]+[1.57,1.57,0]
        goalSpaceBounds = [[1.4,2], [-.5,.5], [1.3,2.1]] + [[math.radians(-180), math.radians(180)]] + [[-.1, .1]]*2
        env = RigidObjectCaging(args)
        env.add_obstacles(scale=[.1]*3, pos=[0,0,2], qtn=p.getQuaternionFromEuler([1.57, -0.3, 0]))
        env.robot.set_search_bounds(basePosBounds=[[-.3,2], [-.5,.5], [1.3,3]])
        env.reset_start_and_goal(start=start, goal=goal)
    if args.object == 'FishHole':
        env = RigidObjectCaging(args)
        goalSpaceBounds = [[-1,1], [-.5,3], [1.2,1.3]] + [[math.radians(-180), math.radians(180)]]*3
        env.add_obstacles(scale=[.1]*3, pos=[0.5,1.150,3.60], qtn=[0.223,0.0,0.0,0.974])
        env.robot.set_search_bounds(basePosBounds=[[-3,3], [-1,5], [0,6]])
        objEul = list(p.getEulerFromQuaternion([0.381828,-0.174,-0.06460,0.90529]))
        env.reset_start_and_goal(start=[-0.0522,1.4564,2.3247,]+objEul, goal=[0,3,2]+[0,0,0])
    elif args.object == 'Fish':
        objScale = 1
        env = ArticulatedObjectCaging(args, objScale)
        env.add_obstacles(scale=[1]*3, pos=[0,0,2], qtn=p.getQuaternionFromEuler([0, 0, 0]))
        env.reset_start_and_goal(start=[0,0,3.8]+[0,0,0]+[0]*env.robot.articulate_num, goal=[0,0,.1]+[0,0,0]+[0]*env.robot.articulate_num)
    elif args.object == 'Snaplock':
        objScale = 3
        env = ArticulatedObjectCaging(args, objScale)
        env.add_obstacles(scale=[.1]*3, pos=[-.5,0,3], qtn=p.getQuaternionFromEuler([0, 0, 0])) # ring
        # env.robot.set_search_bounds(basePosBounds=[[-2,2], [-2,2], [0,3.5]])
        env.reset_start_and_goal(start=[0,0,1.8,0,0,1.57]+[0], goal=[0,0,.01]+[0,1.57,0]+[0])
        goalSpaceBounds = [[1.4,2], [-.5,.5], [1.3,2.1]] + [[math.radians(-180), math.radians(180)]] + [[-.1, .1]]*2
    elif args.object == 'Starfish':
        objScale = 1
        env = ArticulatedObjectCaging(args, objScale)
        env.add_obstacles(scale=[1]*3, pos=[0,0,.6], qtn=p.getQuaternionFromEuler([0, 0, 0])) # splash bowl
        # env.robot.set_search_bounds(basePosBounds=[[-2,2], [-2,2], [0,3.5]])
        start = [-0.401751, -0.198753, 1.46589, -0.529868, 0.138347, -0.357632, -0.5, 0.5, -0.17, 0.35, 0.5, 0.5, 0.22, 0.5, 0.5, 0.41]
        env.reset_start_and_goal(start=start, goal=[0,0,.01]+[0,0,1.57]+[0]*env.robot.articulate_num)
    elif args.object == 'Band':
        numCtrlPoint = 6
        start = generate_circle_points(numCtrlPoint, rad=.3, z=1.4)
        goal = generate_circle_points(numCtrlPoint, rad=.3, z=.1)
        goalSpaceBounds = [[-2,2], [-2,2], [0,.1]] + [[-.1,.1], [-.1,.1], [-.1,.1]]
        env = ElasticBandCaging(args, numCtrlPoint, start, goal)
        env.add_obstacles(scale=[1]*3, pos=[0,0,0], qtn=p.getQuaternionFromEuler([0, 0, 0]))
    elif args.object == 'Rope':
        numCtrlPoint = 4
        linkLen = 0.3
        start = [0,0,.7,1.57,0,0] + [math.radians(360/(numCtrlPoint+1)),0]*numCtrlPoint
        goal = [0,0,.1,0,1.57,0] + [math.radians(360/(numCtrlPoint+1)),0]*numCtrlPoint
        env = RopeCaging(args, numCtrlPoint, linkLen, start, goal)
        env.add_obstacles(scale=[1]*3, pos=[0,0,.5], qtn=p.getQuaternionFromEuler([0, 0, 0])) # bucket
    elif args.object == 'Chain':
        numCtrlPoint = 4 # numCtrlPoint+3 links
        linkLen = 0.7
        start = [-.5,0.3,1.,1.,0,1.57] + [math.radians(360/(numCtrlPoint+3)-1),0]*numCtrlPoint + [0]
        goal = [0,0,.1,0,1.57,0] + [math.radians(360/(numCtrlPoint+3)-1),0]*numCtrlPoint + [0]
        goalSpaceBounds = [[-2,2], [-2,2], [0,.1]] + [[-.1,.1], [-.1,.1], [-.1,.1]]
        env = ChainCaging(args, numCtrlPoint, linkLen, start, goal)
        env.robot.set_search_bounds(vis=1, basePosBounds=[[-2,2], [-2,2], [0, 3]])
        env.add_obstacles(scale=[10]*3, pos=[0,-.5,2.2], qtn=p.getQuaternionFromEuler([-1.57, -2, 1.57])) # 3fGripper
    elif args.object == '2Dlock':
        objScale = 1
        basePosBounds = [[-5, 5], [-5, 5]]
        env = SnapLock2DCaging(args, objScale, basePosBounds)
        env.add_obstacles(scale=[1]*3, pos=[1.25,-2.9,0], qtn=p.getQuaternionFromEuler([0, 0, 3.7]))
        env.reset_start_and_goal(start=[-2,2,0,-0.36], goal=[2,2,0,-0.36])
        goalSpaceBounds = [[1.4,2], [-.5,.5], [1.3,2.1]] + [[math.radians(-180), math.radians(180)]] + [[-.1, .1]]*2
    elif args.object == 'Ftennis':
        # all geometries are 10 times large (/m). The final escape energy should /10 and *mg
        if args.obstacle == 'FbowlS':
            start = [0.365,0,0.486,0,0,0] # bowlS, gt: 0.20
        elif args.obstacle == 'FbowlM':
            start = [0.367,0,0.466,0,0,0] # bowlM, gt: 0.37
        else:
            start = [0.367,0,0.508,0,0,0] # bowlL, gt: 0.51
        goal = [1,1,-.5]+[0,0,0]
        goalSpaceBounds = [[1.4,2], [-.5,.5], [1.3,2.1]] + [[math.radians(-180), math.radians(180)]] + [[-.1, .1]]*2
        env = RigidObjectCaging(args)
        env.add_obstacles(scale=[10]*3, pos=[0,0,0], qtn=p.getQuaternionFromEuler([0,0,0]))
        env.robot.set_search_bounds(basePosBounds=[[-1.4,1.4], [-1.4,1.4], [-.6,1]])
        env.reset_start_and_goal(start=start, goal=goal)
    elif args.object == 'Fglue':
        if args.obstacle == 'FbowlS':
            start = [0.571,0,0.428,0,1.013,0] # bowlS, gt: 0.08
        elif args.obstacle == 'FbowlM':
            start = [0.544,0,0.481,0,0.884,0] # bowlM, gt: 0.18
        else:
            start = [0.515,0,0.544,0,0.764,0] # bowlL, gt: 0.28
        goal = [1,1,-.5]+[0,0,0]
        goalSpaceBounds = [[1.4,2], [-.5,.5], [1.3,2.1]] + [[math.radians(-180), math.radians(180)]] + [[-.1, .1]]*2
        env = RigidObjectCaging(args)
        env.add_obstacles(scale=[10]*3, pos=[0,0,0], qtn=p.getQuaternionFromEuler([0,0,0]))
        env.robot.set_search_bounds(basePosBounds=[[-1.4,1.4], [-1.4,1.4], [-.6,1]])
        env.reset_start_and_goal(start=start, goal=goal)
    elif args.object == 'Ftape':
        if args.obstacle == 'FbowlS':
            start = [0.338,0,0.238,0,-0.425,0] # bowlS, gt: 0.24
        elif args.obstacle == 'FbowlM':
            start = [0.337,0,0.251,0,-0.456,0] # bowlM, gt: 0.37
        else:
            start = [0.337,0,0.276,0,-0.456,0] # bowlL, gt: 0.53
        goal = [1,1,-.5]+[0,0,0]
        goalSpaceBounds = [[1.4,2], [-.5,.5], [1.3,2.1]] + [[math.radians(-180), math.radians(180)]] + [[-.1, .1]]*2
        env = RigidObjectCaging(args)
        env.add_obstacles(scale=[10]*3, pos=[0,0,0], qtn=p.getQuaternionFromEuler([0,0,0]))
        env.robot.set_search_bounds(basePosBounds=[[-1.4,1.4], [-1.4,1.4], [-.6,1]])
        env.reset_start_and_goal(start=start, goal=goal)
    elif args.object == 'Fbanana':
        if args.obstacle == 'FbowlS':
            start = [0.142,0,0.235,0,-2.147,0] # bowlS, gt: 0.24
        elif args.obstacle == 'FbowlM':
            start = [0.142,0,0.245,0,-2.147,0] # bowlM, gt: 0.40
        else:
            start = [0.136,0.009,0.265,0,-2.404,0] # bowlL, gt: 0.55
        goal = [1,1,-.5]+[0,0,0]
        goalSpaceBounds = [[1.4,2], [-.5,.5], [1.3,2.1]] + [[math.radians(-180), math.radians(180)]] + [[-.1, .1]]*2
        env = RigidObjectCaging(args)
        env.add_obstacles(scale=[10]*3, pos=[0,0,0], qtn=p.getQuaternionFromEuler([0,0,0]))
        env.robot.set_search_bounds(basePosBounds=[[-1.4,1.4], [-1.4,1.4], [-.6,1]])
        env.reset_start_and_goal(start=start, goal=goal)
    elif args.object == 'Donut45':
        start = [0.050,0.022,0.222,0,0,0] # bowlS, gt: 0.26
        goal = [1,1,-.5]+[0,0,0]
        goalSpaceBounds = [[1.4,2], [-.5,.5], [1.3,2.1]] + [[math.radians(-180), math.radians(180)]] + [[-.1, .1]]*2
        env = RigidObjectCaging(args)
        env.add_obstacles(scale=[10]*3, pos=[0,0,0], qtn=p.getQuaternionFromEuler([0,0,0]))
        env.robot.set_search_bounds(basePosBounds=[[-1.4,1.4], [-1.4,1.4], [-.6,1]])
        env.reset_start_and_goal(start=start, goal=goal)
    elif args.object == 'Donut60':
        start = [0.041,-0.070,0.222,0,0,0] # bowlS, gt: 0.26
        goal = [1,1,-.5]+[0,0,0]
        goalSpaceBounds = [[1.4,2], [-.5,.5], [1.3,2.1]] + [[math.radians(-180), math.radians(180)]] + [[-.1, .1]]*2
        env = RigidObjectCaging(args)
        env.add_obstacles(scale=[10]*3, pos=[0,0,0], qtn=p.getQuaternionFromEuler([0,0,0]))
        env.robot.set_search_bounds(basePosBounds=[[-1.4,1.4], [-1.4,1.4], [-.6,1]])
        env.reset_start_and_goal(start=start, goal=goal)
    elif args.object == 'Donut90':
        start = [0.091,-0.089,0.223,0,0,0] # bowlS, gt: 0.26
        goal = [1,1,-.5]+[0,0,0]
        goalSpaceBounds = [[1.4,2], [-.5,.5], [1.3,2.1]] + [[math.radians(-180), math.radians(180)]] + [[-.1, .1]]*2
        env = RigidObjectCaging(args)
        env.add_obstacles(scale=[10]*3, pos=[0,0,0], qtn=p.getQuaternionFromEuler([0,0,0]))
        env.robot.set_search_bounds(basePosBounds=[[-1.4,1.4], [-1.4,1.4], [-.6,1]])
        env.reset_start_and_goal(start=start, goal=goal)
    elif args.object == 'Donut120':
        start = [0.151,-0.087,0.223,0,0,0] # bowlS, gt: 0.26
        goal = [1,1,-.5]+[0,0,0]
        goalSpaceBounds = [[1.4,2], [-.5,.5], [1.3,2.1]] + [[math.radians(-180), math.radians(180)]] + [[-.1, .1]]*2
        env = RigidObjectCaging(args)
        env.add_obstacles(scale=[10]*3, pos=[0,0,0], qtn=p.getQuaternionFromEuler([0,0,0]))
        env.robot.set_search_bounds(basePosBounds=[[-1.4,1.4], [-1.4,1.4], [-.6,1]])
        env.reset_start_and_goal(start=start, goal=goal)


    env.create_ompl_interface()
    env.pb_ompl_interface.set_goal_space_bounds(goalSpaceBounds)
    if args.object == 'Band':
        springneutralLen = .2
        k_spring = 2/100
        env.pb_ompl_interface.set_spring_params(springneutralLen, k_spring)

    # Choose from different searching methods
    if args.search == 'BisectionSearch':
        # useGreedySearch = False # True: bisection search; False: Conservative search
        env.energy_bisection_search(numIter=15, maxTimeTaken=50, useBisectionSearch=1)
        # env.visualize_bisection_search() # visualize
    elif args.search == 'EnergyBiasedSearch':
        numInnerIter = 1
        env.energy_biased_search(numIter=numInnerIter, save_escape_path=1, )
        E2real = env.sol_final_costs
        # env.visualize_energy_biased_search()
        print('Energy costs of current obstacle and object config: {}'.format(env.sol_final_costs))

    # shut down pybullet (GUI)
    p.disconnect()
import os.path as osp
import sys
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))
from cagingSearchAlgo import *
import pybullet as p
from utils import *
from time import sleep

if __name__ == '__main__':
    args, parser = argument_parser()
    # basePosBounds=[[-5,5], [-5,5], [-3,5]] # searching bounds

    # create caging environment and items in pybullet
    # if args.object in get_non_articulated_objects():
    #     env = RigidObjectCaging(args)
    #     env.add_obstacles(scale=[.1]*3, pos=[0,0,0], qtn=p.getQuaternionFromEuler([1.57, 0, 0]))

    if args.object == 'Ring':
        env = RigidObjectCaging(args)
        env.add_obstacles(scale=[.1]*3, pos=[0,0,2], qtn=p.getQuaternionFromEuler([1.57, -0.3, 0]))
        # env.robot.set_search_bounds([[-2,2], [-2,2], [0,3.5]])
        env.reset_start_and_goal(start=[.3,-.1,2.3,0,0,0], goal=[0,0,.01]+[1.57,0,0])
    if args.object == 'FishHole':
        env = RigidObjectCaging(args)
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
        # env.robot.set_search_bounds([[-2,2], [-2,2], [0,3.5]])
        env.reset_start_and_goal(start=[0,0,1.8,0,0,1.57]+[0], goal=[0,0,.01]+[0,1.57,0]+[0])
    elif args.object == 'Starfish':
        objScale = 1
        env = ArticulatedObjectCaging(args, objScale)
        env.add_obstacles(scale=[1]*3, pos=[0,0,.6], qtn=p.getQuaternionFromEuler([0, 0, 0])) # splash bowl
        # env.add_obstacles(scale=[10]*3, pos=[0,0,1], qtn=p.getQuaternionFromEuler([1.57, 0, 0])) # 3fGripper
        # env.robot.set_search_bounds([[-2,2], [-2,2], [0,3.5]])
        env.reset_start_and_goal(start=[0,0,3,0,0,0]+[0]*env.robot.articulate_num, goal=[0,0,.01]+[0,0,1.57]+[0]*env.robot.articulate_num)
    elif args.object == 'Band':
        numCtrlPoint = 6
        start = generate_circle_points(numCtrlPoint, rad=.3, z=1.4)
        goal = generate_circle_points(numCtrlPoint, rad=.3, z=.1)
        # start = generate_circle_points(numCtrlPoint, rad=.8, z=0.98)
        # goal = generate_circle_points(numCtrlPoint, rad=.2, z=2.18)
        env = ElasticBandCaging(args, numCtrlPoint, start, goal)
        env.add_obstacles(scale=[1]*3, pos=[0,0,0], qtn=p.getQuaternionFromEuler([0, 0, 0]))
    elif args.object == 'Rope':
        numCtrlPoint = 6
        linkLen = 0.2
        start = [0,0,.7,1.57,0,0] + [0,0]*numCtrlPoint
        goal = [0,0,.1,0,0,0] + [0,0]*numCtrlPoint
        env = RopeCaging(args, numCtrlPoint, linkLen, start, goal)
        env.add_obstacles(scale=[1]*3, pos=[0,0,.5], qtn=p.getQuaternionFromEuler([0, 0, 0])) # bucket
    elif args.object == 'Jelly':
        numCtrlPoint = 4
        l = 1
        zs = 1
        ofs = 0.7
        start = [-l/2,-l/2,-l/2+zs] + [-l/2,-l/2,l/2+zs] + [l/2,-l/2,l/2+zs] + [-l/2,l/2,l/2+zs]
        goal = [-l/2-ofs,-l/2-ofs,-l/2+zs] + [-l/2-ofs,-l/2-ofs,l/2+zs] + [l/2-ofs,-l/2-ofs,l/2+zs] + [-l/2-ofs,l/2-ofs,l/2+zs]
        env = ElasticJellyCaging(args, numCtrlPoint, start, goal)
        env.add_obstacles(scale=[1]*3, pos=[0,0,0], qtn=p.getQuaternionFromEuler([0, 0, 0])) # maze
        env.robot.set_search_bounds([[-2,2], [-2,2], [0,3]])
    elif args.object == '2Dlock':
        objScale = 1
        basePosBounds = [[-5, 5], [-5, 5]]
        env = SnapLock2DCaging(args, objScale, basePosBounds)
        env.add_obstacles(scale=[1]*3, pos=[1.25,-2.9,0], qtn=p.getQuaternionFromEuler([0, 0, 3.7]))
        env.reset_start_and_goal(start=[-2,2,0,-0.36], goal=[2,2,0,0])

    # env.pb_ompl_interface = PbOMPL(env.robot, args, env.obstacles)
    env.create_ompl_interface()
    
    # Choose from different searching methods
    if args.search == 'BisectionSearch':
        # useGreedySearch = False # True: bisection search; False: Conservative search
        # env.bound_shrink_search(useGreedySearch)
        env.energy_bisection_search(maxTimeTaken=40)
        env.visualize_bisection_search() # visualize

    elif args.search == 'EnergyBiasedSearch':
        numInnerIter = 1
        env.energy_biased_search(numInnerIter)
        env.visualize_energy_biased_search()
        print('Energy costs of current obstacle and object config: {}'.format(env.sol_final_costs))

    # shut down pybullet (GUI)
    p.disconnect()

    # TODO: comapare the results with ground truth (Open3d OBB - donut model)
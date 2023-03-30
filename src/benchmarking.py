import pybullet as p
import time
import os.path as osp
import sys
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))
from pbOmplInterface import PbOMPL
from cagingSearchAlgo import RigidObjectCaging, ArticulatedObjectCaging
from main import argument_parser
from utils import *
from visualization import *
import numpy as np
from runScenario import runScenario

if __name__ == '__main__':
    # Hyperparameters
    numInnerIter = 6
    frames = list(range(64,68))
    maxTimeTaken = 480
    useGreedySearch = 0 # True: bisection search; False: Conservative search

    # Settings and paths
    args, parser = argument_parser()
    rigidObjectList = get_non_articulated_objects()
    isArticulatedObj = False if args.object in rigidObjectList else True
    folderName = None
    
    # Run a dynamic falling scenario and analyze frame-wise escape energy
    sce = runScenario(args)
    if args.scenario in ['FishFallsInBowl', 'HookTrapsFish', 'HookTrapsRing']:
        sce.runDynamicFalling()
    elif args.scenario in ['GripperClenchesStarfish']:
        sce.runClenchFist()

    # create caging environment and items in pybullet
    if args.object in rigidObjectList:
        eps_thres = 1e-2 # threshold of loop terminating
        env = RigidObjectCaging(args, eps_thres)
    else:
        env = ArticulatedObjectCaging(args)

    # set searching bounds and add obstacles
    env.robot.set_search_bounds(sce.basePosBounds)
    env.add_obstacles(sce.obsBasePosSce[0], sce.obsBaseQtnSce[0], sce.obstacleScale, sce.obsJointPosSce[0])
    
    # Run the caging analysis algorithm over downsampled frames we extracted above
    for i in frames:
        print('@@@@@index: ', sce.idxSce[i])
        frameId = sce.idxSce[i]

        # Set obstacle's state
        if args.scenario in ['GripperClenchesStarfish']:
            env.obstacle._set_joint_positions(env.obstacle.joint_idx, sce.obsJointPosSce[i])
            p.resetBasePositionAndOrientation(env.obstacle_id, sce.obsBasePosSce[i], sce.obsBaseQtnSce[i])

        # Set object's start and goal states
        objStartState = sce.objBasePosSce[i] + sce.objBaseEulSce[i] + sce.objJointPosSce[i]
        objGoalState = sce.goalCoMPose + [0]*env.robot.articulate_num
        isValidStartAndGoal = env.reset_start_and_goal(objStartState, objGoalState)
        if not isValidStartAndGoal: # start or goal state invalid
            continue

        # Create OMPL interface
        env.pb_ompl_interface = PbOMPL(env.robot, args, env.obstacles)

        # Choose a searching method
        if args.search == 'BoundShrinkSearch':
            env.bound_shrink_search(useGreedySearch, initSearchBound=sce.basePosBounds, numIter=numInnerIter, maxTimeTaken=maxTimeTaken)
            
            # # Visualization
            # env.visualize_bound_shrink_search(useGreedySearch) # visualize
            
            # Create new folder
            createFolder = 1 if i == frames[0] else 0
            if createFolder:
                now = datetime.now()
                dt_string = now.strftime("%d-%m-%Y-%H-%M-%S") # dd/mm/YY H:M:S
                folderName = './results/Benchmarking/{}'.format(dt_string)
                os.mkdir(folderName)

            # Record data to the folder
            record_data_benchmark_bound_shrink(env.escape_cost_list_runs, env.time_taken_list_runs, frameId, folderName)

        elif args.search == 'EnergyMinimizeSearch':
            isSolved = env.energy_minimize_search(numInnerIter)
            # env.visualize_energy_minimize_search()
            print('Energy costs of current obstacle and object config: {}'.format(env.sol_final_costs))

    # Shut down pybullet (GUI)
    p.disconnect()
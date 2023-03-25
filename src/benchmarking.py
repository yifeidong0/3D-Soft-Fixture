import pybullet as p
import time
import os.path as osp
import sys
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))
from pbOmplInterface import PbOMPL
from cagingSearchAlgo import RigidObjectCaging, ArticulatedObjectCaging
from main import argument_parser
import pybullet_data
from utils import *
from object import CagingObstacle
from visualization import *
import numpy as np
from runScenario import runScenario

if __name__ == '__main__':
    args, parser = argument_parser()
    rigidObjectList = get_non_articulated_objects()
    isArticulatedObj = False if args.object in rigidObjectList else True

    # run a dynamic falling scenario and analyze frame-wise escape energy
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
    # escapeEnergyCostSce = []
    # startEnergySce = [] # start state energy
    # startGEnergySce = [] # start G potential energy
    # startEEnergySce = [] # start E potential energy
    
    # Run the caging analysis algorithm over downsampled frames we extracted above
    numMainIter = len(sce.objJointPosSce)
    for i in [50]:
        print('@@@@@index: ', sce.idxSce[i])
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
            useGreedySearch = 0 # True: bisection search; False: Conservative search
            env.bound_shrink_search(useGreedySearch, initSearchBound=sce.basePosBounds, numIter=10)
            env.visualize_bound_shrink_search(useGreedySearch) # visualize
            # print('final z threshold: {}, escape energy: {}'.format(z_thres, escape_energy))

        elif args.search == 'EnergyMinimizeSearch':
            numInnerIter = 1
            isSolved = env.energy_minimize_search(numInnerIter)
            # env.visualize_energy_minimize_search()
            print('Energy costs of current obstacle and object config: {}'.format(env.sol_final_costs))

            # # Record start and escape energy
            # if isArticulatedObj:
            #     startGEnergy = env.pb_ompl_interface.potentialObjective.getGravityEnergy(objStartState)
            #     startEEnergy = env.pb_ompl_interface.potentialObjective.getElasticEnergy(objStartState)
            #     startEnergy = startGEnergy + startEEnergy
            # else: # non-articulated objects' z_world
            #     # startEnergy = env.energy_minimize_paths_energies[0][0] if isSolved else np.inf
            #     startGEnergy, startEEnergy = None, None
            #     startEnergy = objStartState[2]
            # startEnergySce.append(startEnergy)
            # startGEnergySce.append(startGEnergy)
            # startEEnergySce.append(startEEnergy)
            
            # escapeEnergyCost = min(env.sol_final_costs) if isSolved else np.inf
            # escapeEnergyCostSce.append(escapeEnergyCost) # list(numMainIter*list(numInnerIter))
            
            # # Create txt, csv for data recording
            # if i == 0:
            #     folderName = record_data_init(sce, args, env)
            # # print('@@@i: ',i)
            
            # # Record data in this loop 
            # energyData = [startEnergy, startGEnergy, startEEnergy, escapeEnergyCost]
            # record_data_loop(sce, energyData, folderName, i)
            # # print('@@@Initial state energy: {}, Energy costs of current obstacle and object config: {}'.format(startEnergy,escapeEnergyCost))

    # Shut down pybullet (GUI)
    p.disconnect()
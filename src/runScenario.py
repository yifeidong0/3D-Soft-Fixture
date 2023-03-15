import pybullet as p
import time
import os.path as osp
import sys
import argparse
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))
from pbOmplInterface import PbOMPL
from cagingSearchAlgo import RigidObjectCaging, ArticulatedObjectCaging
import matplotlib.pyplot as plt
from main import argument_parser
import pybullet_data
from utils import path_collector, get_non_articulated_objects
from object import ObjectToCage, CagingObstacle

class runScenario():
    def __init__(self, args):
        p.connect(p.GUI)
        p.setTimeStep(1./240.)
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setRealTimeSimulation(0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # planeId = p.loadURDF("plane.urdf", [0,0,0.4])
        self.paths = path_collector()
        self.args = args
        self.gravity = -9.8
        self.downsampleRate = 100
        self.endFrame = 800

        # load object and obstacle
        self.initializeParams()
        self.loadObject()
        self.loadObstacle()

        # data structure for object and obstacle configs
        self.objBasePosSce = []
        self.objBaseQtnSce = []
        self.objJointPosSce = []
        self.obsBasePosSce = []
        self.obsBaseQtnSce = []
        self.obsJointPosSce = []

    def initializeParams(self):
        match self.args.scenario:
            case 'FishFallsInBowl':
                self.object = 'Fish'
                self.objectPos = [0,0,3]
                self.objectQtn = [0,1,0,1]
                self.objectEul = list(p.getEulerFromQuaternion(self.objectQtn))
                self.obstacle = 'Bowl'
                self.obstaclePos = [-0.5, 0.5, 0]
                self.obstacleEul = [0, 0, 0]
                self.obstacleQtn = list(p.getQuaternionFromEuler(self.obstacleEul))
                self.obstacleScale = [.1, .1, .1]
                self.basePosBounds = [[-2,2], [-2,2], [0,3]] # searching bounds
                self.goalCoMPose = [0,0,0.01] + [0]*3
            case 'HookTrapsFish':
                self.object = 'FishWithRing'
                self.objectPos = [.7,-2.1,3.4]
                self.objectQtn = [0,1,0,1]
                self.objectEul = list(p.getEulerFromQuaternion(self.objectQtn))
                self.obstacle = 'Hook'
                self.obstaclePos = [0, 0, 2]
                self.obstacleEul = [1.57, 0, 0]
                self.obstacleQtn = list(p.getQuaternionFromEuler(self.obstacleEul))
                self.obstacleScale = [.1,.1,.1]
                self.basePosBounds=[[-2,2], [-2.5,2.5], [-0.5,3.5]] # searching bounds
                self.goalCoMPose = [0,0,-0.49] + [0]*3
            case 'HookTrapsRing':
                self.object = 'Ring'
                self.objectPos = [1.3,-.1,3.4]
                self.objectQtn = [0,1,0,1]
                self.objectEul = list(p.getEulerFromQuaternion(self.objectQtn))
                self.obstacle = 'Hook'
                self.obstaclePos = [0, 0, 2]
                self.obstacleEul = [1.57, 0, 0]
                self.obstacleQtn = list(p.getQuaternionFromEuler(self.obstacleEul))
                self.obstacleScale = [.1,.1,.1]
                self.basePosBounds=[[-3,3], [-3,3], [-1,3.5]] # searching bounds
                self.goalCoMPose = [0,0,-0.9] + [1.57, 0, 0]
            case 'GripperClenchesStarfish':
                self.object = 'Starfish'
                self.objectPos = [.1,.1,2.3]
                self.objectEul = [-1.57, -2.57, -0.5]
                self.objectQtn = list(p.getQuaternionFromEuler(self.objectEul))
                self.obstacle = '3fGripper'
                self.obstaclePos = [0, 0, 1]
                self.obstacleEul = [1.57, 0, 0]
                self.obstacleQtn = list(p.getQuaternionFromEuler(self.obstacleEul))
                self.obstacleScale = 10.0 # float for loadURDF globalScaling
                self.basePosBounds=[[-3,3], [-3,3], [-1,3.5]] # searching bounds
                self.goalCoMPose = [0,0,-0.9] + [1.57, 0, 0]
   
    def loadObject(self):
        # p.changeDynamics(bowl, -1, mass=0)
        self.objectId = p.loadURDF(self.paths[self.args.object], self.objectPos, self.objectQtn)

    def loadObstacle(self):
        obst = self.args.obstacle
        if obst == 'Bowl' or obst == 'Hook':
            mesh_collision_shape = p.createCollisionShape(
                shapeType=p.GEOM_MESH,
                fileName=self.paths[self.args.obstacle],
                meshScale=self.obstacleScale,
                flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
            )
            # mesh_visual_shape = p.createVisualShape(shapeType=p.GEOM_MESH,
            #     fileName="models/bowl/small_bowl.obj",
            #     rgbaColor=[1, 1, 1, 1],
            #     specularColor=[0.4, .4, 0],
            #     # visualFramePosition=shift,
            #     meshScale=mesh_scale
            # )
            mesh_visual_shape = -1  # Use the same shape for visualization
            self.obstacleId = p.createMultiBody(
                # baseMass=1.,
                baseCollisionShapeIndex=mesh_collision_shape,
                baseVisualShapeIndex=mesh_visual_shape,
                basePosition=self.obstaclePos,
                baseOrientation=self.obstacleQtn,
            )
        elif obst == '3fGripper':
            self.obstacleId = p.loadURDF(fileName=self.paths[self.args.obstacle], 
                                          basePosition=self.obstaclePos, 
                                          baseOrientation=self.obstacleQtn, 
                                          globalScaling=self.obstacleScale
                                          )
            self.obstacle = CagingObstacle(self.obstacleId)
            print('@@@self.articulate_num', self.obstacle.articulate_num)
            # print('@@@self.joint_bounds', self.obstacle.joint_bounds)
            # self.obstacle.set_state(self.start)

    def getJointStates(self, id):
        numJoints = p.getNumJoints(id)
        if numJoints == 0:  # rigid object with no joints
            return [], [], []
        joint_states = p.getJointStates(id, range(numJoints))
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        
        return joint_positions, joint_velocities, joint_torques

    def runClenchFist(self):
        '''For the task of gripper clenching starfish'''
        i = 0
        # jbounds = self.obstacle.joint_bounds
        # time.sleep(1)
        # print('@@@initial joint positions: ', self.getJointStates(self.obstacleId))
        self.obstaclePose = self.obstaclePos + self.obstacleEul

        # start simulation of clenching the fist
        while (1):
            p.stepSimulation()
            p.setGravity(0, 0, self.gravity)
            time.sleep(1/240.)
            
            # get obstacle joint positions and update them
            jointPositions,_,_ = self.getJointStates(self.obstacleId) # list(12)
            # obstacleJointPos = [j+1/1000 for j in jointPositions]
            obstacleJointPos = [jointPositions[i]+1/1000 if (i==1 or i==5 or i==9) else jointPositions[i] for i in range(len(jointPositions))]
            obstacleState = self.obstaclePose + obstacleJointPos
            self.obstacle.set_state(obstacleState)

            i += 1
            if i % self.downsampleRate == 0 and i > 140:
                # record obstacle pose
                self.obsJointPosSce.append(obstacleJointPos)

                # get object pose and record
                gemPos, gemQtn = p.getBasePositionAndOrientation(self.objectId) # tuple(3), tuple(4)
                self.objBasePosSce.append(list(gemPos))
                self.objBaseQtnSce.append(list(gemQtn))
                self.objJointPosSce.append([])
            
            if i == self.endFrame:
                p.disconnect()
                break

        # record obstacle poses - unchanged
        sceLen = len(self.objBasePosSce)
        self.obsBasePosSce = [self.obstaclePos for i in range(sceLen)]
        self.obsBaseQtnSce = [self.obstacleQtn for i in range(sceLen)]
        self.obsBaseEulSce = [self.obstacleEul for i in range(sceLen)]

        # quaternion to euler
        self.objBaseEulSce = [list(p.getEulerFromQuaternion(q)) for q in self.objBaseQtnSce]

    def runDynamicFalling(self):
        '''For the tasks of articulated fish or ring falling'''
        i = 0        
        # time.sleep(1)
        while (1):
            p.stepSimulation()
            p.setGravity(0, 0, self.gravity)
            time.sleep(1/240.)

            i += 1
            if i % self.downsampleRate == 0:
                jointPositions,_,_ = self.getJointStates(self.objectId) # list(11)
                gemPos, gemQtn = p.getBasePositionAndOrientation(self.objectId) # tuple(3), tuple(4)

                # record objects' DoF
                self.objBasePosSce.append(list(gemPos))
                self.objBaseQtnSce.append(list(gemQtn))
                self.objJointPosSce.append(jointPositions)

            if i == self.endFrame:
                p.disconnect()
                break

        # record obstacles' DoF
        sceLen = len(self.objBasePosSce)
        self.obsBasePosSce = [self.obstaclePos for i in range(sceLen)]
        self.obsBaseQtnSce = [self.obstacleQtn for i in range(sceLen)]
        self.obsBaseEulSce = [self.obstacleEul for i in range(sceLen)]
        self.obsJointPosSce = [[] for i in range(sceLen)]

        # quaternion to euler
        self.objBaseEulSce = [list(p.getEulerFromQuaternion(q)) for q in self.objBaseQtnSce]


if __name__ == '__main__':
    args = argument_parser()
    rigidObjectList = get_non_articulated_objects()

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
    bestCostSce = []
    startEnergySce = [] # start state energy
    
    # run the caging analysis algorithm over downsampled frames we extracted above
    numMainIter = len(sce.objJointPosSce)
    for i in range(numMainIter):
        # set obstacle's state
        # obsState = sce.obsBasePosSce[i] + sce.obsBaseEulSce[i] + sce.obsJointPosSce[i]
        if args.scenario in ['GripperClenchesStarfish']:
            env.obstacle._set_joint_positions(env.obstacle.joint_idx, sce.obsJointPosSce[i])
            p.resetBasePositionAndOrientation(env.obstacle_id, sce.obsBasePosSce[i], sce.obsBaseQtnSce[i])

        # set object's start and goal states
        objStartState = sce.objBasePosSce[i] + sce.objBaseEulSce[i] + sce.objJointPosSce[i]
        objGoalState = sce.goalCoMPose + [0]*env.robot.articulate_num
        isValidStartAndGoal = env.reset_start_and_goal(objStartState, objGoalState)
        if not isValidStartAndGoal:
            continue
        # objBaseZs.append(start[2])
        # print('Current object z_world: {}'.format(start[2]))

        env.pb_ompl_interface = PbOMPL(env.robot, args, env.obstacles)

        # Choose a searching method
        if args.search == 'BoundShrinkSearch':
            useBisecSearch = True # True: bisection search; False: Conservative search
            env.bound_shrink_search(useBisecSearch)
            escape_energy, z_thres = env.visualize_bound_shrink_search(useBisecSearch) # visualize
            print('final z threshold: {}, escape energy: {}'.format(z_thres, escape_energy))

        elif args.search == 'EnergyMinimizeSearch':
            numInnerIter = 1
            isSolved = env.energy_minimize_search(numInnerIter)
            if not isSolved:
                continue
            # env.visualize_energy_minimize_search()
            bestCost = min(env.sol_final_costs)
            startCost = env.energy_minimize_paths_energies[0][0]
            bestCostSce.append(bestCost) # list(numMainIter*list(numInnerIter))
            startEnergySce.append(startCost)
            print('@@@Initial state energy: {}, Energy costs of current obstacle and object config: {}'.format(startCost,bestCost))

    # shut down pybullet (GUI)
    p.disconnect()

    # plot escape energy in the dynamic fall
    _, ax1 = plt.subplots()
    ax1.plot(bestCostSce, 'r--', label='Escape energy')
    ax1.plot(startEnergySce, 'g-*', label='Current energy')
    ax1.set_xlabel('# iterations')
    ax1.set_ylabel('G-potential energy')
    ax1.grid(True)
    ax1.legend()
    plt.title('Escape energy in a dynamic scenario - fish falls into a bowl')
    # plt.show()
    plt.savefig('./images/fishFalls3.png')

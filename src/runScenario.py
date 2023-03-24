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

class runScenario():
    def __init__(self, args):
        p.connect(p.GUI)
        p.setTimeStep(1./240.)
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setRealTimeSimulation(0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        planeId = p.loadURDF("plane.urdf", [0,0,-1])
        self.paths = path_collector()
        self.pathsTex = texture_path_list()
        self.args = args
        self.gravity = -9.81
        self.downsampleRate = 300
        self.endFrame = 500

        # load object and obstacle
        self.initializeParams()
        self.loadObject()
        self.loadObstacle()

        # data structures for object and obstacle configs
        self.objBasePosSce = []
        self.objBaseQtnSce = []
        self.objJointPosSce = []
        self.obsBasePosSce = []
        self.obsBaseQtnSce = []
        self.obsJointPosSce = []
        self.idxSce = []

    def initializeParams(self):
        match self.args.scenario:
            case 'FishFallsInBowl':
                self.object = 'Fish'
                self.objectPos = [0,0,2.4]
                self.objectQtn = [0,1,0,1]
                self.objectEul = list(p.getEulerFromQuaternion(self.objectQtn))
                self.obstacle = 'Bowl'
                self.obstaclePos = [-0.5, 0.5, 0]
                self.obstacleEul = [0, 0, 0]
                self.obstacleQtn = list(p.getQuaternionFromEuler(self.obstacleEul))
                self.obstacleScale = [.1, .1, .1]
                self.basePosBounds = [[-2,2], [-2,2], [0,3]] # searching bounds
                self.goalCoMPose = [0,0,0.01] + [0]*3
            # case 'HookTrapsFish':
            #     self.object = 'FishWithRing'
            #     self.objectPos = [.7,-2.1,3.4]
            #     self.objectQtn = [0,1,0,1]
            #     self.objectEul = list(p.getEulerFromQuaternion(self.objectQtn))
            #     self.obstacle = 'Hook'
            #     self.obstaclePos = [0, 0, 2]
            #     self.obstacleEul = [1.57, 0, 0]
            #     self.obstacleQtn = list(p.getQuaternionFromEuler(self.obstacleEul))
            #     self.obstacleScale = [.1,.1,.1]
            #     self.basePosBounds=[[-2,2], [-2.5,2.5], [-0.5,3.5]] # searching bounds
            #     self.goalCoMPose = [0,0,-0.49] + [0]*3
            case 'HookTrapsRing':
                self.object = 'Ring'
                self.objectPos = [1.3,-.1,3.4]
                self.objectQtn = [0,1,0,1]
                self.objectEul = list(p.getEulerFromQuaternion(self.objectQtn))
                self.obstacle = 'Hook'
                self.obstaclePos = [0, 0, 2]
                self.obstacleEul = [1.57, -0.3, 0]
                self.obstacleQtn = list(p.getQuaternionFromEuler(self.obstacleEul))
                self.obstacleScale = [.1,.1,.1]
                self.basePosBounds=[[-2,2], [-2,2], [0,3.5]] # searching bounds
                self.goalCoMPose = [0,0,.01] + [1.57, 0, 0]
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
                self.basePosBounds=[[-2,2], [-2,2], [-.5,3]] # searching bounds
                self.goalCoMPose = [0,0,-0.4] + [1.57, 0, 0]
   
    def loadObject(self):
        # p.changeDynamics(bowl, -1, mass=0)
        self.objectId = p.loadURDF(self.paths[self.args.object], self.objectPos, self.objectQtn)
        # texUid = p.loadTexture(self.pathsTex[self.object])
        # p.changeVisualShape(self.objectId, -1, textureUniqueId=texUid)

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

        # Load texture file - png
        # texObsId = p.loadTexture(self.pathsTex[self.obstacle])
        # p.changeVisualShape(self.obstacleId, -1, textureUniqueId=texObsId)

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

        # set initial joint states
        jointPositions,_,_ = self.getJointStates(self.obstacleId) # list(12)
        # obstacleJointPos = [j+1/1000 for j in jointPositions]
        obstacleJointPos = [jointPositions[i]-450/1000 if (i==1 or i==5 or i==9) else jointPositions[i] for i in range(len(jointPositions))]
        obstacleState = self.obstaclePose + obstacleJointPos
        self.obstacle.set_state(obstacleState)
        
        # start simulation of clenching the fist
        while (1):
            p.stepSimulation()
            p.setGravity(0, 0, self.gravity)
            time.sleep(1/240.)
            
            # get obstacle joint positions and update them
            jointPositions,_,_ = self.getJointStates(self.obstacleId) # list(12)
            # obstacleJointPos = [j+1/1000 for j in jointPositions]
            obstacleJointPos = [jointPositions[i]+.8/1000 for i in range(len(jointPositions))]
            obstacleState = self.obstaclePose + obstacleJointPos
            self.obstacle.set_state(obstacleState)

            if i % self.downsampleRate == 0:
                # record obstacle pose
                self.obsJointPosSce.append(obstacleJointPos)

                # get object pose and record
                gemPos, gemQtn = p.getBasePositionAndOrientation(self.objectId) # tuple(3), tuple(4)
                self.objBasePosSce.append(list(gemPos))
                self.objBaseQtnSce.append(list(gemQtn))
                self.objJointPosSce.append([])
                self.idxSce.append(i)
            
            if i == self.endFrame:
                p.disconnect()
                break
            i += 1

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
            # print(i)
            p.stepSimulation()
            p.setGravity(0, 0, self.gravity)
            time.sleep(1/240.)

            if i % self.downsampleRate == 0:
                jointPositions,_,_ = self.getJointStates(self.objectId) # list(11)
                gemPos, gemQtn = p.getBasePositionAndOrientation(self.objectId) # tuple(3), tuple(4)

                # Calculate G potential energy
                # state = list(gemPos) + list(p.getEulerFromQuaternion(gemQtn)) + list(jointPositions)
                # gravityEnergy = getGravityEnergy(state, self.args, self.paths)
                # print('@@@gravityEnergy', gravityEnergy)

                # record objects' DoF
                self.objBasePosSce.append(list(gemPos))
                self.objBaseQtnSce.append(list(gemQtn))
                self.objJointPosSce.append(jointPositions)
                self.idxSce.append(i)

            if i == self.endFrame:
                p.disconnect()
                break
            i += 1

        # record obstacles' DoF
        sceLen = len(self.objBasePosSce)
        self.obsBasePosSce = [self.obstaclePos for i in range(sceLen)]
        self.obsBaseQtnSce = [self.obstacleQtn for i in range(sceLen)]
        self.obsBaseEulSce = [self.obstacleEul for i in range(sceLen)]
        self.obsJointPosSce = [[] for i in range(sceLen)]

        # quaternion to euler
        self.objBaseEulSce = [list(p.getEulerFromQuaternion(q)) for q in self.objBaseQtnSce]


if __name__ == '__main__':
    for n in range(1):
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
        escapeEnergyCostSce = []
        startEnergySce = [] # start state energy
        startGEnergySce = [] # start G potential energy
        startEEnergySce = [] # start E potential energy
        
        # Run the caging analysis algorithm over downsampled frames we extracted above
        numMainIter = len(sce.objJointPosSce)
        for i in range(numMainIter):
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
                useBisecSearch = True # True: bisection search; False: Conservative search
                env.bound_shrink_search(useBisecSearch)
                escape_energy, z_thres = env.visualize_bound_shrink_search(useBisecSearch) # visualize
                # print('final z threshold: {}, escape energy: {}'.format(z_thres, escape_energy))

            elif args.search == 'EnergyMinimizeSearch':
                numInnerIter = 1
                isSolved = env.energy_minimize_search(numInnerIter)
                # env.visualize_energy_minimize_search()

                # Record start and escape energy
                if isArticulatedObj:
                    startGEnergy = env.pb_ompl_interface.potentialObjective.getGravityEnergy(objStartState)
                    startEEnergy = env.pb_ompl_interface.potentialObjective.getElasticEnergy(objStartState)
                    startEnergy = startGEnergy + startEEnergy
                else: # non-articulated objects' z_world
                    # startEnergy = env.energy_minimize_paths_energies[0][0] if isSolved else np.inf
                    startGEnergy, startEEnergy = None, None
                    startEnergy = objStartState[2]
                startEnergySce.append(startEnergy)
                startGEnergySce.append(startGEnergy)
                startEEnergySce.append(startEEnergy)
                
                escapeEnergyCost = min(env.sol_final_costs) if isSolved else np.inf
                escapeEnergyCostSce.append(escapeEnergyCost) # list(numMainIter*list(numInnerIter))
                
                # Create txt, csv for data recording
                if i == 0:
                    folderName = record_data_init(sce, args, env)
                # print('@@@i: ',i)
                
                # Record data in this loop 
                energyData = [startEnergy, startGEnergy, startEEnergy, escapeEnergyCost]
                record_data_loop(sce, energyData, folderName, i)
                # print('@@@Initial state energy: {}, Energy costs of current obstacle and object config: {}'.format(startEnergy,escapeEnergyCost))

        # Shut down pybullet (GUI)
        p.disconnect()
        # del(sce)
        # del(env)
        # time.sleep(300)

        # # Plot
        # energyData = (startEnergySce, startGEnergySce, startEEnergySce, escapeEnergyCostSce)
        # plot_escape_energy(energyData, args, folderName, isArticulatedObj)
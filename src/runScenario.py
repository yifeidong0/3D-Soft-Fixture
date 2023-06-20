import pybullet as p
import time
import os.path as osp
import sys
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))
from pbOmplInterface import PbOMPL
from cagingSearchAlgo import *
from main import argument_parser
import pybullet_data
from utils import *
from object import obstascle3fGripper
from visualization import *
import numpy as np

class runScenario():
    def __init__(self, args):
        p.connect(p.GUI)
        p.setTimeStep(1./240.)
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setRealTimeSimulation(0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # planeId = p.loadURDF("plane.urdf", [0,0,-1])
        self.paths = path_collector()
        self.pathsTex = texture_path_list()
        self.args = args
        self.gravity = -9.81
        self.downsampleRate = 1

        # load object and obstacle
        self.initializeParams()
        self.loadObject()
        self.loadObstacle()

        # data structures for object and obstacle configs
        self.objBasePosSce = []
        self.objBaseQtnSce = []
        self.objBaseEulSce = []
        self.objJointPosSce = []
        self.obsBasePosSce = []
        self.obsBaseQtnSce = []
        self.obsBaseEulSce = []
        self.obsJointPosSce = []
        self.idxSce = []

    def initializeParams(self):
        # match self.args.scenario:
        #     case 'HookFishHole':
        #         self.object = 'FishHole'
        #         self.objectPos = [.0,-.4,1.4]
        #         self.objectEul = [3.14,3.14,3.14]
        #         self.objectQtn = list(p.getQuaternionFromEuler(self.objectEul)) # XYZW
        #         self.obstacle = 'Hook'
        #         self.obstaclePos = [0.5, -.35, 3]
        #         self.obstacleEul = [-.6, 0, 0]
        #         self.obstacleQtn = list(p.getQuaternionFromEuler(self.obstacleEul))
        #         self.obstacleScale = [.1,.1,.1]
        #         self.basePosBounds=[[-.3,.3], [.5,2.5], [1.8,3]]
        #         self.goalCoMPose = [0,2,1.9] + [0, 0, 0]
        #         self.goalSpaceBounds = [[-.3,.3], [.5,2.5], [1.8,1.83]] + [[math.radians(-10), math.radians(10)]]*3
        #         self.startFrame = 12000
        #         self.endFrame = 18000
        #         self.downsampleRate = 200
        #         self.half_box_size = [1,2.5,.1]
        #     case 'ShovelFish':
        #         self.object = 'Fish'
        #         self.objectPos = [0,-1.3,1.7]
        #         self.objectEul = [3.14,3.14,3.14]
        #         self.objectQtn = list(p.getQuaternionFromEuler(self.objectEul)) # XYZW
        #         self.obstacle = 'Shovel'
        #         self.obstaclePos = [0, 6, 1.75]
        #         self.obstacleEul = [-.1, 0, 3.14]
        #         self.obstacleQtn = list(p.getQuaternionFromEuler(self.obstacleEul))
        #         self.obstacleScale = [1]*3
        #         self.handScale = [1,1,1]
        #         self.handPos = [.5,-1.6,1.8]
        #         self.handQtn = list(p.getQuaternionFromEuler([-1.57,.2,-2.2]))
        #         self.basePosBounds = [[-2,2], [-1.5,1], [0,2]] # [[-2,2], [-1.5,1], [0,2]] # [[-1,1], [-1,1], [1.5,7]]
        #         self.goalCoMPose = [1.5,0,.5] + [0, 0, 0] # [1.5,0,.5] + [0, 0, 0] # [0,0.8,1.51] + [0, 0, 0]
        #         self.goalSpaceBounds = [[-.3,.3], [.5,2.5], [1.8,1.83]] + [[-.1,.1]]*9
        #         self.startFrame = 5200 # 0 # 28000
        #         self.endFrame = 28000 # 28000 # 40000
        #         self.downsampleRate = 650 # 500
        #         self.boxBasePos = [0,0,1]
        #         self.half_box_size = [1,2.5,.1]
        #     case 'StarfishBowl':
        #         self.object = 'Starfish'
        #         self.objectPos = [0,1.5,3.5]
        #         self.objectEul = [0,0,0]
        #         self.objectQtn = list(p.getQuaternionFromEuler(self.objectEul)) # XYZW
        #         self.obstacle = 'LeftHandAndBowl'
        #         self.obstaclePos = [0,0,1]
        #         self.obstacleEul = [-.3, 0, 0]
        #         self.obstacleQtn = list(p.getQuaternionFromEuler(self.obstacleEul))
        #         self.obstacleScale = [1, 1, 1]
        #         self.basePosBounds = [[-1.5,1.5], [-2,2], [0,3.51]]
        #         self.goalCoMPose = [0,1,0.01] + [0]*3
        #         self.goalSpaceBounds = [[-2,2], [-2,2], [0,.1]] + [[-.1,.1], [-.1,.1], [-.1,.1]] + [[math.radians(-10), math.radians(10)]]*10
        #         self.startFrame = 0
        #         self.endFrame = 3400
        #         self.downsampleRate = 50
        #         self.boxBasePos = [0,1.5,3]
        #         self.boxBaseEul = [0,0,0]
        #         self.boxBaseQtn = list(p.getQuaternionFromEuler(self.boxBaseEul))
        #         self.half_box_size = [1,1,.05]
        #     case 'BimanualRubic':
        #         self.object = 'Rubic'
        #         self.objectPos = [0,0,2]
        #         self.objectEul = [0,0,0]
        #         self.objectQtn = list(p.getQuaternionFromEuler(self.objectEul)) # XYZW
        #         self.obstacle = '3fGripper'
        #         self.obstaclePos = [0,0,1]
        #         self.obstacleEul = [1.57, 0, 0]
        #         self.obstacleQtn = list(p.getQuaternionFromEuler(self.obstacleEul))
        #         self.obstaclePos1 = [0,0,4]
        #         self.obstacleEul1 = [-1.57, 0, 1.57,]
        #         self.obstacleQtn1 = list(p.getQuaternionFromEuler(self.obstacleEul))
        #         self.obstacleScale = [10.0,]*3
        #         self.basePosBounds = [[-2,2], [-2,2], [0,4]]
        #         self.goalCoMPose = [0,1,0.01] + [0]*3
        #         self.endFrame = 4000
        #         self.downsampleRate = 10
        #     case 'HandbagGripper':
        #         self.object = 'Chain'
        #         self.numCtrlPoint = 4 # numChainLink = numChainNode = numCtrlPoint+3
        #         self.linkLen = 0.85
        #         self.objectPos = [-0.05,0.3,.3]
        #         self.objectEul = [.5,0,1.57]
        #         self.objectQtn = list(p.getQuaternionFromEuler(self.objectEul))
        #         self.objectJointPos = [math.radians(360/(self.numCtrlPoint+3)-1),0]*self.numCtrlPoint + [-1.57]
        #         self.objectStart = self.objectPos + self.objectEul + self.objectJointPos
        #         self.goalCoMPose = [0,0,.1,] + [0,1.57,0]
        #         self.objectGoal = self.goalCoMPose + self.objectJointPos
        #         self.obstacle = '3fGripper'
        #         self.obstaclePos = [0,-.5,2.2]
        #         self.obstacleEul = [-1.57, -2, 1.57]
        #         # self.obstaclePos = [0,-1,2.5]
        #         # self.obstacleEul = [0, -1.57, 0]
        #         self.obstacleQtn = list(p.getQuaternionFromEuler(self.obstacleEul))
        #         self.obstacleScale = [10.0,]*3
        #         self.basePosBounds = [[-2,2], [-2,2], [0,3]]
        #         self.endFrame = 347
        #         self.downsampleRate = 1
        #     case 'MaskEar':
        #         self.object = 'MaskBand'
        #         self.numCtrlPoint = 6 # numChainLink = numChainNode = numCtrlPoint+3
        #         self.objectStart = [0,0,0] * self.numCtrlPoint
        #         self.objectGoal = [] # [0,0,.5] * self.numCtrlPoint
        #         self.obstacle = 'Ear'
        #         self.obstaclePos = [0,0,0]
        #         self.obstacleEul = [0,0,0]
        #         self.obstacleQtn = list(p.getQuaternionFromEuler(self.obstacleEul))
        #         self.obstacleScale = [1.0,]*3
        #         self.basePosBounds = [[0.55,1], [-.5,.3], [-.4,.5]]
        #         self.startFrame = 120 # 137
        #         self.endFrame = 163
        #     case 'HookTrapsRing':
            self.object = 'Ring'
            self.objectPos = [1.3,-.1,3.4]
            self.objectQtn = [0,1,0,1]
            self.objectEul = list(p.getEulerFromQuaternion(self.objectQtn))
            self.obstacle = 'Hook'
            self.obstaclePos = [0, 0, 2]
            self.obstacleEul = [1.57, -0.3, 0]
            self.obstacleQtn = list(p.getQuaternionFromEuler(self.obstacleEul))
            self.obstacleScale = [.1,.1,.1]
            self.basePosBounds=[[-.5,2], [-.5,.5], [1.3,2.7]] # searching bounds
            self.goalSpaceBounds = [[1.4,2], [-.5,.5], [1.3,2.1]] + [[math.radians(-180), math.radians(180)]] + [[-.2, .2]]*2
            self.goalCoMPose = [1.6,0,1.5] + [1.57, 0, 0]
            self.startFrame = 390 # 280 - 520
            self.endFrame = 390
            self.downsampleRate = 1

    def loadObject(self):
        if self.args.object not in ['Chain', 'MaskBand']:
            self.objectId = p.loadURDF(self.paths[self.args.object], self.objectPos, self.objectQtn)

    def loadObstacle(self):
        obst = self.args.obstacle
        if obst in ['Bowl', 'Hook', 'SplashBowl', 'LeftHand', 'Shovel', 'LeftHandAndBowl', 'Ear']:
            mesh_collision_shape = p.createCollisionShape(
                shapeType=p.GEOM_MESH,
                fileName=self.paths[self.args.obstacle],
                meshScale=self.obstacleScale,
                flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
            )
            mesh_visual_shape = -1  # Use the same shape for visualization
            self.obstacleId = p.createMultiBody(
                baseCollisionShapeIndex=mesh_collision_shape,
                baseVisualShapeIndex=mesh_visual_shape,
                basePosition=self.obstaclePos,
                baseOrientation=self.obstacleQtn,
            )
            self.obstacle = ObjectFromUrdf(self.obstacleId)
        elif obst == '3fGripper':
            self.obstacleId = p.loadURDF(fileName=self.paths[self.args.obstacle], 
                                          basePosition=self.obstaclePos, 
                                          baseOrientation=self.obstacleQtn, 
                                          globalScaling=self.obstacleScale[0]
                                          )
            self.obstacle = obstascle3fGripper(self.obstacleId)

    def getJointStates(self, id):
        numJoints = p.getNumJoints(id)
        if numJoints == 0:  # rigid object with no joints
            return [], [], []
        joint_states = p.getJointStates(id, range(numJoints))
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        
        return joint_positions, joint_velocities, joint_torques

    def runHandbagGripper(self):
        '''For the task of a gripper grabbing a handbag.'''
        self.obstaclePose = self.obstaclePos + self.obstacleEul

        # Set initial joint states - hand
        jointPositions,_,_ = self.getJointStates(self.obstacleId) # list(12)
        self.numJoints = len(jointPositions)
        # obstacleJointPos = [jointPositions[i]-0/1000 if (i==1 or i==5 or i==9) else jointPositions[i] for i in range(len(jointPositions))]
        obstacleState = self.obstaclePose + jointPositions
        self.obstacle.set_state(obstacleState)

        # start simulation
        i = 0
        new_state_chain = self.objectStart
        while (1):
            print(i)
            p.stepSimulation()
            p.setGravity(0, 0, self.gravity)
            # time.sleep(20/240.)

            i_thres = [150,450]
            if i < i_thres[0]:
                increment_obs = [0,0,0,] + [0,0,0.] + [0,0.005,0,0]*3 # + [0]*4
                increment_chain = [0,0,2e-3,] + [0,0,0] + [0]*len(self.objectStart[6:])
            elif i < i_thres[1]:
                increment_obs = [0,0,0,] + [0,0.0102,0] + self.numJoints*[0]
                increment_chain = [0,-3.3e-3,-8e-3,] + [0,0,0] + [0]*len(self.objectStart[6:])
    
            # Get obstacle joint positions - hand
            curr_state_obs = self.obstacle.get_cur_state() if i>0 else obstacleState
            new_state_left = [curr_state_obs[k]+increment_obs[k] for k in range(len(curr_state_obs))]
            self.obstacle.set_state(new_state_left)

            # Get new chain state
            new_state_chain = [new_state_chain[k]+increment_chain[k] for k in range(len(self.objectStart))]

            # Validate collision status
            if i < i_thres[0]:
                print('STATE IS INVALID',  chain_collision_raycast(new_state_chain, self.linkLen, visRays=1))
            else:
                print('STATE IS INVALID',  chain_collision_raycast(new_state_chain, self.linkLen, visRays=1))
                
            if i % self.downsampleRate == 0:
                # Record obstacle state - hand
                self.obsBasePosSce.append(new_state_left[:3])
                self.obsBaseEulSce.append(new_state_left[3:6])
                self.obsBaseQtnSce.append(list(p.getQuaternionFromEuler(new_state_left[3:6])))
                self.obsJointPosSce.append(new_state_left[6:])

                # Record object state
                self.objBasePosSce.append(new_state_chain[:3])
                self.objBaseEulSce.append(new_state_chain[3:6])
                self.objBaseQtnSce.append(list(p.getQuaternionFromEuler(new_state_chain[3:6])))
                self.objJointPosSce.append(new_state_chain[6:])
                self.idxSce.append(i)
            
            if i == self.endFrame:
                p.disconnect()
                break
            i += 1

    def runHookFish(self):
        '''For the tasks of hooking fish'''
        i = 0

        # Add a table
        box_pos = [0,-2.78,1]
        box_state = box_pos + [0,0,0]
        colBox_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=self.half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBox_id, basePosition=[0,-2.78,1])
        self.box = ObjectFromUrdf(box_id)
        self.boxBasePosSce = []

        while (1):
            # print(i)
            p.stepSimulation()
            p.setGravity(0, 0, self.gravity)
            time.sleep(.1/240.)
            
            i_thres = 4500
            if i < i_thres:
                increment_hook = [0,0.0001,-0.0001,] + [0,0,0]
                increment_box = [0,0,0,] + [0,0,0]
            else:
                increment_hook = [0,0.0001,0.0001,] + [0.0001,0,0]
                increment_box = [0,-0.0002,-0.00005,] + [0,0,0]
            
            # Hook state
            curr_state_hook = self.obstacle.get_cur_state() if i>0 else self.obstaclePos+self.obstacleEul
            new_state_hook = [curr_state_hook[k]+increment_hook[k] for k in range(len(curr_state_hook))]
            self.obstacle.set_state(new_state_hook)
            
            # Box state
            curr_state_box = self.box.get_cur_state() if i>0 else box_state
            new_state_box = [curr_state_box[k]+increment_box[k] for k in range(len(curr_state_box))]
            self.box.set_state(new_state_box)

            if i>self.startFrame and i%self.downsampleRate == 0:
                gemPos, gemQtn = p.getBasePositionAndOrientation(self.objectId) # tuple(3), tuple(4)

                # Record fish state
                self.objBasePosSce.append(list(gemPos))
                self.objBaseQtnSce.append(list(gemQtn))
                self.objBaseEulSce.append(list(p.getEulerFromQuaternion(list(gemQtn))))
                self.objJointPosSce.append([])

                # Record hook state
                self.obsBasePosSce.append(new_state_hook[:3])
                self.obsBaseEulSce.append(new_state_hook[3:])
                self.obsBaseQtnSce.append(list(p.getQuaternionFromEuler(new_state_hook[3:])))
                self.obsJointPosSce.append([])
                self.idxSce.append(i)

                # Record box position
                self.boxBasePosSce.append(new_state_box[:3])

            if i == self.endFrame:
                p.disconnect()
                break
            i += 1

    def runShovelFish(self):
        '''For the task of shoveling fish with a shovel, a supporting hand and tabletop.'''
        i = 0

        # Add a table
        # box_state = self.boxBasePos + [0,0,0]
        colBox_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=self.half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBox_id, basePosition=self.boxBasePos)
        self.box = ObjectFromUrdf(box_id)
        self.boxBasePosSce = []

        # Add a hand
        mesh_collision_shape = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName=self.paths['LeftHand'],
            meshScale=self.handScale,
            flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
        )
        self.obstacleId = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=mesh_collision_shape,
            baseVisualShapeIndex=-1,
            basePosition=self.handPos,
            baseOrientation=self.handQtn,
        )

        # Dynamics rolling on
        while (1):
            # print(i)
            p.stepSimulation()
            p.setGravity(0, 0, self.gravity)
            # time.sleep(10/240.)
            
            i_thres = 24000
            if i < i_thres:
                increment_shovel = [0,-0.0001,-0.000,] + [0,0,0]
            else:
                increment_shovel = [0,0.0001,0.00015,] + [0.00004,0,0]
            
            # Shovel state
            curr_state_shovel = self.obstacle.get_cur_state() if i>0 else self.obstaclePos+self.obstacleEul
            new_state_shovel = [curr_state_shovel[k]+increment_shovel[k] for k in range(len(curr_state_shovel))]
            self.obstacle.set_state(new_state_shovel)
            
            if i % self.downsampleRate == 0 and i >= self.startFrame:
                jointPositions,_,_ = self.getJointStates(self.objectId) # list(11)
                gemPos, gemQtn = p.getBasePositionAndOrientation(self.objectId) # tuple(3), tuple(4)

                # Record fish state
                self.objBasePosSce.append(list(gemPos))
                self.objBaseQtnSce.append(list(gemQtn))
                self.objBaseEulSce.append(list(p.getEulerFromQuaternion(list(gemQtn))))
                self.objJointPosSce.append(jointPositions)

                # Record shovel state
                self.obsBasePosSce.append(new_state_shovel[:3])
                self.obsBaseEulSce.append(new_state_shovel[3:])
                self.obsBaseQtnSce.append(list(p.getQuaternionFromEuler(new_state_shovel[3:])))
                self.obsJointPosSce.append([])
                self.idxSce.append(i)

            if i == self.endFrame:
                p.disconnect()
                break
            i += 1

    def runStarfishBowl(self):
        '''For the task of catching a starfish with a bowl.'''
        i = 0

        # Add a right hand that initially holds the starfish and sheds
        box_state = self.boxBasePos + self.boxBaseEul
        colBox_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=self.half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBox_id, 
                                   basePosition=self.boxBasePos, baseOrientation=self.boxBaseQtn)
        self.box = ObjectFromUrdf(box_id)
        self.boxBasePosSce = []
        self.boxBaseEulSce = []
        self.boxBaseQtnSce = []

        # Change dynamics of objects
        p.changeDynamics(box_id, -1, lateralFriction=0.8, spinningFriction=0, rollingFriction=0,)
        p.changeDynamics(self.obstacleId, -1, lateralFriction=0.20, spinningFriction=0, rollingFriction=0.001,)

        # Dynamics rolling on
        while (1):
            # print(i)
            p.stepSimulation()
            p.setGravity(0, 0, self.gravity)
            time.sleep(.2/240.)
            
            i_thres = [1000,2200]
            if i < i_thres[0]: # right hand sheds
                increment_bowl = [0,0,0,] + [0,0,0]
                increment_box = [0,-0.0,-0.0,] + [0.0008,0,0]
            elif i < i_thres[1]: # bowl left-ward rotate
                increment_bowl = [0,-0.0001,-0.,] + [0.0012,0,0]
                increment_box = [0,0.001,0] + [0.0005,0,0]
            else: # bowl right-ward rotate
                increment_bowl = [0,0,-0.,] + [-0.0012,0,0]
                increment_box = [0,0.0,0] + [0,0,0]
            
            # Bowl state
            curr_state_bowl = self.obstacle.get_cur_state() if i>0 else self.obstaclePos+self.obstacleEul
            new_state_bowl = [curr_state_bowl[k]+ increment_bowl[k] for k in range(len(curr_state_bowl))]
            self.obstacle.set_state(new_state_bowl)

            # Box / right hand state
            curr_state_box = self.box.get_cur_state() if i>0 else box_state
            new_state_box = [curr_state_box[k]+increment_box[k] for k in range(len(curr_state_box))]
            self.box.set_state(new_state_box)

            if i % self.downsampleRate == 0 and i >= self.startFrame:
                jointPositions,_,_ = self.getJointStates(self.objectId) # list(11)
                gemPos, gemQtn = p.getBasePositionAndOrientation(self.objectId) # tuple(3), tuple(4)

                # Record starfish state
                self.objBasePosSce.append(list(gemPos))
                self.objBaseQtnSce.append(list(gemQtn))
                self.objBaseEulSce.append(list(p.getEulerFromQuaternion(list(gemQtn))))
                self.objJointPosSce.append(jointPositions)

                # Record bowl state
                self.obsBasePosSce.append(new_state_bowl[:3])
                self.obsBaseEulSce.append(new_state_bowl[3:])
                self.obsBaseQtnSce.append(list(p.getQuaternionFromEuler(new_state_bowl[3:])))
                self.obsJointPosSce.append([])
                self.idxSce.append(i)

                # Record box position
                self.boxBasePosSce.append(new_state_box[:3])
                self.boxBaseEulSce.append(new_state_box[3:])
                self.boxBaseQtnSce.append(list(p.getQuaternionFromEuler(new_state_box[3:])))

            if i == self.endFrame:
                p.disconnect()
                break
            i += 1

    def runBimanualRubic(self):
        '''For the task of 2 grippers clenching rubic cube.'''
        self.obstaclePose = self.obstaclePos + self.obstacleEul
        self.obstaclePose1 = self.obstaclePos1 + self.obstacleEul1
        self.obsBasePosSce1 = []
        self.obsBaseQtnSce1 = []
        self.obsBaseEulSce1 = []
        self.obsJointPosSce1 = []

        # Add right hand
        self.obstacleId1 = p.loadURDF(fileName=self.paths[self.args.obstacle], 
                                    basePosition=self.obstaclePos1, 
                                    baseOrientation=self.obstacleQtn1, 
                                    globalScaling=self.obstacleScale[0]
                                    )
        self.obstacle1 = obstascle3fGripper(self.obstacleId1)

        # Set initial joint states - left hand
        jointPositions,_,_ = self.getJointStates(self.obstacleId) # list(12)
        self.numJoints = len(jointPositions)
        # obstacleJointPos = [jointPositions[i]-0/1000 if (i==1 or i==5 or i==9) else jointPositions[i] for i in range(len(jointPositions))]
        obstacleState = self.obstaclePose + jointPositions
        self.obstacle.set_state(obstacleState)
        
        # Set initial joint states - right hand
        jointPositions1,_,_ = self.getJointStates(self.obstacleId1)
        # obstacleJointPos1 = [jointPositions1[i]-0/1000 if (i==1 or i==5 or i==9) else jointPositions1[i] for i in range(len(jointPositions1))]
        obstacleState1 = self.obstaclePose1 + jointPositions1
        self.obstacle1.set_state(obstacleState1)
        
        # start simulation
        i = 0
        while (1):
            # print(i)
            p.stepSimulation()
            p.setGravity(0, 0, self.gravity)
            time.sleep(.5/240.)

            i_thres = [1000,2000]
            if i < i_thres[0]:
                increment_obs = [0,0,0,] + [0,0,0] + self.numJoints*[0]
                increment_right = [0,0,-0.001,] + [0.0,0,0] + self.numJoints*[0]
            elif i < i_thres[1]:
                alpha = (i-i_thres[0]) / (i_thres[1]-i_thres[0]) * 1.57
                increment_obs = [0,-1.57e-3*np.cos(alpha),1.57e-3*np.sin(alpha),] + [-1.57e-3,0,0] + self.numJoints*[0]
                increment_right = [0,1.57e-3*np.cos(alpha),-1.57e-3*np.sin(alpha),] + [0,1.57e-3,0] + [0,]*8 + [0,0.0001,0,0]
            else:
                increment_obs = [0,-0.000,0.000,] + [-0.000,0,0] + self.numJoints*[0]
                increment_right = [0,0.000,-0.000,] + [0,0.000,0] + [0,-0.00055,0,0] + [0,-0.00055,0,0] + [0,0.000,0,0]
    
            # Get obstacle joint positions - left hand
            curr_state_left = self.obstacle.get_cur_state() if i>0 else obstacleState
            new_state_left = [curr_state_left[k]+increment_obs[k] for k in range(len(curr_state_left))]
            self.obstacle.set_state(new_state_left)

            # Get obstacle joint positions - right hand
            curr_state_right = self.obstacle1.get_cur_state() if i>0 else obstacleState1
            new_state_right = [curr_state_right[k]+increment_right[k] for k in range(len(curr_state_right))]
            self.obstacle1.set_state(new_state_right)

            # print(len(p.getClosestPoints(bodyA=self.objectId, bodyB=self.obstacleId, distance=-0.025)))

            if i % self.downsampleRate == 0:
                # Record obstacle state - left hand
                self.obsJointPosSce.append(new_state_left[6:])
                self.obsBasePosSce.append(new_state_left[:3])
                self.obsBaseEulSce.append(new_state_left[3:6])
                self.obsBaseQtnSce.append(list(p.getQuaternionFromEuler(new_state_left[3:6])))

                # Record obstacle state - right hand
                self.obsJointPosSce1.append(new_state_right[6:])
                self.obsBasePosSce1.append(new_state_right[:3])
                self.obsBaseEulSce1.append(new_state_right[3:6])
                self.obsBaseQtnSce1.append(list(p.getQuaternionFromEuler(new_state_right[3:6])))

                # Record rubic cube pose
                gemPos, gemQtn = p.getBasePositionAndOrientation(self.objectId) # tuple(3), tuple(4)
                self.objBasePosSce.append(list(gemPos))
                self.objBaseQtnSce.append(list(gemQtn))
                self.objJointPosSce.append([])
                self.idxSce.append(i)
            
            if i == self.endFrame:
                p.disconnect()
                break
            i += 1

        # quaternion to euler
        self.objBaseEulSce = [list(p.getEulerFromQuaternion(q)) for q in self.objBaseQtnSce]

    def readMaskEar(self, folderName):
        self.frameID = []
        self.objectStateSce = []
        headers = ['frameID', 
                'vertex005fixed-x', 'vertex005fixed-y', 'vertex005fixed-z', 
                'vertex140-x', 'vertex140-y', 'vertex140-z', 
                'vertex145-x', 'vertex145-y', 'vertex145-z', 
                'vertex150-x', 'vertex150-y', 'vertex150-z', 
                'vertex160-x', 'vertex160-y', 'vertex160-z', 
                'vertex170-x', 'vertex170-y', 'vertex170-z', 
                'vertex180-x', 'vertex180-y', 'vertex180-z', 
                'vertex190fixed-x', 'vertex190fixed-y', 'vertex190fixed-z',]
        
        with open('{}/data.csv'.format(folderName), 'r') as csvfile:
            csvreader = csv.DictReader(csvfile)
            for i, row in enumerate(csvreader):
                # fixed vertices positions of the mask band
                if i == 0:
                    self.bandFixedV0, self.bandFixedV1 = [], []
                    for h in headers[1:4]:
                        self.bandFixedV0.append(float(row[h]))
                    for h in headers[4:22]:
                        self.objectGoal.append(float(row[h]))
                    for h in headers[-3:]:
                        self.bandFixedV1.append(float(row[h]))

                # record a section of band states over frames
                id = int(row['frameID'])
                if id >= self.startFrame and id <= self.endFrame:
                    self.frameID.append(id)
                    bandVerticesState = []
                    for h in headers[4:22]:
                        bandVerticesState.append(float(row[h]))
                    mask_band_collision_raycast(bandVerticesState, self.bandFixedV0, self.bandFixedV1, visRays=1, lifeTime=.1)
                    sleep(.1)
                    self.objectStateSce.append(bandVerticesState)
        
        # Set goal region
        ofs = 0.05
        self.goalSpaceBounds = [[x-ofs, x+ofs] for x in self.objectGoal] # [[0,0.1], [0,0.1], [0.5,.51]] * self.numCtrlPoint
        
        p.disconnect()

    def runDynamicFalling(self):
        '''For the tasks of articulated fish or ring falling'''
        i = 0        
        while (1):
            p.stepSimulation()
            p.setGravity(0, 0, self.gravity)
            time.sleep(1/240.)

            if i % self.downsampleRate == 0 and i >= self.startFrame:
                jointPositions,_,_ = self.getJointStates(self.objectId) # list(11)
                gemPos, gemQtn = p.getBasePositionAndOrientation(self.objectId) # tuple(3), tuple(4)

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
    # t_list = [2,6,10,15,22,30,38,50,70,100,200,350,600]
    # for t in t_list:
    args, parser = argument_parser()
    # args.runtime = t
    numRunTime = 10
    for n in range(numRunTime):
        rigidObjectList = get_non_articulated_objects()
        isArticulatedObj = False if args.object in rigidObjectList else True
    
        # run a dynamic falling scenario and analyze frame-wise escape energy
        sce = runScenario(args)
        if args.scenario in ['HookFishHole']:
            sce.runHookFish()
            record_dynamics_scene(sce, args)
        elif args.scenario in ['ShovelFish']:
            sce.runShovelFish()
            record_dynamics_scene(sce, args)
        elif args.scenario in ['StarfishBowl']:
            sce.runStarfishBowl()
            record_dynamics_scene(sce, args)
        elif args.scenario in ['BimanualRubic']:
            sce.runBimanualRubic()
            record_dynamics_scene(sce, args)
        elif args.scenario in ['HandbagGripper']:
            sce.runHandbagGripper()
            record_dynamics_scene(sce, args)
        elif args.scenario in ['MaskEar']:
            folderName = 'results/4blender/mask-head'
            sce.readMaskEar(folderName)
        elif args.scenario in ['HookTrapsRing']:
            sce.runDynamicFalling()
            # record_dynamics_scene(sce, args)

        # Create caging environment and items in pybullet
        if args.object in rigidObjectList:
            env = RigidObjectCaging(args)
        elif args.object in ['Chain']:
            env = ChainCaging(args, sce.numCtrlPoint, sce.linkLen, sce.objectStart, sce.objectGoal)
        elif args.object in ['MaskBand']:
            env = MaskBandCaging(args, sce.numCtrlPoint, sce.objectStart, sce.objectGoal)
        else:
            objScale = 1
            env = ArticulatedObjectCaging(args, objScale)

        # Set searching bounds and add obstacles
        env.robot.set_search_bounds(basePosBounds=sce.basePosBounds)
        if args.scenario in ['MaskEar']:
            env.add_obstacles(sce.obstaclePos, sce.obstacleQtn, sce.obstacleScale)
        else:
            env.add_obstacles(sce.obsBasePosSce[0], sce.obsBaseQtnSce[0], sce.obstacleScale)

        # Add extra obstacles
        # if args.scenario in ['HookFishHole']:
        #     box_id = env.add_box(sce.boxBasePosSce[0], sce.half_box_size)
        if args.scenario in ['ShovelFish']:
            box_id = env.add_box(sce.boxBasePos, sce.half_box_size)
            env.add_obstacles(sce.handPos, sce.handQtn, sce.handScale, obstacleName='LeftHand')
        elif args.scenario in ['StarfishBowl']:
            box_id = env.add_box(sce.boxBasePos, sce.half_box_size, sce.boxBaseQtn)
        elif args.scenario in ['BimanualRubic']:
            env.add_obstacles(sce.obsBasePosSce1[0], sce.obsBaseQtnSce1[0], sce.obstacleScale, obstacleName='3fGripper')

        # print('FISH {}, BOX {}, HAND {}, SHOVEL {} IDs'.format(env.robot.id, box_id, env.obstacle_id_new, env.obstacle_id))
        escapeEnergyCostSce = []
        startEnergySce = [] # start state energy
        startGEnergySce = [] # start G potential energy
        startEEnergySce = [] # start E potential energy
        
        # Run the caging analysis algorithm over downsampled frames we extracted above
        numMainIter = len(sce.objectStateSce) if args.scenario in ['MaskEar'] else len(sce.objJointPosSce)
        for i in range(numMainIter):
            # if i == 1:
            #     continue

            # Set obstacle's state
            if args.scenario in ['GripperClenchesStarfish',]:
                env.obstacle._set_joint_positions(env.obstacle.joint_idx, sce.obsJointPosSce[i])
                p.resetBasePositionAndOrientation(env.obstacle_id, sce.obsBasePosSce[i], sce.obsBaseQtnSce[i])
            if args.scenario in ['HookFishHole']:
                p.resetBasePositionAndOrientation(env.obstacle_id, sce.obsBasePosSce[i], sce.obsBaseQtnSce[i])
                # p.resetBasePositionAndOrientation(box_id, sce.boxBasePosSce[i], list(p.getQuaternionFromEuler([0,0,0])))
            if args.scenario in ['ShovelFish']:
                p.resetBasePositionAndOrientation(env.obstacle_id, sce.obsBasePosSce[i], sce.obsBaseQtnSce[i])
            if args.scenario in ['StarfishBowl']:
                p.resetBasePositionAndOrientation(env.obstacle_id, sce.obsBasePosSce[i], sce.obsBaseQtnSce[i])
                p.resetBasePositionAndOrientation(box_id, sce.boxBasePosSce[i], sce.boxBaseQtnSce[i])
            if args.scenario in ['BimanualRubic']:
                env.obstacle._set_joint_positions(env.obstacle.joint_idx, sce.obsJointPosSce[i]) # left hand
                p.resetBasePositionAndOrientation(env.obstacle_id, sce.obsBasePosSce[i], sce.obsBaseQtnSce[i])
                env.obstacle1._set_joint_positions(env.obstacle1.joint_idx, sce.obsJointPosSce1[i]) # right hand
                p.resetBasePositionAndOrientation(env.obstacle_id1, sce.obsBasePosSce1[i], sce.obsBaseQtnSce1[i])
            if args.scenario in ['HandbagGripper']:
                env.obstacle._set_joint_positions(env.obstacle.joint_idx, sce.obsJointPosSce[i])
                p.resetBasePositionAndOrientation(env.obstacle_id, sce.obsBasePosSce[i], sce.obsBaseQtnSce[i])
            if args.scenario in ['MaskEar']:
                p.resetBasePositionAndOrientation(env.obstacle_id, sce.obstaclePos, sce.obstacleQtn)

            # Set object's start and goal states
            if args.scenario in ['MaskEar']:
                objStartState = sce.objectStateSce[i]
            else:
                objJointPos = [round(n, 2) for n in sce.objJointPosSce[i]]
                # TODO
                basePos = [sce.objBasePosSce[i][k] if k<2 else sce.objBasePosSce[i][k]+0.018 for k in range(len(sce.objBasePosSce[i]))] # 0.018
                objStartState = basePos + sce.objBaseEulSce[i] + objJointPos
            if args.object in ['Chain']:
                objGoalState = sce.objectGoal
            elif args.object in ['MaskBand']:
                objGoalState = sce.objectGoal
            else:
                objGoalState = sce.goalCoMPose + [0]*env.robot.articulate_num
            env.reset_start_and_goal(objStartState, objGoalState)
            # if not isValidStartAndGoal: # start or goal state invalid
            #     continue

            # Create OMPL interface
            env.create_ompl_interface()
            env.pb_ompl_interface.set_goal_space_bounds(sce.goalSpaceBounds)
            if args.scenario in ['MaskEar']:
                env.pb_ompl_interface.record_fixed_vertex_pos(sce.bandFixedV0, sce.bandFixedV1)

            # Choose a searching method
            if args.search == 'BisectionSearch':
                useBisecSearch = 1 # True: bisection search; False: Conservative search
                maxT = 600
                numIter = 3
                env.energy_bisection_search(numIter=numIter, useBisectionSearch=useBisecSearch, maxTimeTaken=maxT)
                # env.visualize_bisection_search() # visualize
                # print('final z threshold: {}, escape energy: {}'.format(z_thres, escape_energy))

                # Create new folder
                createFolder = 1 if i == 0 else 0
                if createFolder:
                    now = datetime.now()
                    dt_string = now.strftime("%d-%m-%Y-%H-%M-%S") # dd/mm/YY H:M:S
                    folderName = './results/ICRA2024/{}'.format(dt_string)
                    os.mkdir(folderName)

                # Record data to the folder
                record_data_benchmark_bound_shrink(env.escape_energy_list_runs, env.time_taken_list_runs, i, folderName)

            elif args.search == 'EnergyBiasedSearch':
                numInnerIter = 1
                save_escape_path = 0
                get_cost_from_path = 0 # for rigid objects and incremental cost
                isSolved = env.energy_biased_search(numInnerIter, save_escape_path, get_cost_from_path=get_cost_from_path)
                # env.visualize_energy_biased_search()

                # Record start and escape energy
                if args.object in ['Fish']:
                    startGEnergy = env.pb_ompl_interface.potentialObjective.getGravityEnergy(objStartState)
                    startEEnergy = env.pb_ompl_interface.potentialObjective.getElasticEnergy(objStartState)
                    startEnergy = startGEnergy + startEEnergy
                elif args.object in ['Starfish']:
                    startGEnergy = env.pb_ompl_interface.potentialObjective.getGravityEnergy(objStartState)
                    startEEnergy, startEnergy = 0, startGEnergy
                elif args.object in ['Band', 'MaskBand']:
                    startEEnergy = env.pb_ompl_interface.potentialObjective.getElasticEnergy(objStartState)
                    startGEnergy, startEnergy = 0, startEEnergy
                else:
                    # startEnergy = env.state_energy_escape_path_iter[0][0] if isSolved else np.inf
                    startGEnergy, startEEnergy = None, None
                    startEnergy = objStartState[2]
                startEnergySce.append(startEnergy)
                startGEnergySce.append(startGEnergy)
                startEEnergySce.append(startEEnergy)
                
                if args.object in ['Ring'] and get_cost_from_path:
                    escapeEnergyCost = env.cost_from_path
                else:
                    escapeEnergyCost = min(env.sol_final_costs) if isSolved else np.inf
                escapeEnergyCostSce.append(escapeEnergyCost) # list(numMainIter*list(numInnerIter))
                
                # Create txt, csv for data recording
                if i == 0:
                    folderName = record_data_init(sce, args, env)
                
                # Record data in this loop 
                energyData = [startEnergy, startGEnergy, startEEnergy, escapeEnergyCost]
                record_data_loop(sce, args, energyData, folderName, i)

            # sleep(10)

        # Shut down pybullet (GUI)
        p.disconnect()

        # # Plot
        # energyData = (startEnergySce, startGEnergySce, startEEnergySce, escapeEnergyCostSce)
        # plot_escape_energy(energyData, args, folderName, isArticulatedObj)
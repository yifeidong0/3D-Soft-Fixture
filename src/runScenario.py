import pybullet as p
import time
import os.path as osp
import sys
import argparse
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))
from pbOmplInterface import PbOMPL
from rigidObjCaging import RigidObjectCaging
from articulatedObjCaging import ArticulatedObjectCaging
import matplotlib.pyplot as plt
from main import argument_parser
import pybullet_data
from utils import path_collector

SCENARIOS = ['FishFallsInBowl', 'HookTrapsFish', 'GripperGraspsDonut']

class runScenario():
    def __init__(self, args, scenario='FishFallsInBowl'):
        p.connect(p.GUI)
        # p.setGravity(0, 0, -9.8)
        p.setTimeStep(1./240.)
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setRealTimeSimulation(0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # planeId = p.loadURDF("plane.urdf", [0,0,0])
        self.scenario = scenario
        self.paths = path_collector()
        self.args = args
        self.gravity = -9.81
        self.downsampleRate = 100
        self.endFrame = 800

        # load object and obstacle
        self.initializePoses()
        self.loadObject()
        self.loadObstacle()
    
    def initializePoses(self):
        match self.scenario:
            case 'FishFallsInBowl':
                self.object = 'Fish'
                self.obstacle = 'Bowl'
                self.objectPos = (0,0,4)
                self.objectOrn = (0,1,0,1)
                self.obstaclePos = [-0.5, 0.5, 0]
                self.obstacleOrn = p.getQuaternionFromEuler([0, 0, 0])
                self.obstacleScale = [.1, .1, .1]
                self.basePosBounds=[[-2,2], [-2,2], [0,3]] # searching bounds
            case 'HookTrapsFish':
                self.object = 'FishWithRing'
                self.obstacle = 'Hook'
                self.objectPos = (1,-2.1,3.4)
                self.objectOrn = (0,1,0,1)
                self.obstaclePos = [0, 0, 2]
                self.obstacleOrn = p.getQuaternionFromEuler([1.57, 0, 0])
                self.obstacleScale = [.1,.1,.1]
                self.basePosBounds=[[-2,2], [-2.5,2.5], [-0.5,3.5]] # searching bounds
            
            # case 'GripperGraspDonut':

    def loadObject(self):
        # bowl = p.loadURDF('models/bowl/bowl.urdf', (0,0,0), (0,0,1,1), globalScaling=5)
        # self.object = p.loadURDF('models/articulate_fish.xacro', (0,0,4), (0,0,1,1))
        # p.changeDynamics(bowl, -1, mass=0)
        self.objectId = p.loadURDF(self.paths[self.object], self.objectPos, self.objectOrn)

    def loadObstacle(self):
        # Upload the mesh data to PyBullet and create a static object
        mesh_collision_shape = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName=self.paths[self.obstacle],
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
            baseOrientation=self.obstacleOrn,
        )

    def getJointStates(self):
        joint_states = p.getJointStates(self.objectId, range(p.getNumJoints(self.objectId)))
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        
        return joint_positions, joint_velocities, joint_torques

    def runDynamicFalling(self):
        i = 0
        jointPosSce = []
        basePosSce = []
        baseOrnSce = []
        time.sleep(2)

        while (1):
            p.stepSimulation()
            #p.setJointMotorControl2(botId, 1, p.TORQUE_CONTROL, force=1098.0)
            # p.applyExternalTorque(mesh_id, -1, [1,0,0], p.WORLD_FRAME)
            # print(gemPos, gemOrn)
            p.setGravity(0, 0, self.gravity)
            time.sleep(1/240.)
            
            i += 1
            if i % self.downsampleRate == 0 and i > 140:
                jointPositions,_,_ = self.getJointStates() # list(11)
                gemPos, gemOrn = p.getBasePositionAndOrientation(self.objectId) # tuple(3), tuple(4)
                jointPosSce.append(jointPositions)
                basePosSce.append(list(gemPos))
                baseOrnSce.append(list(gemOrn))
            
            if i == self.endFrame:
                p.disconnect()
                return jointPosSce, basePosSce, baseOrnSce, self.obstaclePos, self.obstacleOrn


if __name__ == '__main__':
    # setup
    scenarioId = 1

    args = argument_parser()
    rigidObjs = ['Donut', 'Hook', 'Bowl']

    # run a dynamic falling scenario and analyze frame-wise escape energy
    sce = runScenario(args, SCENARIOS[scenarioId])
    objJointPosSce, objBasePosSce, objBaseQtnSce, obsPos, obsOrn = sce.runDynamicFalling()
    objBaseOrnSce = [list(p.getEulerFromQuaternion(q)) for q in objBaseQtnSce]
    numMainIter = len(objJointPosSce)

    # create caging environment and items in pybullet
    if args.object in rigidObjs:
        eps_thres = 1e-2 # threshold of loop terminating
        env = RigidObjectCaging(args, eps_thres)
    else:
        env = ArticulatedObjectCaging(args)

    # set searching bounds and add obstacles
    env.robot.set_search_bounds(sce.basePosBounds)
    env.add_obstacles(obsPos, obsOrn, sce.obstacleScale)
    bestCostsSce = []
    startCostSce = [] # start state energy
    # objBaseZs = []

    for i in range(numMainIter):
        start = objBasePosSce[i] + objBaseOrnSce[i] + objJointPosSce[i]
        goal = [0,0,0.01] + [0]*3 + [0]*env.robot.articulate_num
        isValidStartAndGoal = env.reset_start_and_goal(start, goal)
        if not isValidStartAndGoal:
            continue
        # objBaseZs.append(start[2])
        # print('Current object z_world: {}'.format(start[2]))

        env.pb_ompl_interface = PbOMPL(env.robot, args, env.obstacles)

        # Choose from different searching methods
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
            bestCostsSce.append(min(env.sol_final_costs)) # list(numMainIter*list(numInnerIter))
            startCostSce.append(env.energy_minimize_paths_energies[0][0])
            print('Energy costs of current obstacle and object config: {}'.format(env.sol_final_costs))

    # shut down pybullet (GUI)
    p.disconnect()

    # plot escape energy in the dynamic fall
    _, ax1 = plt.subplots()
    ax1.plot(bestCostsSce, 'r--', label='Escape energy')
    ax1.plot(startCostSce, 'g-*', label='Current energy')
    
    ax1.set_xlabel('# iterations')
    ax1.set_ylabel('G-potential energy')
    ax1.grid(True)
    ax1.legend()
    plt.title('Escape energy in a dynamic scenario - fish falls into a bowl')
    # plt.show()
    plt.savefig('./images/fishFalls3.png')

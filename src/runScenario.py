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

class runScenario():
    def __init__(self, args):
        p.connect(p.GUI)
        # p.setGravity(0, 0, -9.8)
        p.setTimeStep(1./240.)
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setRealTimeSimulation(0)

        self.paths = {
            'Fish': 'models/articulate_fish.xacro', 
            'Hook': 'models/triple_hook/triple_hook.urdf', 
            'Donut': 'models/donut/donut.urdf',
            '3fGripper': 'models/robotiq_3f_gripper_visualization/cfg/robotiq-3f-gripper_articulated.urdf',
            'PandaArm': 'models/franka_description/robots/panda_arm.urdf',
            'PlanarRobot': 'models/planar_robot_4_link.xacro',
            'Humanoid': 'models/humanoid.urdf',
            'Bowl': 'models/bowl/small_bowl.stl', 
            }
        self.args = args
        self.gravity = -9.81
        self.downsampleRate = 80
        self.endFrame = 600

        # load object and obstacle
        self.loadObject()
        self.loadObstacle()
        
    def loadObject(self):
        # bowl = p.loadURDF('models/bowl/bowl.urdf', (0,0,0), (0,0,1,1), globalScaling=5)
        # self.object = p.loadURDF('models/articulate_fish.xacro', (0,0,4), (0,0,1,1))
        self.objectPos = (0,0,4)
        self.objectOrn = (0,1,0,1)
        self.objectId = p.loadURDF(self.paths[self.args.object], self.objectPos, self.objectOrn)
        # p.changeDynamics(bowl, -1, mass=0)

    def loadObstacle(self):
        # Upload the mesh data to PyBullet and create a static object
        self.obstaclePos = [-0.5, 0.5, 0]  # The position of the mesh, [-0.5, 1.5, 0]
        self.obstacleOrn = p.getQuaternionFromEuler([0, 0, 0])  # The orientation of the mesh, [.6, 0, 0]
    
        mesh_scale = [.1, .1, .1]  # The scale of the mesh
        mesh_collision_shape = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName=self.paths[self.args.obstacle],
            meshScale=mesh_scale,
            flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
        )
        mesh_visual_shape = -1  # Use the same shape for visualization
        self.obstacleId = p.createMultiBody(
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
        while (1):
            p.stepSimulation()
            #p.setJointMotorControl2(botId, 1, p.TORQUE_CONTROL, force=1098.0)
            # p.applyExternalTorque(mesh_id, -1, [1,0,0], p.WORLD_FRAME)
            # print(gemPos, gemOrn)
            p.setGravity(0, 0, self.gravity)
            time.sleep(5/240.)
            
            i += 1
            # print(i)
            if i % self.downsampleRate == 0 and i > 140:
                jointPositions,_,_ = self.getJointStates() # list(11)
                gemPos, gemOrn = p.getBasePositionAndOrientation(self.objectId) # tuple(3), tuple(4)
                jointPosSce.append(jointPositions)
                basePosSce.append(list(gemPos))
                baseOrnSce.append(list(gemOrn))
            
            if i == self.endFrame:
                p.disconnect()
                return jointPosSce, basePosSce, baseOrnSce, self.obstaclePos, self.obstacleOrn
            # print(jointPositions)


if __name__ == '__main__':
    args = argument_parser()
    rigidObjs = ['Donut', 'Hook', 'Bowl']
    basePosBounds=[[-2,2], [-2,2], [0,3]] # searching bounds

    # run a dynamic falling scenario and analyze frame-wise escape energy
    sce = runScenario(args)
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
    env.robot.set_search_bounds(basePosBounds)
    env.add_obstacles(obsPos, obsOrn)
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
        # CP = p.getClosestPoints(bodyA=env.robot_id, bodyB=env.obstacle_id, distance=-0.025)
        # if len(CP)>0:
        #     dis = [CP[i][8] for i in range(len(CP))]
        #     print('!!!!CP', dis)

        # Choose from different searching methods
        if args.search == 'BoundShrinkSearch':
            useBisecSearch = True # True: bisection search; False: Conservative search
            env.bound_shrink_search(useBisecSearch)
            escape_energy, z_thres = env.visualize_bound_shrink_search(useBisecSearch) # visualize
            print('final z threshold: {}, escape energy: {}'.format(z_thres, escape_energy))

        elif args.search == 'EnergyMinimizeSearch':
            numInnerIter = 1
            env.energy_minimize_search(numInnerIter)
            # env.visualize_energy_minimize_search()
            bestCostsSce.append(min(env.sol_final_costs)) # list(numMainIter*list(numInnerIter))
            startCostSce.append(env.energy_minimize_paths_energies[0][0])
            print('Energy costs of current obstacle and object config: {}'.format(env.sol_final_costs))

    # shut down pybullet (GUI)
    p.disconnect()

    # plot escape energy in the dynamic fall
    _, ax1 = plt.subplots()
    ax1.plot(bestCostsSce, 'r--', label='escape energy')
    ax1.plot(startCostSce, 'g-*', label='start energy')
    
    ax1.set_xlabel('# iterations')
    ax1.set_ylabel('energy')
    ax1.grid(True)
    ax1.legend()
    plt.title('Escape energy in a dynamic scenario - fish falls into a bowl')
    # plt.show()
    plt.savefig('./images/fishFalls2.png')

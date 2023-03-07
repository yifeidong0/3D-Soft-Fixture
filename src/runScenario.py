
import pybullet as p
import time

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
        self.gravity = -10
        self.downsampleRate = 30
        self.endFrame = 500

        # load object and obstacle
        self.loadObject()
        self.loadObstacle()
        
    def loadObject(self):
        # bowl = p.loadURDF('models/bowl/bowl.urdf', (0,0,0), (0,0,1,1), globalScaling=5)
        # self.object = p.loadURDF('models/articulate_fish.xacro', (0,0,4), (0,0,1,1))
        self.objectPos = (0,0,4)
        self.objectOrn = (0,0,1,1)
        self.objectId = p.loadURDF(self.paths[self.args.object], self.objectPos, self.objectOrn)
        # p.changeDynamics(bowl, -1, mass=0)

    def loadObstacle(self):
        # Upload the mesh data to PyBullet and create a static object
        self.obstaclePos = [-0.5, 1.5, 0]  # The position of the mesh
        self.obstacleOrn = p.getQuaternionFromEuler([.6, 0, 0])  # The orientation of the mesh
    
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
            time.sleep(1/240.)
            
            i += 1
            if i % self.downsampleRate == 0:
                jointPositions,_,_ = self.getJointStates() # list(11)
                gemPos, gemOrn = p.getBasePositionAndOrientation(self.objectId) # tuple(3), tuple(4)
                jointPosSce.append(jointPositions)
                basePosSce.append(list(gemPos))
                baseOrnSce.append(list(gemOrn))
            
            if i == self.endFrame:
                p.disconnect()
                return jointPosSce, basePosSce, baseOrnSce, self.obstaclePos, self.obstacleOrn
            # print(jointPositions)

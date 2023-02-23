import pybullet as p
from time import sleep
import pybullet_data
import numpy as np

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
p.setGravity(0, 0, -10)

def add_box(box_pos, half_box_size):
    colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
    box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

    # self.obstacles.append(box_id)
    return box_id

planeOrn = [0,0,0,1]#p.getQuaternionFromEuler([0.3,0,0])
planeId = p.loadURDF("plane.urdf", [0,0,-2],planeOrn)

# boxId = p.loadURDF("cube.urdf", [0,3,2],useMaximalCoordinates = True)

basePosition = [0,0,2]
ballId = p.loadSoftBody("models/deformable_ball/ball.obj", 
                        simFileName = "models/deformable_ball/ball.vtk", 
                        basePosition = basePosition,
                        scale = 1, mass = .01, 
                        useNeoHookean = 1, 
                        NeoHookeanMu = 4, NeoHookeanLambda = 6, 
                        NeoHookeanDamping = 0.001,
                        # useMassSpring=1, 
                        # useBendingSprings=1, 
                        # springElasticStiffness=1,
                        # springDampingStiffness=1,
                        # springBendingStiffness=1,
                        useSelfCollision = 1, 
                        frictionCoeff = .5, 
                        collisionMargin = 0.001,
                        useFaceContact=1,)
print('!!!!!!!!!', (p.getMeshData(ballId)[0]))
print('!!!!!!!!!', (p.getDynamicsInfo(ballId,-1)))

# box_pos = [0, 0, 0]
# box_id = add_box(box_pos, [5, 5, .1])
# c = [078.0 / 255.0, 121.0 / 255.0, 167.0 / 255.0, 1]
# radius = .1
# part_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[radius]*3)
# part_visual = p.createVisualShape(p.GEOM_SPHERE, radius=radius*1.5)
# boxId2 = p.createMultiBody(0.01, part_shape, part_visual, basePosition=[0,0,0])
# p.changeVisualShape(boxId2, -1)
# vert1pos = [-0.9585980176925659, -0.1718350052833557, 0.103534996509552]
# boxId2 = p.loadURDF("cube.urdf", [1,0,0], useMaximalCoordinates=True, globalScaling=.1)
# p.createSoftBodyAnchor(ballId, 0, boxId2,-1, [0,-0,0])
# _ = p.createSoftBodyAnchor(
#         softBodyBodyUniqueId=ballId,
#         nodeIndex=100,
#         bodyUniqueId=boxId2,
#         linkIndex=-1,)
# constraint_id = p.createConstraint(
#         parentBodyUniqueId=ballId,
#         parentLinkIndex=0,
#         childBodyUniqueId=boxId2,
#         childLinkIndex=-1,
#         jointType=p.JOINT_POINT2POINT,
#         jointAxis=(0, 0, 0),
#         parentFramePosition=(0, 0, 0),
#         childFramePosition=(0, 0, 0))

# boxId = p.loadURDF("cube.urdf", [0,1,2],useMaximalCoordinates = True)
# clothId = p.loadSoftBody("cloth_z_up.obj", basePosition = [0,0,2], scale = 0.5, mass = 1., useNeoHookean = 0, useBendingSprings=1,useMassSpring=1, springElasticStiffness=40, springDampingStiffness=.1, springDampingAllDirections = 1, useSelfCollision = 0, frictionCoeff = .5, useFaceContact=1)

# p.changeVisualShape(clothId, -1, flags=p.VISUAL_SHAPE_DOUBLE_SIDED)
# p.createSoftBodyAnchor(clothId  ,24,-1,-1)
# p.createSoftBodyAnchor(clothId ,20,-1,-1)
# p.createSoftBodyAnchor(clothId ,15,boxId,-1, [0.5,-0.5,0])
# p.createSoftBodyAnchor(clothId ,19,boxId,-1, [-0.5,-0.5,0])
# p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)

p.setTimeStep(0.001)
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)

#logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "perf.json")
# h = np.linspace(basePosition[2], box_pos[2], 1000)
# i = 0
while p.isConnected():
  # p.performCollisionDetection()
  p.stepSimulation()
  # p.applyExternalForce(boxId, 0, [0,0,10], [0,0,0], p.WORLD_FRAME)

  # p.setJointMotorControl2(ballId, 2, 
  #                         p.POSITION_CONTROL, h[i],force=.005 * 240.)
  # print(len(p.getContactPoints(bodyA=ballId, bodyB=box_id)))
  # print(p.getClosestPoints(bodyA=ballId, bodyB=planeId, distance=0.))

  #there can be some artifacts in the visualizer window, 
  #due to reading of deformable vertices in the renderer,
  #while the simulators updates the same vertices
  #it can be avoided using
  #p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
  #but then things go slower
  # p.setGravity(0,0,-10)
  sleep(1./240.)
  # i += 1
  # if i == 999:
  #    break
#p.resetSimulation()
#p.stopStateLogging(logId)

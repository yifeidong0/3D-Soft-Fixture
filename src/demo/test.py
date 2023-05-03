import os.path as osp
import sys
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))
from utils import *
#########################################################################
#########################################################################
#########################################################################

# FORWARD KINEMATICS

# import kinpy as kp
# import math

# chain = kp.build_chain_from_urdf(open("models/articulate_fish.xacro").read())
# print(chain)

# th = {'joint0': math.pi / 4.0, 'joint1': math.pi / 4.0}
# th = {'joint0': 0 / 4.0, 'joint1': 0 / 4.0}
# ret = chain.forward_kinematics(th)
# print(ret)


#########################################################################
#########################################################################
#########################################################################

# # CREATE CONCAVE SHAPES
# name_in = 'models/snap-lock/snap-lock-arm.obj'
# name_out = 'models/snap-lock/snap-lock-arm-vhacd.obj'
# create_convex_vhacd(name_in, name_out, resolution=int(1e6))

#########################################################################
#########################################################################
#########################################################################

# FISH WITH RING GOT HOOKED

# import pybullet as p
# import time
# import numpy as np
# import matplotlib.pyplot as plt
# import os
# import subprocess
# import glob
# from utils import *

# p.connect(p.GUI)
# # p.setGravity(0, 0, -9.8)
# p.setTimeStep(1./240.)
# # p.setAdditionalSearchPath(pybullet_data.getDataPath())

# GRAVITY = -10
# p.setRealTimeSimulation(0)
# # bowl = p.loadURDF('models/bowl/bowl.urdf', (0,0,0), (0,0,1,1), globalScaling=5)
# # fish = p.loadURDF('models/fish/fishWithRing.xacro', (1,-2.1,1), (0,1,0,1))
# start = generate_circle_points(6, rad=0.8, z=.98)
# # start = [start[i]+.075 if i<2 else start[i] for i in range(len(start))]
# # ring = p.loadURDF('models/fish/ring2_vhacd.OBJ', (0,0,1), (0,0,1,1))

# # p.changeDynamics(bowl, -1, mass=0)

# # Upload the mesh data to PyBullet and create a static object
# # mesh_scale = [.04, .04, .04]  # The scale of the mesh
# # mesh_collision_shape = p.createCollisionShape(
# #     shapeType=p.GEOM_MESH,
# #     fileName="models/fish/ring2_vhacd.OBJ",
# #     meshScale=mesh_scale,
# #     # flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
# #     # meshData=mesh_data,
# # )
# # mesh_visual_shape = -1  # Use the same shape for visualization
# # mesh_position = [0.3, 0, 2]  # The position of the mesh
# # mesh_orientation = p.getQuaternionFromEuler([.6, 1.57, 0])  # The orientation of the mesh
# # ring = p.createMultiBody(
# #     baseMass=1.,
# #     baseCollisionShapeIndex=mesh_collision_shape,
# #     baseVisualShapeIndex=mesh_visual_shape,
# #     basePosition=mesh_position,
# #     baseOrientation=mesh_orientation,
# # )

# path = "models/hourglass/hourglass.obj"
# mesh_scale = [.1, .1, .1]  # The scale of the mesh
# # mesh_collision_shape = p.createCollisionShape(
# #     shapeType=p.GEOM_MESH,
# #     fileName=path,
# #     meshScale=mesh_scale,
# #     flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
# #     # meshData=mesh_data,
# # )
# # mesh_visual_shape = -1  # Use the same shape for visualization
# mesh_position = [0, 0, 0]  # The position of the mesh
# mesh_orientation = p.getQuaternionFromEuler([1.57, 0, 0])  # The orientation of the mesh
# # hook = p.createMultiBody(
# #     baseCollisionShapeIndex=mesh_collision_shape,
# #     baseVisualShapeIndex=mesh_visual_shape,
# #     basePosition=mesh_position,
# #     baseOrientation=mesh_orientation,
# # )

# mesh_collision_shape = p.createCollisionShape(
#     shapeType=p.GEOM_MESH,
#     fileName=path,
#     meshScale=mesh_scale,
#     flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
# )
# mesh_visual_shape = p.createVisualShape(shapeType=p.GEOM_MESH,
#     fileName=path,
#     rgbaColor=[1, 1, 1, 1],
#     specularColor=[0.4, .4, 0],
#     # visualFramePosition=shift,
#     meshScale=mesh_scale
# )
# # mesh_visual_shape = -1  # Use the same shape for visualization
# # mesh_position = pos  # The position of the mesh
# # mesh_orientation = qtn  # The orientation of the mesh
# obstacle_id = p.createMultiBody(
#     baseCollisionShapeIndex=mesh_collision_shape,
#     baseVisualShapeIndex=mesh_visual_shape,
#     basePosition=mesh_position,
#     baseOrientation=mesh_orientation,
# )


# def getJointStates(robot):
#   joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
#   joint_positions = [state[0] for state in joint_states]
#   joint_velocities = [state[1] for state in joint_states]
#   joint_torques = [state[3] for state in joint_states]
#   return joint_positions, joint_velocities, joint_torques

# # i = 0
# # jointPositionsSce = []
# # gemPosAll = []
# # gemOrnAll = []

# # viewMat = [
# #     0.642787516117096, -0.4393851161003113, 0.6275069713592529, 0.0, 0.766044557094574,
# #     0.36868777871131897, -0.5265407562255859, 0.0, -0.0, 0.8191521167755127, 0.5735764503479004,
# #     0.0, 2.384185791015625e-07, 2.384185791015625e-07, -5.000000476837158, 1.0
# # ]
# # width = 512 # 128
# # height = 512 # 128
# # folderName = './results/test/'

# while (1):
#     p.stepSimulation()
#     results = band_collision_raycast(start)
#     print(start)
#     print(results)

#     #p.setJointMotorControl2(botId, 1, p.TORQUE_CONTROL, force=1098.0)
#     # p.applyExternalTorque(mesh_id, -1, [1,0,0], p.WORLD_FRAME)
#     # print(gemPos, gemOrn)
#     p.setGravity(0, 0, GRAVITY)
#     # dep = i
#     # images = p.getCameraImage(width, height, viewMatrix=viewMat,
#     #                           )
#     # rgb_tiny = np.reshape(images[2], (height, width, 4)) * 1. / 255.
#     # fig = plt.figure(num=1, clear=True)
#     # ax = fig.add_subplot() 
#     # plt.text(width+10, height, 'Caging depth:{}'.format(dep))
#     # plt.text(width+10, height-20, 'curr depth:{}'.format(dep))
#     # plt.title('RGB')
#     # plt.imshow(rgb_tiny)
#     # plt.savefig(folderName + "file%03d.png" % i)
#     # plt.close()

#     # if i == 50:
#     #     os.chdir(folderName)
#     #     subprocess.call([
#     #         'ffmpeg', '-framerate', '8', '-i', 'file%03d.png', '-r', '30', '-pix_fmt', 'yuv420p',
#     #         'video_name.mp4'
#     #     ])
#     #     for file_name in glob.glob("*.png"):
#     #         os.remove(file_name)
    
#     # time.sleep(5/240.)

#     # i += 1
#     # print(i)
#     # CP = p.getClosestPoints(bodyA=fish, bodyB=mesh_id, distance=-0.01)
#     # if len(CP)>0:
#     #     dis = [CP[i][8] for i in range(len(CP))]
#     #     print('!!!!CP', dis)

#     # if i % 20 == 0:
#     #     jointPositions,_,_ = getJointStates(fish) # list(11)
#     #     gemPos, gemOrn = p.getBasePositionAndOrientation(fish) # tuple(3), tuple(4)
#     #     jointPositionsSce.append(jointPositions)
#     #     gemPosAll.append(list(gemPos))
#     #     gemOrnAll.append(list(gemOrn))
    
#     # if i == 500:
#     #     break
#     # # print(jointPositions)

# # # replay


#########################################################################
#########################################################################
#########################################################################

# SAVE TO CSV

# from main import argument_parser
# import argparse

# args, parser = argument_parser()
# print('@@@', args)

# parser.set_defaults(object='Fish')
# args = parser.parse_args()

# def flatten(l):
#     """
#     Flatten a nested list.
#     """
#     for i in l:
#         if isinstance(i, (list, tuple)):
#             for j in flatten(i):
#                 yield j
#         else:
#             yield i

# def flatten2(data):
#     flattened_data = []
#     for item in data:
#         if isinstance(item, list):
#             item = ','.join(map(str, item))
#         flattened_data.append(item)
#     return flattened_data

# def list2csv(l):
#     """
#     Return CSV-ish text for a nested list.
#     """
#     lines = []
#     for row in l:
#         if isinstance(row, (list, tuple)):
#             lines.append(",".join(str(i) for i in flatten(row)))
#         else:
#             lines.append(str(row))
#     return "\n".join(lines)

# data = [
#         1.0,
#         'One',
#         [1, 'Two'],
#         [1, 'Two', ['Three', 4.5]],
#         ['One', 2, [3.4, ['Five', 6]]]
#     ]


# import numpy as np
# data = [[1,2,None], [np.inf,2,3], [1,7,3,8], [1,2,4],[]]
# data=[['1', 'atul','tpa'],['2', 'carl','CN']]
# headers = ['serial', 'name', 'subject']
# print(list2csv(data))
# import csv
# from utils import flatten_nested_list

# # print(flatten_nested_list(data))

# # flattened_data = list2csv(data)

# # Save the flattened data to a CSV file
# import os
# os.mkdir('./results/fd/')
# with open('./results/fd/data.csv', 'w', newline='') as csvfile:
#     writer = csv.writer(csvfile)
#     writer.writerow(data)

# with open('./results/fd/data.csv', 'w', newline='') as csvfile:
#     writer = csv.writer(csvfile)
#     writer.writerow(i for i in headers)

# with open('./results/fd/data.csv', 'w', newline='') as csvfile:
#     writer = csv.writer(csvfile)
#     writer.writerow(data[0])
# with open('./results/fd/data.csv', 'w', newline='') as csvfile:
#     writer = csv.writer(csvfile)
#     writer.writerow(data[1])
# with open('./results/fd/info.txt', "w") as text_file:
#     text_file.write("goalCoMPose: {}\n".format(1))
#     text_file.write("goalCoMPose: {}\n".format(2))
#     text_file.write("goalCoMPose: {}\n".format(3))


#########################################################################
#########################################################################
#########################################################################

# PLOT ESCAPE CURVES

# import matplotlib.pyplot as plt
# from visualization import *

# folderName = './results/HookTrapsRing_16-03-2023-09-48-19/'
# plot_escape_energy_from_csv(args, folderName, isArticulatedObject=0)
# _, ax1 = plt.subplots()
# ax1.plot([1,2,3,np.inf,5,991,3,5,7,8,9], 'r--', label='Escape energy')
# ax1.set_xlabel('# iterations')
# ax1.set_ylabel('G-potential energy')
# ax1.grid(True)
# plt.xticks(np.linspace(0,111,10).astype(int))
# ax1.legend()
# plt.title('Escape energy in a dynamic scenario - fish falls into a bowl')
# plt.show()




# import matplotlib.pyplot as plt
# fig = plt.figure(constrained_layout=True)
# fig.set_size_inches(16,7)
# axs = fig.subplot_mosaic([['Left', 'A', 'B'],['Left', 'C', 'D']],
#                           gridspec_kw={'width_ratios':[2,1,1]})
# axs['Left'].set_title('Plot on Left')
# axs['Left'].set_aspect(30)

# axs['A'].set_title('A')
# axs['B'].set_title('B')
# axs['C'].set_title('C')
# axs['D'].set_title('D')
# plt.show()


#########################################################################
#########################################################################
#########################################################################

# import pybullet as p
# import pybullet_data
# import time 

# p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
# useMaximalCoordinates = False
# id1 = p.loadURDF("plane.urdf", useMaximalCoordinates=useMaximalCoordinates)
# id2 = p.loadURDF("sphere_1cm.urdf",[0,0,1], globalScaling=10)
# # p.loadURDF("cube.urdf", [0, 0, 1], useMaximalCoordinates=useMaximalCoordinates)
# p.setGravity(0, 0, -10)
# while (1):
#   p.stepSimulation()
#   print(p.getClosestPoints(id1,id2,0))
#   time.sleep(1/240)

#   print("num pts=", len(pts))
#   totalNormalForce = 0
#   totalFrictionForce = [0, 0, 0]
#   totalLateralFrictionForce = [0, 0, 0]
#   for pt in pts:
#     #print("pt.normal=",pt[7])
#     #print("pt.normalForce=",pt[9])
#     totalNormalForce += pt[9]
#     #print("pt.lateralFrictionA=",pt[10])
#     #print("pt.lateralFrictionADir=",pt[11])
#     #print("pt.lateralFrictionB=",pt[12])
#     #print("pt.lateralFrictionBDir=",pt[13])
#     totalLateralFrictionForce[0] += pt[11][0] * pt[10] + pt[13][0] * pt[12]
#     totalLateralFrictionForce[1] += pt[11][1] * pt[10] + pt[13][1] * pt[12]
#     totalLateralFrictionForce[2] += pt[11][2] * pt[10] + pt[13][2] * pt[12]

#   print("totalNormalForce=", totalNormalForce)
#   print("totalLateralFrictionForce=", totalLateralFrictionForce)


#########################################################################
#########################################################################
#########################################################################

# try:
#     from ompl import util as ou
#     from ompl import base as ob
#     from ompl import geometric as og
# except ImportError:
#     # if the ompl module is not in the PYTHONPATH assume it is installed in a
#     # subdirectory of the parent directory called "py-bindings."
#     from os.path import abspath, dirname, join
#     import sys
#     sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'ompl/py-bindings'))
#     # sys.path.insert(0, join(dirname(abspath(__file__)), '../whole-body-motion-planning/src/ompl/py-bindings'))
#     print(sys.path)
#     from ompl import util as ou
#     from ompl import base as ob
#     from ompl import geometric as og
# import numpy as np
# import sys
# import kinpy as kp
# from scipy.spatial.transform import Rotation as R
# from utils import path_collector
# import pybullet as p

# class RopePotentialObjective():
#     def __init__(self, linkLen):
#         # super(RopePotentialObjective, self).__init__(si)
#         # self.si_ = si
#         # self.start_ = start
#         self.linkLen_ = linkLen # fixed-length rope links
#         # self.numStateSpace_ = len(start)
#         self.baseDof_ = 6
#         self.ctrlPointDof_ = 2
#         self.numCtrlPoint_ = int((self.numStateSpace_-self.baseDof_) / self.ctrlPointDof_) ###

#         self.TLastRow_ = np.array([[0., 0., 0., 1.]]) # 1*4
#         self.nextFPosInThisF_ = np.array([[0.], [0.], [self.linkLen_], [1.]]) # 4*1
#         self.framePositionsInWorld = [] # list of (3,) numpy arrays
#         self.framePosZsInWorld = []
#         # self.startStateEnergy = self.stateEnergy(self.start_)

#     def ropeForwardKinematics(self, state):
#         # Retrieve object's base transform
#         basePosInWorld = np.array(state[0:3]).reshape((3))
#         baseEulInWorld = state[3:6] # euler
#         baseQuatInWorld = p.getQuaternionFromEuler(baseEulInWorld)
#         r = R.from_quat(baseQuatInWorld) # BUG: quat to euler translation causes mistakes!
#         mat = r.as_matrix() # (3,3)
#         first3Rows = (mat, basePosInWorld.reshape((3,1)))
#         first3Rows = np.hstack(first3Rows) # (3,4). first 3 rows of Transform matrix
#         baseTInWorld = np.vstack((first3Rows, self.TLastRow_)) # (4,4)
#         F0PosInWorld = baseTInWorld @ self.nextFPosInThisF_ # (4,1)
#         F0PosInWorld = F0PosInWorld[:3].reshape((3))

#         # Record
#         self.framePositionsInWorld.append(basePosInWorld)
#         self.framePositionsInWorld.append(F0PosInWorld)
#         self.framePosZsInWorld.append(float(basePosInWorld[2]))
#         self.framePosZsInWorld.append(float(F0PosInWorld[2]))

#         # Iterate over control points
#         Fi_1TInWorld = baseTInWorld
#         for i in range(self.numCtrlPoint_):
#             # Fi_1: F_{i-1} the (i-1)'th frame, F_{-1} is base frame
#             # Fip1: F_{i+1} the (i+1)'th frame

#             # Build local rotation matrix
#             FiEulInFi_1 = state[6+2*i:6+2*(i+1)] ###
#             FiEulInFi_1.append(0.) # euler
#             FiQuatInFi_1 = p.getQuaternionFromEuler(FiEulInFi_1)
#             r = R.from_quat(FiQuatInFi_1)
#             mat = r.as_matrix() # (3,3)

#             # Build global transformation matrix
#             first3Rows = (mat, np.array(self.nextFPosInThisF_[:3]))
#             first3Rows = np.hstack(first3Rows) # (3,4). first 3 rows of Transform matrix
#             FiTInFi_1 = np.vstack((first3Rows, self.TLastRow_)) # (4,4)
#             FiTInWorld = Fi_1TInWorld @ FiTInFi_1 # (4,4)
#             Fip1PosInWorld = FiTInWorld @ self.nextFPosInThisF_ # (4,1)
#             Fip1PosInWorld = Fip1PosInWorld[:3].reshape((3)) # (3,)

#             # Record
#             self.framePositionsInWorld.append(Fip1PosInWorld)
#             self.framePosZsInWorld.append(float(Fip1PosInWorld[2]))
            
#             # Update last transformation matrix
#             Fi_1TInWorld = FiTInWorld

# if __name__ == '__main__':
#     start = [1,1,1,0,0,0] + [0,np.pi/6]*6
#     linkLen = 1
#     rope = RopePotentialObjective(linkLen)
    
#     rope.ropeForwardKinematics(start)

#     print('framePositionsInWorld', rope.framePositionsInWorld)
#     print('framePosZsInWorld',rope.framePosZsInWorld)
#     print(rope.numCtrlPoint_)


#########################################################################
#########################################################################
#########################################################################
# Rope and band test
import os.path as osp
import sys
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))

import pybullet as p
import pybullet_data
import time
import numpy as np
import matplotlib.pyplot as plt
import os
import subprocess
import glob
from utils import *
from cagingSearchAlgo import *
from pbOmplInterface import *

# p.connect(p.GUI)
# # p.setGravity(0, 0, -9.8)
# p.setTimeStep(1./240.)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())

# p.setRealTimeSimulation(0)
# # id2 = p.loadURDF("sphere_1cm.urdf",[0,0,1], globalScaling=150)
# id2 = p.loadURDF("plane.urdf",[0,0,1], globalScaling=1)

args, parser = argument_parser()
# basePosBounds=[[-5,5], [-5,5], [-3,5]] # searching bounds

# create caging environment and items in pybullet
if args.object in get_non_articulated_objects():
    env = RigidObjectCaging(args)
    env.add_obstacles(scale=[.1]*3, pos=[0,0,0], qtn=p.getQuaternionFromEuler([1.57, 0, 0]))

elif args.object == 'Fish':
    objScale = 1
    env = ArticulatedObjectCaging(args, objScale)
    env.add_obstacles(scale=[1]*3, pos=[0,0,0], qtn=p.getQuaternionFromEuler([0, 0, 0]))

elif args.object == 'Band':
    numCtrlPoint = 6
    start = generate_circle_points(numCtrlPoint, rad=.8, z=0.98)
    goal = [0,0,2.18] * numCtrlPoint
    env = ElasticBandCaging(args, numCtrlPoint, start, goal)
    env.add_obstacles(scale=[1]*3, pos=[0,0,0], qtn=p.getQuaternionFromEuler([0, 0, 0]))

elif args.object == 'Rope':
    numCtrlPoint = 6
    linkLen = 0.2
    start = [0,0,.7,0,0,0] + [0,0]*numCtrlPoint
    goal = [0,0,.1,1.57,0,0] + [0,0]*numCtrlPoint
    env = RopeCaging(args, numCtrlPoint, linkLen, start, goal)
    # env.add_obstacles(scale=[.03, .03, .1], pos=[0,0,-0.5], qtn=p.getQuaternionFromEuler([0, 0, 0])) # box
    # env.add_obstacles(scale=[.3]*3, pos=[0,0,1], qtn=p.getQuaternionFromEuler([0, 0, 0])) # bowl
    env.add_obstacles(scale=[1]*3, pos=[0,0,.5], qtn=p.getQuaternionFromEuler([0, 0, 0])) # bucket

elif args.object == 'Snaplock':
    objScale = 3
    env = ArticulatedObjectCaging(args, objScale)
    env.add_obstacles(scale=[.1]*3, pos=[-.5,0,3], qtn=p.getQuaternionFromEuler([0, 0, 0]))
    env.robot.set_search_bounds([[-2,2], [-2,2], [0,3.5]])
    env.reset_start_and_goal(start=[0,0,1.5,0,0,1.57]+[0], goal=[0,0,.01]+[0,1.57,0]+[0])

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

env.pb_ompl_interface = PbOMPL(env.robot, args, env.obstacles)

i=0
'''Snaplock test'''
# while (1):
#     p.stepSimulation()
#     state = [0,0,1,0,0,0] + [0,0]*numCtrlPoint
#     # state = [-0.3,0,1.5,0,0,0]+[0,0,0,0,0,0,0,0,-.0+0.0*i] # 10-link fish
#     env.robot.set_state(state)
#     print(env.pb_ompl_interface.is_state_valid(state))
#     # rope_collision_raycast(state, linkLen, rayHitColor=[1,0,0], rayMissColor=[0,1,0], visRays=1)
#     # goal = [0.5,.5,.1,0,i*np.pi/60,0] + [0,0]*numCtrlPoint
#     # rope_collision_raycast(goal, linkLen, rayHitColor=[1,0,0], rayMissColor=[0,1,0], visRays=1)
#     i += 1
#     sleep(.03)

# '''Rope test'''
# while (1):
#     p.stepSimulation()
#     # start = [0.1,0,1.1,0,20*np.pi/60,0] + [0,0]*numCtrlPoint
#     state = [0,0,0]+[0,0,0] + [0,0] + [0,i*np.pi/60] + [0,0]*(numCtrlPoint-2)
#     start = [0,0,.7,1.57,0,0] + [0,0]*numCtrlPoint
#     # print(env.pb_ompl_interface.is_state_valid(state))
#     rope_collision_raycast(state, linkLen, rayHitColor=[1,0,0], rayMissColor=[0,1,0], visRays=1)
#     # goal = [0.5,.5,.1,0,i*np.pi/60,0] + [0,0]*numCtrlPoint
#     # rope_collision_raycast(goal, linkLen, rayHitColor=[1,0,0], rayMissColor=[0,1,0], visRays=1)
#     i += 1

# # '''Band test'''
# while (1):
#     p.stepSimulation()
#     start = generate_circle_points(numCtrlPoint, rad=1.4-i/100, z=1.2)
#     # state = [-.2,-.2,1.5,0,i*np.pi/60,0] + [0,0]*numCtrlPoint
#     # print('is_state_valid: ', env.pb_ompl_interface.is_state_valid(start))
#     band_collision_raycast(start, visRays=1)
#     i += 1
#     if 1.2-i/100 < 0:
#         break

# '''Jelly test'''
while (1):
    p.stepSimulation()
    ofs = 0.7 - 0.005*i
    goal = [-l/2-ofs,-l/2-ofs,-l/2+zs] + [-l/2-ofs,-l/2-ofs,l/2+zs] + [l/2-ofs,-l/2-ofs,l/2+zs] + [-l/2-ofs,l/2-ofs,l/2+zs]
    # print('is_state_valid: ', env.pb_ompl_interface.is_state_valid(start))
    jelly_collision_raycast(goal, visRays=1)
    i += 1

# import sys
# try:
#     from ompl import util as ou
#     from ompl import base as ob
#     from ompl import geometric as og
# except ImportError:
#     # if the ompl module is not in the PYTHONPATH assume it is installed in a
#     # subdirectory of the parent directory called "py-bindings."
#     from os.path import abspath, dirname, join
#     sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
#     from ompl import util as ou
#     from ompl import base as ob
#     from ompl import geometric as og
# from math import sqrt
# import argparse

# ## @cond IGNORE
# # Our "collision checker". For this demo, our robot's state space
# # lies in [0,1]x[0,1], with a circular obstacle of radius 0.25
# # centered at (0.5,0.5). Any states lying in this circular region are
# # considered "in collision".
# class ValidityChecker(ob.StateValidityChecker):
#     # Returns whether the given state's position overlaps the
#     # circular obstacle
#     def isValid(self, state):
#         return self.clearance(state) > 0.0

#     # Returns the distance from the given state's position to the
#     # boundary of the circular obstacle.
#     def clearance(self, state):
#         # Extract the robot's (x,y) position from its state
#         x = state[0]
#         y = state[1]

#         # Distance formula between two points, offset by the circle's
#         # radius
#         return sqrt((x-0.5)*(x-0.5) + (y-0.)*(y-0.)) - 0.45


# ## Returns a structure representing the optimization objective to use
# #  for optimal motion planning. This method returns an objective
# #  which attempts to minimize the length in configuration space of
# #  computed paths.
# def getPathLengthObjective(si):
#     return ob.PathLengthOptimizationObjective(si)

# ## Returns an optimization objective which attempts to minimize path
# #  length that is satisfied when a path of length shorter than 1.51
# #  is found.
# def getThresholdPathLengthObj(si):
#     obj = ob.PathLengthOptimizationObjective(si)
#     obj.setCostThreshold(ob.Cost(1.51))
#     return obj

# ## Defines an optimization objective which attempts to steer the
# #  robot away from obstacles. To formulate this objective as a
# #  minimization of path cost, we can define the cost of a path as a
# #  summation of the costs of each of the states along the path, where
# #  each state cost is a function of that state's clearance from
# #  obstacles.
# #
# #  The class StateCostIntegralObjective represents objectives as
# #  summations of state costs, just like we require. All we need to do
# #  then is inherit from that base class and define our specific state
# #  cost function by overriding the stateCost() method.
# #
# class minPathPotentialObjective(ob.OptimizationObjective):
#     def __init__(self, si, start):
#         super(minPathPotentialObjective, self).__init__(si)
#         self.si_ = si
#         self.start_ = start

#     # Our requirement is to maximize path clearance from obstacles,
#     # but we want to represent the objective as a path cost
#     # minimization. Therefore, we set each state's cost to be the
#     # reciprocal of its clearance, so that as state clearance
#     # increases, the state cost decreases.
#     # def stateCost(self, s):
#     #     return ob.Cost(1 / (self.si_.getStateValidityChecker().clearance(s) +
#     #                         sys.float_info.min))

#     def combineCosts(self, c1, c2):
#         return ob.Cost(max(c1.value(), c2.value()))

#     def motionCost(self, s1, s2):
#         return ob.Cost(s2[1] - self.start_[1])
    
    
# ## Return an optimization objective which attempts to steer the robot
# #  away from obstacles.
# def getClearanceObjective(si, start):
#     return minPathPotentialObjective(si, start)

# ## Create an optimization objective which attempts to optimize both
# #  path length and clearance. We do this by defining our individual
# #  objectives, then adding them to a MultiOptimizationObjective
# #  object. This results in an optimization objective where path cost
# #  is equivalent to adding up each of the individual objectives' path
# #  costs.
# #
# #  When adding objectives, we can also optionally specify each
# #  objective's weighting factor to signify how important it is in
# #  optimal planning. If no weight is specified, the weight defaults to
# #  1.0.
# def getBalancedObjective1(si):
#     lengthObj = ob.PathLengthOptimizationObjective(si)
#     clearObj = minPathPotentialObjective(si)

#     opt = ob.MultiOptimizationObjective(si)
#     opt.addObjective(lengthObj, 5.0)
#     opt.addObjective(clearObj, 1.0)

#     return opt

# ## Create an optimization objective equivalent to the one returned by
# #  getBalancedObjective1(), but use an alternate syntax.
# #  THIS DOESN'T WORK YET. THE OPERATORS SOMEHOW AREN'T EXPORTED BY Py++.
# # def getBalancedObjective2(si):
# #     lengthObj = ob.PathLengthOptimizationObjective(si)
# #     clearObj = ClearanceObjective(si)
# #
# #     return 5.0*lengthObj + clearObj


# ## Create an optimization objective for minimizing path length, and
# #  specify a cost-to-go heuristic suitable for this optimal planning
# #  problem.
# def getPathLengthObjWithCostToGo(si):
#     obj = ob.PathLengthOptimizationObjective(si)
#     obj.setCostToGoHeuristic(ob.CostToGoHeuristic(ob.goalRegionCostToGo))
#     return obj


# # Keep these in alphabetical order and all lower case
# def allocatePlanner(si, plannerType):
#     if plannerType.lower() == "bfmtstar":
#         return og.BFMT(si)
#     elif plannerType.lower() == "bitstar":
#         return og.BITstar(si)
#     elif plannerType.lower() == "fmtstar":
#         return og.FMT(si)
#     elif plannerType.lower() == "informedrrtstar":
#         return og.InformedRRTstar(si)
#     elif plannerType.lower() == "prmstar":
#         return og.PRMstar(si)
#     elif plannerType.lower() == "rrtstar":
#         return og.RRTstar(si)
#     elif plannerType.lower() == "sorrtstar":
#         return og.SORRTstar(si)
#     else:
#         ou.OMPL_ERROR("Planner-type is not implemented in allocation function.")


# # Keep these in alphabetical order and all lower case
# def allocateObjective(si, objectiveType, start):
#     if objectiveType.lower() == "pathclearance":
#         return getClearanceObjective(si, start)
#     elif objectiveType.lower() == "pathlength":
#         return getPathLengthObjective(si)
#     elif objectiveType.lower() == "thresholdpathlength":
#         return getThresholdPathLengthObj(si)
#     elif objectiveType.lower() == "weightedlengthandclearancecombo":
#         return getBalancedObjective1(si)
#     else:
#         ou.OMPL_ERROR("Optimization-objective is not implemented in allocation function.")



# def plan(runTime, plannerType, objectiveType, fname):
#     # Construct the robot state space in which we're planning. We're
#     # planning in [0,1]x[0,1], a subset of R^2.
#     space = ob.RealVectorStateSpace(2)

#     # Set the bounds of space to be in [0,1].
#     space.setBounds(0.0, 1.0)

#     # Construct a space information instance for this state space
#     si = ob.SpaceInformation(space)

#     # Set the object used to check which states in the space are valid
#     validityChecker = ValidityChecker(si)
#     si.setStateValidityChecker(validityChecker)

#     si.setup()

#     # Set our robot's starting state to be the bottom-left corner of
#     # the environment, or (0,0).
#     start = ob.State(space)
#     start[0] = 0.0
#     start[1] = 0.0

#     # Set our robot's goal state to be the top-right corner of the
#     # environment, or (1,1).
#     goal = ob.State(space)
#     goal[0] = 1.0
#     goal[1] = 0.0

#     # Create a problem instance
#     pdef = ob.ProblemDefinition(si)

#     # Set the start and goal states
#     pdef.setStartAndGoalStates(start, goal)

#     # Create the optimization objective specified by our command-line argument.
#     # This helper function is simply a switch statement.
#     pdef.setOptimizationObjective(allocateObjective(si, objectiveType, start))

#     # Construct the optimal planner specified by our command line argument.
#     # This helper function is simply a switch statement.
#     optimizingPlanner = allocatePlanner(si, plannerType)

#     # Set the problem instance for our planner to solve
#     optimizingPlanner.setProblemDefinition(pdef)
#     optimizingPlanner.setup()

#     # attempt to solve the planning problem in the given runtime
#     solved = optimizingPlanner.solve(runTime)

#     if solved:
#         # Output the length of the path found
#         print('{0} found solution of path length {1:.4f} with an optimization ' \
#             'objective value of {2:.4f}'.format( \
#             optimizingPlanner.getName(), \
#             pdef.getSolutionPath().length(), \
#             pdef.getSolutionPath().cost(pdef.getOptimizationObjective()).value()))
#         print(pdef.getSolutionPath())

#         # If a filename was specified, output the path as a matrix to
#         # that file for visualization
#         if fname:
#             with open(fname, 'w') as outFile:
#                 outFile.write(pdef.getSolutionPath().printAsMatrix())
#     else:
#         print("No solution found.")

# if __name__ == "__main__":
#     # Create an argument parser
#     parser = argparse.ArgumentParser(description='Optimal motion planning demo program.')

#     # Add a filename argument
#     parser.add_argument('-t', '--runtime', type=float, default=1.0, help=\
#         '(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.')
#     parser.add_argument('-p', '--planner', default='BITstar', \
#         choices=['BFMTstar', 'BITstar', 'FMTstar', 'InformedRRTstar', 'PRMstar', 'RRTstar', \
#         'SORRTstar'], \
#         help='(Optional) Specify the optimal planner to use, defaults to RRTstar if not given.')
#     parser.add_argument('-o', '--objective', default='PathClearance', \
#         choices=['PathClearance', 'PathLength', 'ThresholdPathLength', \
#         'WeightedLengthAndClearanceCombo'], \
#         help='(Optional) Specify the optimization objective, defaults to PathLength if not given.')
#     parser.add_argument('-f', '--file', default=None, \
#         help='(Optional) Specify an output path for the found solution path.')
#     parser.add_argument('-i', '--info', type=int, default=0, choices=[0, 1, 2], \
#         help='(Optional) Set the OMPL log level. 0 for WARN, 1 for INFO, 2 for DEBUG.' \
#         ' Defaults to WARN.')

#     # Parse the arguments
#     args = parser.parse_args()

#     # Check that time is positive
#     if args.runtime <= 0:
#         raise argparse.ArgumentTypeError(
#             "argument -t/--runtime: invalid choice: %r (choose a positive number greater than 0)" \
#             % (args.runtime,))

#     # Set the log level
#     if args.info == 0:
#         ou.setLogLevel(ou.LOG_WARN)
#     elif args.info == 1:
#         ou.setLogLevel(ou.LOG_INFO)
#     elif args.info == 2:
#         ou.setLogLevel(ou.LOG_DEBUG)
#     else:
#         ou.OMPL_ERROR("Invalid log-level integer.")

#     # Solve the planning problem
#     plan(args.runtime, args.planner, args.objective, args.file)




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
# CREATE CONCAVE SHAPES
#  
# from utils import *
# name_in = 'models/bust/female_bust.obj'
# name_out = 'models/bust/female_bust_vhacd.obj'
# create_convex_vhacd(name_in, name_out, resolution=int(1e6))

#########################################################################
# FISH WITH RING GOT HOOKED

import pybullet as p
import time
import numpy as np
import matplotlib.pyplot as plt
import os
import subprocess
import glob
from utils import *

p.connect(p.GUI)
# p.setGravity(0, 0, -9.8)
p.setTimeStep(1./240.)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())

GRAVITY = -10
p.setRealTimeSimulation(0)
# bowl = p.loadURDF('models/bowl/bowl.urdf', (0,0,0), (0,0,1,1), globalScaling=5)
# fish = p.loadURDF('models/fish/fishWithRing.xacro', (1,-2.1,1), (0,1,0,1))
start = generate_circle_points(6, rad=0.8, z=.98)
# start = [start[i]+.075 if i<2 else start[i] for i in range(len(start))]
# ring = p.loadURDF('models/fish/ring2_vhacd.OBJ', (0,0,1), (0,0,1,1))

# p.changeDynamics(bowl, -1, mass=0)

# Upload the mesh data to PyBullet and create a static object
# mesh_scale = [.04, .04, .04]  # The scale of the mesh
# mesh_collision_shape = p.createCollisionShape(
#     shapeType=p.GEOM_MESH,
#     fileName="models/fish/ring2_vhacd.OBJ",
#     meshScale=mesh_scale,
#     # flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
#     # meshData=mesh_data,
# )
# mesh_visual_shape = -1  # Use the same shape for visualization
# mesh_position = [0.3, 0, 2]  # The position of the mesh
# mesh_orientation = p.getQuaternionFromEuler([.6, 1.57, 0])  # The orientation of the mesh
# ring = p.createMultiBody(
#     baseMass=1.,
#     baseCollisionShapeIndex=mesh_collision_shape,
#     baseVisualShapeIndex=mesh_visual_shape,
#     basePosition=mesh_position,
#     baseOrientation=mesh_orientation,
# )

path = "models/hourglass/hourglass.obj"
mesh_scale = [.1, .1, .1]  # The scale of the mesh
# mesh_collision_shape = p.createCollisionShape(
#     shapeType=p.GEOM_MESH,
#     fileName=path,
#     meshScale=mesh_scale,
#     flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
#     # meshData=mesh_data,
# )
# mesh_visual_shape = -1  # Use the same shape for visualization
mesh_position = [0, 0, 0]  # The position of the mesh
mesh_orientation = p.getQuaternionFromEuler([1.57, 0, 0])  # The orientation of the mesh
# hook = p.createMultiBody(
#     baseCollisionShapeIndex=mesh_collision_shape,
#     baseVisualShapeIndex=mesh_visual_shape,
#     basePosition=mesh_position,
#     baseOrientation=mesh_orientation,
# )

mesh_collision_shape = p.createCollisionShape(
    shapeType=p.GEOM_MESH,
    fileName=path,
    meshScale=mesh_scale,
    flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
)
mesh_visual_shape = p.createVisualShape(shapeType=p.GEOM_MESH,
    fileName=path,
    rgbaColor=[1, 1, 1, 1],
    specularColor=[0.4, .4, 0],
    # visualFramePosition=shift,
    meshScale=mesh_scale
)
# mesh_visual_shape = -1  # Use the same shape for visualization
# mesh_position = pos  # The position of the mesh
# mesh_orientation = qtn  # The orientation of the mesh
obstacle_id = p.createMultiBody(
    baseCollisionShapeIndex=mesh_collision_shape,
    baseVisualShapeIndex=mesh_visual_shape,
    basePosition=mesh_position,
    baseOrientation=mesh_orientation,
)


def getJointStates(robot):
  joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
  joint_positions = [state[0] for state in joint_states]
  joint_velocities = [state[1] for state in joint_states]
  joint_torques = [state[3] for state in joint_states]
  return joint_positions, joint_velocities, joint_torques

# i = 0
# jointPositionsSce = []
# gemPosAll = []
# gemOrnAll = []

# viewMat = [
#     0.642787516117096, -0.4393851161003113, 0.6275069713592529, 0.0, 0.766044557094574,
#     0.36868777871131897, -0.5265407562255859, 0.0, -0.0, 0.8191521167755127, 0.5735764503479004,
#     0.0, 2.384185791015625e-07, 2.384185791015625e-07, -5.000000476837158, 1.0
# ]
# width = 512 # 128
# height = 512 # 128
# folderName = './results/test/'

while (1):
    p.stepSimulation()
    results = band_collision_raycast(start)
    print(start)
    print(results)

    #p.setJointMotorControl2(botId, 1, p.TORQUE_CONTROL, force=1098.0)
    # p.applyExternalTorque(mesh_id, -1, [1,0,0], p.WORLD_FRAME)
    # print(gemPos, gemOrn)
    p.setGravity(0, 0, GRAVITY)
    # dep = i
    # images = p.getCameraImage(width, height, viewMatrix=viewMat,
    #                           )
    # rgb_tiny = np.reshape(images[2], (height, width, 4)) * 1. / 255.
    # fig = plt.figure(num=1, clear=True)
    # ax = fig.add_subplot() 
    # plt.text(width+10, height, 'Caging depth:{}'.format(dep))
    # plt.text(width+10, height-20, 'curr depth:{}'.format(dep))
    # plt.title('RGB')
    # plt.imshow(rgb_tiny)
    # plt.savefig(folderName + "file%03d.png" % i)
    # plt.close()

    # if i == 50:
    #     os.chdir(folderName)
    #     subprocess.call([
    #         'ffmpeg', '-framerate', '8', '-i', 'file%03d.png', '-r', '30', '-pix_fmt', 'yuv420p',
    #         'video_name.mp4'
    #     ])
    #     for file_name in glob.glob("*.png"):
    #         os.remove(file_name)
    
    # time.sleep(5/240.)

    # i += 1
    # print(i)
    # CP = p.getClosestPoints(bodyA=fish, bodyB=mesh_id, distance=-0.01)
    # if len(CP)>0:
    #     dis = [CP[i][8] for i in range(len(CP))]
    #     print('!!!!CP', dis)

    # if i % 20 == 0:
    #     jointPositions,_,_ = getJointStates(fish) # list(11)
    #     gemPos, gemOrn = p.getBasePositionAndOrientation(fish) # tuple(3), tuple(4)
    #     jointPositionsSce.append(jointPositions)
    #     gemPosAll.append(list(gemPos))
    #     gemOrnAll.append(list(gemOrn))
    
    # if i == 500:
    #     break
    # # print(jointPositions)

# # replay


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
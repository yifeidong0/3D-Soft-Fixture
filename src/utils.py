
"""
Title: collection of utility functions
Author: Yifei Dong
Date: 14/07/2023
Description: This script contains most utility functions in this repo. Adapted from
https://github.com/StanfordVL/iGibson/blob/master/igibson/external/pybullet_tools/utils.py
"""

from __future__ import print_function
import pybullet as p
from collections import defaultdict, deque, namedtuple
from itertools import product, combinations, count
import argparse
import math
import numpy as np
from numpy import linalg as LA
from itertools import product
import copy
import kinpy as kp
from scipy.spatial.transform import Rotation as R

BASE_LINK = -1
MAX_DISTANCE = -0.025
SCENARIO_ID = 3

def argument_parser():
    '''
    Hyperparemeter setup.
    '''
    # Create an argument parser
    parser = argparse.ArgumentParser(description='3D energy-bounded caging demo program.')

    # Add a filename argument
    # 1. ShovelFish - Fish, Shovel
    # 2. MaskEar - MaskBand, Ear
    # 3. StarfishBowl - Starfish, LeftHandAndBowl
    # 4. HookFishHole - FishHole, Hook
    # 5. HandbagGripper - Chain, 3fGripper
    # 6. BimanualRubic - Rubic, 3fGripper
    parser.add_argument('-c', '--scenario', default='ShovelFish', \
        choices=['FishFallsInBowl', 'HookTrapsRing', 'GripperClenchesStarfish', 'BustTrapsBand', \
                 'RopeBucket', 'BandHourglass', 'JellyMaze', '2DSnapLock', '3DSnapLock', \
                 'StarfishBowl', 'HookFishHole', 'ShovelFish', 'BimanualRubic', 'HandbagGripper', \
                 'MaskEar'], \
        help='(Optional) Specify the scenario of demo, defaults to FishFallsInBowl if not given.')

    parser.add_argument('-s', '--search', default='EnergyBiasedSearch', \
        choices=['BisectionSearch', 'EnergyBiasedSearch'], \
        help='(Optional) Specify the sampling-based search method to use, defaults to BisectionSearch if not given.')
    
    parser.add_argument('-p', '--planner', default='BITstar', \
        choices=['BFMTstar', 'BITstar', 'FMT', 'InformedRRTstar', 'PRMstar', 'RRTstar', \
        'SORRTstar', 'RRT', 'AITstar', 'LBTRRT'], \
        help='(Optional) Specify the optimal planner to use, defaults to RRTstar if not given.')

    parser.add_argument('-j', '--object', default='Fish', \
        choices=['Fish', 'FishWithRing', 'Starfish', 'Ring', 'Band', 'BandHorizon', 'MaskBand', 'Rope', 'Humanoid', 'Donut', \
                 'Jelly', '3fGripper', 'PlanarRobot', 'Snaplock', 'PandaArm', 'FishHole', '2Dlock', \
                 'Rubic', 'Chain', 'Ftennis', 'Ftape', 'Fglue', 'Fbanana', 'Donut45', 'Donut60', 'Donut90', 'Donut120',], \
        help='(Optional) Specify the object to cage.')

    parser.add_argument('-l', '--obstacle', default='Shovel', \
        choices=['Box', 'Hook', '3fGripper', 'Bowl', 'Bust', 'Hourglass', 'Ring', 'Hole', \
                 'Maze', '2Dkey', 'SplashBowl', 'Radish', 'Shovel', 'LeftHand', 'LeftHandAndBowl', \
                 'ShadowHand', 'Bucket', 'Ear', 'FbowlS', 'FbowlM', 'FbowlL',], \
        help='(Optional) Specify the obstacle that cages the object.')
    
    parser.add_argument('-t', '--runtime', type=float, default=30, help=\
        '(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0. (In the current settings, 240 s not better a lot than 120 s)')
    
    parser.add_argument('-v', '--visualization', type=bool, default=1, help=\
        '(Optional) Specify whether to visualize the pybullet GUI. Defaults to False and must be False or True.')

    # Parse the arguments
    args = parser.parse_args()

    return args, parser

def path_collector():
    '''
    Paths of all models.
    '''
    return {
            'Fish': 'models/fine-fish-10/fine-fish-10.urdf', 
            'FishHole': 'models/fish-hole/fish-hole-rigid.urdf', 
            'Starfish': 'models/starfish-soft/starfish-soft-16dof.urdf', 
            'Ring': 'models/fish/ring2.urdf', 
            'Donut': 'models/donut/donut.urdf',
            '3fGripper': 'models/robotiq_3f_gripper_visualization/cfg/robotiq-3f-gripper_articulated_decimate.urdf',
            'ShadowHand': 'models/sr_description/hand/xacro/hand_c.urdf.xacro',
            'PandaArm': 'models/franka_description/robots/panda_arm.urdf',
            'PlanarRobot': 'models/planar_robot_4_link.xacro',
            'Humanoid': 'models/humanoid.urdf',
            'Bowl': 'models/bowl/bowl_ceramics.stl', 
            'SplashBowl': 'models/splash-bowl/splash-bowl.stl', 
            'Hook': 'models/triple_hook/triple_hook_vhacd.obj', 
            'Bust': 'models/bust/female_bust.obj',
            'Bucket': 'models/bucket/bucket-cutPoly.stl',
            'Hourglass': 'models/hourglass/half-hourglass-downsized.stl',
            'Snaplock': 'models/snap-lock/snap-lock.urdf', 
            'Maze': 'models/maze/maze_cutpoly.stl',
            '2Dlock': 'models/2Dsnap-lock/snap-lock.urdf',
            '2Dkey': 'models/2Dsnap-lock/p3-vhacd.obj',
            'Radish': 'models/radish/radish.stl',
            'LeftHand': 'models/hand/hand-d-vhacd.obj',
            'Shovel': 'models/shovel/shovel-d-vhacd.obj',
            'LeftHandAndBowl': 'models/leftHandAndBowl/leftHandAndBowl-vhacd.obj',
            'Rubic': 'models/rubic-cube/rubic.urdf',
            'Ear': 'models/ear/ear.stl',
            'Ftape': 'models/physical-experiment/downsampled4blender/Ftape.urdf',
            'Fglue': 'models/physical-experiment/downsampled4blender/Fglue.urdf',
            'Ftennis': 'models/physical-experiment/downsampled4blender/Ftennis.urdf',
            'Fbanana': 'models/physical-experiment/downsampled4blender/Fbanana.urdf',
            'Donut45': 'models/physical-experiment/downsampled4blender/Donut45.urdf',
            'Donut60': 'models/physical-experiment/downsampled4blender/Donut60.urdf',
            'Donut90': 'models/physical-experiment/downsampled4blender/Donut90.urdf',
            'Donut120': 'models/physical-experiment/downsampled4blender/Donut120.urdf',
            'FbowlS': 'models/physical-experiment/downsampled4blender/FbowlS.stl',
            'FbowlM': 'models/physical-experiment/downsampled4blender/FbowlM.stl',
            'FbowlL': 'models/physical-experiment/downsampled4blender/FbowlL.stl',
            }

def get_non_articulated_objects():
    '''
    Get a list of rigid objects.
    '''
    return ['Donut', 'Hook', 'Bowl', 'Ring', 'Bust', 'Hourglass', \
            'Bucket', 'Maze', '2Dkey', 'SplashBowl', 'Radish', 'FishHole', \
            'LeftHand', 'Shovel', 'LeftHandAndBowl', 'Rubic', 'Ear',\
            'Ftennis', 'Ftape', 'Fglue', 'Fbanana', 'FbowlS', 'FbowlM', 'FbowlL', \
            'Donut45', 'Donut60', 'Donut90', 'Donut120',]

def get_colors():
    '''
    Get a list of color codes for plotting.
    '''
    return ['#31a354', '#756bb1', '#2b8cbe', '#f4a63e', '#FF69B4', '#f03b20', ] # green, purple, blue, orange, pink, red, 

def flatten_nested_list(input):
    '''
    Input in the format of [[1], [2, 3], [4, 5, 6, 7]].
    Two nested layers at most.
    '''
    return [num for sublist in input for num in sublist]

def create_convex_vhacd(name_in, name_out, resolution=int(1e5)):
    '''
    Create concave obj files using pybullet vhacd function
    '''
    name_log = "log.txt"
    p.vhacd(name_in, name_out, name_log, resolution=resolution)

def generate_circle_points(n, rad, height, obj='Band'):
    '''
    Generate <n> points along a circle of radius <rad> (in the plane of z=<height>)
    '''
    points = []
    angles = list(np.linspace(0, 2*np.pi, n, endpoint=0))
    for i in range(n):
        x = rad * math.cos(angles[i])
        y = rad * math.sin(angles[i])
        if obj == 'Band':
            points.append([x,y,height])
        if obj == 'BandHorizon':
            points.append([x,y])
    points = flatten_nested_list(points)
    if obj == 'BandHorizon':
        points.append(height)
    
    return points

def ropeForwardKinematics(state, linkLen, baseDof_=6, ctrlPointDof_=2, TLastRow_=np.array([[0.,0.,0.,1.]])):
    '''
    Calculate rope control points forward dynamics given <state> (DoF: baseDof_+ctrlPointDof_*numCtrlPoint_),
    link length <linkLen>.
    '''
    numStateSpace_ = len(state)
    numCtrlPoint_ = int((numStateSpace_-baseDof_) / ctrlPointDof_)
    nextFPosInThisF_ = np.array([[0.], [linkLen], [0.], [1.]]) # 4*1

    # Data structure
    nodePositionsInWorld = [] # all nodes (numCtrlPoint+2): base + numCtrlPoint + end
    nodePosZsInWorld = []

    # Retrieve object's base transform
    basePosInWorld = np.array(state[0:3]).reshape((3))
    baseEulInWorld = state[3:6] # euler
    baseQuatInWorld = p.getQuaternionFromEuler(baseEulInWorld)
    r = R.from_quat(baseQuatInWorld) # BUG: quat to euler translation causes mistakes!
    mat = r.as_matrix() # (3,3)
    first3Rows = (mat, basePosInWorld.reshape((3,1)))
    first3Rows = np.hstack(first3Rows) # (3,4). first 3 rows of Transform matrix
    baseTInWorld = np.vstack((first3Rows, TLastRow_)) # (4,4)
    F0PosInWorld = baseTInWorld @ nextFPosInThisF_ # (4,1)
    F0PosInWorld = F0PosInWorld[:3].reshape((3))

    # Record
    nodePositionsInWorld.append(list(basePosInWorld))
    nodePositionsInWorld.append(list(F0PosInWorld))
    nodePosZsInWorld.append(float(basePosInWorld[2]))
    nodePosZsInWorld.append(float(F0PosInWorld[2]))

    # Iterate over control points
    Fi_1TInWorld = baseTInWorld
    for i in range(numCtrlPoint_):
        # Fi_1: F_{i-1} the (i-1)'th frame, F_{-1} is base frame
        # Fip1: F_{i+1} the (i+1)'th frame

        # Build local rotation matrix
        FiEulInFi_1 = state[6+2*i:6+2*(i+1)] # (x,z)
        FiEulInFi_1 = [FiEulInFi_1[0], 0.0, FiEulInFi_1[1]] # (x,0,z)
        # FiEulInFi_1.append(0.) # euler
        FiQuatInFi_1 = p.getQuaternionFromEuler(FiEulInFi_1)
        r = R.from_quat(FiQuatInFi_1)
        mat = r.as_matrix() # (3,3)

        # Build global transformation matrix
        first3Rows = (mat, np.array(nextFPosInThisF_[:3]))
        first3Rows = np.hstack(first3Rows) # (3,4). first 3 rows of Transform matrix
        FiTInFi_1 = np.vstack((first3Rows, TLastRow_)) # (4,4)
        FiTInWorld = Fi_1TInWorld @ FiTInFi_1 # (4,4)
        Fip1PosInWorld = FiTInWorld @ nextFPosInThisF_ # (4,1)
        Fip1PosInWorld = Fip1PosInWorld[:3].reshape((3)) # (3,)

        # Record
        nodePositionsInWorld.append(list(Fip1PosInWorld))
        nodePosZsInWorld.append(float(Fip1PosInWorld[2]))
        
        # Update last transformation matrix
        Fi_1TInWorld = FiTInWorld
    
    return nodePositionsInWorld, nodePosZsInWorld

def get_state_energy(state, object):
    '''
    Return the state energy of a rigid object.
    '''
    rigidObjs = get_non_articulated_objects()
    if object in rigidObjs: # rigid object caging
        return state[2] # assuming mg=1.0
    return None

def axiscreator(bodyId, linkId = -1):
    '''
    For visualizing the link axis in Bullet.
    '''
    x_axis = p.addUserDebugLine(lineFromXYZ = [0, 0, 0],
                                lineToXYZ = [1, 0, 0],
                                lineColorRGB = [1, 0, 0],
                                lineWidth = 0.1,
                                lifeTime = 0,
                                parentObjectUniqueId = bodyId,
                                parentLinkIndex = linkId )
    y_axis = p.addUserDebugLine(lineFromXYZ = [0, 0, 0],
                                lineToXYZ = [0, 1, 0],
                                lineColorRGB = [0, 1, 0],
                                lineWidth = 0.1,
                                lifeTime = 0,
                                parentObjectUniqueId = bodyId,
                                parentLinkIndex = linkId)
    z_axis = p.addUserDebugLine(lineFromXYZ = [0, 0, 0],
                                lineToXYZ = [0, 0, 1],
                                lineColorRGB = [0, 0, 1],
                                lineWidth = 0.1,
                                lifeTime = 0,
                                parentObjectUniqueId = bodyId,
                                parentLinkIndex = linkId)
    return [x_axis, y_axis, z_axis]

def visualizeWorkSpaceBound(bound: list) -> None:
    '''
    For visualizing the 3D boundary lines of the workspace.
    '''
    cornerPoints = list(product(bound[0], bound[1], bound[2]))
    edgeCornerPairs = [[1,3], [3,7], [7,5], [5,1], [0,2], [2,6], [6,4], [4,0], [0,1], [2,3], [6,7], [4,5]]
    for pair in edgeCornerPairs:
        p.addUserDebugLine(lineFromXYZ = cornerPoints[pair[0]],
            lineToXYZ = cornerPoints[pair[1]],
            lineColorRGB = [1, 0, 0],
            lineWidth = 0.1,
            lifeTime = 0,
            )

########################### For Soft Fixture ###########################
def band_collision_raycast(state, rayHitColor=[1,0,0], rayMissColor=[0,1,0], visRays=0, obj='Band'):
    '''
    Description:
        check if the lines connecting any two adjacent control points along a band penetrate obstacles.
        Pybullet Raycast method applied here.
    Input: 
        state: list of planner state vector, length is 3*numCtrlPoint
    '''
    # Construct start and end positions of rays
    if obj == 'Band':
        numCtrlPoint = int(len(state)/3)
        rayFromPositions = [state[3*i:3*i+3] for i in range(numCtrlPoint)]
    if obj == 'BandHorizon':
        numCtrlPoint = int((len(state)-1)/2)
        rayFromPositions = [state[2*i:2*i+2] for i in range(numCtrlPoint)]
        rayFromPositions = [r+[state[-1]] for r in rayFromPositions]
    rayToPositions = rayFromPositions[1:]
    rayToPositions.append(rayFromPositions[0])
    results = p.rayTestBatch(rayFromPositions, rayToPositions)

    # Check if any ray hits obstacles
    hitObjectUids = [results[i][0] for i in range(len(results))]
    idMask = [hitObjectUids[i]>=0 for i in range(len(hitObjectUids))]
    collisionExists = (idMask.count(True) > 0)

    # Visualize the band after running the planner
    if visRays:
        for i,idNonNegative in enumerate(idMask):
            if (not idNonNegative): # collision free
                p.addUserDebugLine(rayFromPositions[i], rayToPositions[i], rayMissColor, lineWidth=5, lifeTime=.1)
            else: # in collision
                p.addUserDebugLine(rayFromPositions[i], rayToPositions[i], rayHitColor, lineWidth=5, lifeTime=.1)

    return collisionExists

def mask_band_collision_raycast(state, bandFixedV0, bandFixedV1,
                                rayHitColor=[1,0,0], rayMissColor=[0,1,0], visRays=0, lifeTime=.1):
    '''
    Description:
        check if the lines connecting any two adjacent control points along a mask loop penetrate obstacles.
        Pybullet Raycast method applied here.
    Input: 
        state: list of planner state vector, length is 3*numCtrlPoint
        bandFixedV0, bandFixedV1: two intersection points between a mask loop and a mask filter.
    Return:
        collisionExists: True if there exists collisions
    '''
    # Construct start and end positions of rays
    numCtrlPoint = int(len(state)/3)
    rayFromPositions = [state[3*i:3*i+3] for i in range(numCtrlPoint)]
    rayToPositions = copy.copy(rayFromPositions)
    rayFromPositions.insert(0, bandFixedV0)
    rayToPositions.append(bandFixedV1)
    results = p.rayTestBatch(rayFromPositions, rayToPositions)

    # Check if any ray hits obstacles
    hitObjectUids = [results[i][0] for i in range(len(results))]
    idMask = [hitObjectUids[i]>=0 for i in range(len(hitObjectUids))]
    collisionExists = (idMask.count(True) > 0)

    # Visualize the band after running the planner
    if visRays:
        for i,idNonNegative in enumerate(idMask):
            if (not idNonNegative): # collision free
                p.addUserDebugLine(rayFromPositions[i], rayToPositions[i], rayMissColor, lineWidth=5, lifeTime=lifeTime)
            else: # in collision
                p.addUserDebugLine(rayFromPositions[i], rayToPositions[i], rayHitColor, lineWidth=5, lifeTime=lifeTime)

    return collisionExists

def plane_equation(point, a, b, c, d):
    '''
    Define the equation of the plane.
    '''
    x, y, z = point
    return (a * x) + (b * y) + (c * z) + d

def rope_collision_raycast(state, linkLen, rayHitColor=[1,0,0], rayMissColor=[0,1,0], visRays=0):
    '''
    Description:
        check if the lines connecting any two adjacent control points along a rope penetrate obstacles.
        Pybullet Raycast method applied here.
    Input: 
        state: list of planner state vector, length is 6+2*numCtrlPoint
        obstacles: list of obstacle IDs
    '''
    # Run rope forward kinematics and retrive nodes
    nodePositionsInWorld, _ = ropeForwardKinematics(state, linkLen) # no. of zs - numCtrlPoint_+2

    # Construct start and end positions of rays
    rayFromPositions = nodePositionsInWorld[:-1]
    rayToPositions = nodePositionsInWorld[1:]
    results = p.rayTestBatch(rayFromPositions, rayToPositions)

    # Check if any ray hits obstacles
    hitObjectUids = [results[i][0] for i in range(len(results))]
    idMask = [hitObjectUids[i]>=0 for i in range(len(hitObjectUids))]
    collisionExists = (idMask.count(True) > 0)

    # Visualize the rope after running the planner
    if visRays:
        for i,idNonNegative in enumerate(idMask):
            if (not idNonNegative): # collision free
                p.addUserDebugLine(rayFromPositions[i], rayToPositions[i], rayMissColor, lineWidth=5, lifeTime=.1)
            else: # in collision
                p.addUserDebugLine(rayFromPositions[i], rayToPositions[i], rayHitColor, lineWidth=5, lifeTime=.1)

    return collisionExists

def get_chain_node_pos(state, linkLen):
    '''
    Calculate the control points positions along a rope loop.
    '''
    notALoop = False

    # Run rope forward kinematics and retrive nodes
    nodePositionsInWorld, _ = ropeForwardKinematics(state, linkLen) # no. of zs - numCtrlPoint_+2

    # For loop chain, last node roughly coincides with the first
    node0 = np.asarray(nodePositionsInWorld[0])
    node_2 = np.asarray(nodePositionsInWorld[-1]) # node[-2]
    normal_vector = node_2-node0
    if LA.norm(normal_vector) > 2*linkLen: # not possible forming a loop
        notALoop = True
        return notALoop, None
    
    # Find the position of the last node (node[-1]) in the chain
    mid = (node0 + node_2) / 2
    radius = np.sqrt(linkLen**2 - LA.norm(((node0-node_2)/2)**2))
    node_1 = points_on_circle(radius, mid, normal_vector, state[-1]) # list[3]

    return notALoop, nodePositionsInWorld + [node_1]

def chain_collision_raycast(state, linkLen, rayHitColor=[1,0,0], rayMissColor=[0,1,0], visRays=0):
    '''
    Description:
        check if the lines connecting any two adjacent control points along a chain loop penetrate obstacles.
        Pybullet Raycast method applied here.
    Input: 
        state: list of planner state vector, length is 6+2*numCtrlPoint+1
    '''
    # Get every node's position by the help of the rope model
    notALoop, rayFromPositions = get_chain_node_pos(state, linkLen)
    if notALoop:
        return True
    nodePositionsInWorld = rayFromPositions[:-1]
    node_1 = rayFromPositions[-1]

    # Make sure the base node is the lowest
    nodesZs = [rayFromPositions[i][2] for i in range(len(rayFromPositions))]
    baseNodeHeightBool = [nodesZs[0] <= z for z in nodesZs[1:]]
    if baseNodeHeightBool.count(False) > 0:
        baseNodeNotLowest = True
        return baseNodeNotLowest

    # Construct start and end positions of rays
    rayToPositions = nodePositionsInWorld[1:] + [node_1, nodePositionsInWorld[0]]
    results = p.rayTestBatch(rayFromPositions, rayToPositions)

    # Check if any ray hits obstacles
    hitObjectUids = [results[i][0] for i in range(len(results))]
    idMask = [hitObjectUids[i]>=0 for i in range(len(hitObjectUids))]
    collisionExists = (idMask.count(True) > 0)

    # Visualize the rope after running the planner
    if visRays:
        for i,idNonNegative in enumerate(idMask):
            if (not idNonNegative): # collision free
                p.addUserDebugLine(rayFromPositions[i], rayToPositions[i], rayMissColor, lineWidth=5, lifeTime=.1)
            else: # in collision
                p.addUserDebugLine(rayFromPositions[i], rayToPositions[i], rayHitColor, lineWidth=5, lifeTime=.1)
    return collisionExists

def getGravityEnergy(state, args, path):
    '''
    Get gravitational potential energy of a rope loop.
    '''
    comDof = 6
    numStateSpace = len(state)
    numJoints = numStateSpace - comDof
    numLinks = numJoints + 1
    g = 9.81
    masses = [.1] * numLinks
    chain = kp.build_chain_from_urdf(open(path[args.object]).read())
    
    # Extract positions of links
    jointAngles = state[comDof:] # rad
    linkPosesInBase = chain.forward_kinematics(jointAngles) # dictionary

    # Get object's base transform
    basePositionInWorld = state[0:3]
    baseOrientationInWorld = state[3:comDof] # euler
    quat = p.getQuaternionFromEuler(baseOrientationInWorld)
    r = R.from_quat(quat) # BUG: quat to euler translation causes mistakes!
    mat = r.as_matrix() # 3*3
    thirdRow = (mat[2,:].reshape((1,3)), np.array(basePositionInWorld[2]).reshape((1,1)))
    baseTInWorld = np.hstack(thirdRow) # 1*4. 3rd row of Transform matrix

    # Get link heights in World frame
    linkPosesInBase = list(linkPosesInBase.values()) # list of kinpy.Transforms
    linkPositionsInBase = [np.array(np.concatenate((i.pos,np.array([1.])))).reshape((4,1)) for i in linkPosesInBase]
    linkZsInWorld = [float(baseTInWorld @ j) for j in linkPositionsInBase] # list of links' heights

    # Get links' gravitational potential energy
    linkEnergies = [linkZsInWorld[i] * masses[i] for i in range(numLinks)]
    energyGravity = g * sum(linkEnergies) # sigma(g * m_i * z_i)
    
    return energyGravity


'''Given a point on a plane, find a goal on the circle centered at the point with a given radius.
'''
def points_on_circle(radius, center, normal_vector, theta):
    # Normalize the normal vector
    normal_vector = np.array(normal_vector).astype(float)
    normal_vector /= np.linalg.norm(normal_vector)

    # Generate an orthogonal vector to the normal vector
    v1 = np.cross(normal_vector, [1, 0, 0])
    if np.linalg.norm(v1) == 0:
        v1 = np.cross(normal_vector, [0, 1, 0])

    # Calculate another orthogonal vector to the normal vector and v1
    v2 = np.cross(normal_vector, v1)

    # Normalize v1 and v2
    v1 /= np.linalg.norm(v1)
    v2 /= np.linalg.norm(v2)

    # Generate points on the circle
    x = center[0] + radius * (v1[0] * np.cos(theta) + v2[0] * np.sin(theta))
    y = center[1] + radius * (v1[1] * np.cos(theta) + v2[1] * np.sin(theta))
    z = center[2] + radius * (v1[2] * np.cos(theta) + v2[2] * np.sin(theta))

    return [x,y,z]


################################# Inheriated from Parent Repo ######################################
def pairwise_link_collision(body1, link1, body2, link2=BASE_LINK, max_distance=MAX_DISTANCE):
    '''
    Check if two links in an object are in collision in Pybullet.
    '''
    return len(p.getClosestPoints(bodyA=body1, bodyB=body2, distance=max_distance,
                                  linkIndexA=link1, linkIndexB=link2)) != 0  # getContactPoints

def pairwise_collision(body1, body2, **kwargs):
    '''
    Check if two bodies are in collision in Pybullet.
    '''
    if isinstance(body1, tuple) or isinstance(body2, tuple):
        body1, links1 = expand_links(body1)
        body2, links2 = expand_links(body2)
        return any_link_pair_collision(body1, links1, body2, links2, **kwargs)
    return body_collision(body1, body2, **kwargs)

def expand_links(body):
    body, links = body if isinstance(body, tuple) else (body, None)
    if links is None:
        links = get_all_links(body)
    return body, links

def any_link_pair_collision(body1, links1, body2, links2=None, **kwargs):
    if links1 is None:
        links1 = get_all_links(body1)
    if links2 is None:
        links2 = get_all_links(body2)
    for link1, link2 in product(links1, links2):
        if (body1 == body2) and (link1 == link2):
            continue
        if pairwise_link_collision(body1, link1, body2, link2, **kwargs):
            return True
    return False

def get_self_link_pairs(body, joints, disabled_collisions=set(), only_moving=True):
    moving_links = get_moving_links(body, joints)
    fixed_links = list(set(get_joints(body)) - set(moving_links))
    check_link_pairs = list(product(moving_links, fixed_links))
    if only_moving:
        check_link_pairs.extend(get_moving_pairs(body, joints))
    else:
        check_link_pairs.extend(combinations(moving_links, 2))
    check_link_pairs = list(
        filter(lambda pair: not are_links_adjacent(body, *pair), check_link_pairs))
    check_link_pairs = list(filter(lambda pair: (pair not in disabled_collisions) and
                                                (pair[::-1] not in disabled_collisions), check_link_pairs))
    return check_link_pairs

def get_moving_links(body, joints):
    moving_links = set()
    for joint in joints:
        link = child_link_from_joint(joint)
        if link not in moving_links:
            moving_links.update(get_link_subtree(body, link))
    return list(moving_links)

def get_moving_pairs(body, moving_joints):
    """
    Check all fixed and moving pairs
    Do not check all fixed and fixed pairs
    Check all moving pairs with a common
    """
    moving_links = get_moving_links(body, moving_joints)
    for link1, link2 in combinations(moving_links, 2):
        ancestors1 = set(get_joint_ancestors(body, link1)) & set(moving_joints)
        ancestors2 = set(get_joint_ancestors(body, link2)) & set(moving_joints)
        if ancestors1 != ancestors2:
            yield link1, link2

def body_collision(body1, body2, max_distance=MAX_DISTANCE):
    '''
    Check if two bodies are in collision in Pybullet.
    '''
    return len(p.getClosestPoints(bodyA=body1, bodyB=body2, distance=max_distance)) != 0

JointInfo = namedtuple('JointInfo', ['jointIndex', 'jointName', 'jointType',
                                     'qIndex', 'uIndex', 'flags',
                                     'jointDamping', 'jointFriction', 'jointLowerLimit', 'jointUpperLimit',
                                     'jointMaxForce', 'jointMaxVelocity', 'linkName', 'jointAxis',
                                     'parentFramePos', 'parentFrameOrn', 'parentIndex'])

def get_joint_info(body, joint):
    return JointInfo(*p.getJointInfo(body, joint))

def child_link_from_joint(joint):
    return joint  # link

def get_num_joints(body):
    return p.getNumJoints(body)

def get_joints(body):
    return list(range(get_num_joints(body)))

get_links = get_joints

def get_all_links(body):
    return [BASE_LINK] + list(get_links(body))

def get_link_parent(body, link):
    if link == BASE_LINK:
        return None
    return get_joint_info(body, link).parentIndex

def get_all_link_parents(body):
    return {link: get_link_parent(body, link) for link in get_links(body)}

def get_all_link_children(body):
    children = {}
    for child, parent in get_all_link_parents(body).items():
        if parent not in children:
            children[parent] = []
        children[parent].append(child)
    return children

def get_link_children(body, link):
    children = get_all_link_children(body)
    return children.get(link, [])

def get_link_ancestors(body, link):
    # Returns in order of depth
    # Does not include link
    parent = get_link_parent(body, link)
    if parent is None:
        return []
    return get_link_ancestors(body, parent) + [parent]

def get_joint_ancestors(body, joint):
    link = child_link_from_joint(joint)
    return get_link_ancestors(body, link) + [link]

def get_link_descendants(body, link, test=lambda l: True):
    descendants = []
    for child in get_link_children(body, link):
        if test(child):
            descendants.append(child)
            descendants.extend(get_link_descendants(body, child, test=test))
    return descendants

def get_link_subtree(body, link, **kwargs):
    return [link] + get_link_descendants(body, link, **kwargs)

def are_links_adjacent(body, link1, link2):
    return (get_link_parent(body, link1) == link2) or \
           (get_link_parent(body, link2) == link1)


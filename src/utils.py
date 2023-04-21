'''
Adapted from
https://github.com/StanfordVL/iGibson/blob/master/igibson/external/pybullet_tools/utils.py
'''

from __future__ import print_function
import pybullet as p
from collections import defaultdict, deque, namedtuple
from itertools import product, combinations, count
import argparse
import math
import numpy as np
import time

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
    parser.add_argument('-c', '--scenario', default='BustTrapsBand', \
        choices=['FishFallsInBowl', 'HookTrapsRing', 'GripperClenchesStarfish', 'BustTrapsBand'], \
        help='(Optional) Specify the scenario of demo, defaults to FishFallsInBowl if not given.')

    parser.add_argument('-s', '--search', default='EnergyMinimizeSearch', \
        choices=['BoundShrinkSearch', 'EnergyMinimizeSearch'], \
        help='(Optional) Specify the sampling-based search method to use, defaults to BoundShrinkSearch if not given.')
    
    parser.add_argument('-p', '--planner', default='BITstar', \
        choices=['BFMTstar', 'BITstar', 'FMTstar', 'FMT', 'InformedRRTstar', 'PRMstar', 'RRTstar', \
        'SORRTstar', 'RRT'], \
        help='(Optional) Specify the optimal planner to use, defaults to RRTstar if not given.')
    
    parser.add_argument('-o', '--objective', default='ElasticPotential', \
        choices=['ElasticPotential', 'GravityPotential', 'GravityAndElasticPotential', \
        'PotentialAndPathLength'], \
        help='(Optional) Specify the optimization objective, defaults to PathLength if not given.')

    parser.add_argument('-j', '--object', default='Rope', \
        choices=['Fish', 'FishWithRing', 'Starfish', 'Ring', 'Band', 'Rope', 'Humanoid', 'Donut', 'Hook', '3fGripper', 'PlanarRobot', 'PandaArm'], \
        help='(Optional) Specify the object to cage.')

    parser.add_argument('-l', '--obstacle', default='Box', \
        choices=['Box', 'Hook', '3fGripper', 'Bowl', 'Bust', 'Hourglass', 'Hole'], \
        help='(Optional) Specify the obstacle that cages the object.')
    
    parser.add_argument('-t', '--runtime', type=float, default=150, help=\
        '(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0. (In the current settings, 240 s not better a lot than 120 s)')
    
    parser.add_argument('-v', '--visualization', type=bool, default=1, help=\
        '(Optional) Specify whether to visualize the pybullet GUI. Defaults to False and must be False or True.')
    
    # parser.add_argument('-f', '--file', default=None, \
    #     help='(Optional) Specify anoutput path for the found solution path.')
    
    # parser.add_argument('-i', '--info', type=int, default=0, choices=[0, 1, 2], \
    #     help='(Optional) Set the OMPL log level. 0 for WARN, 1 for INFO, 2 for DEBUG.' \
    #     ' Defaults to WARN.')

    # Parse the arguments
    args = parser.parse_args()

    return args, parser

def path_collector():
    return {
            'Fish': 'models/fish/articulate_fish.xacro', 
            'FishWithRing': 'models/fish/fishWithRing.xacro', 
            'Starfish': 'models/starfish/starfish2.urdf', 
            'Ring': 'models/fish/ring2.urdf', 
            'Donut': 'models/donut/donut.urdf',
            '3fGripper': 'models/robotiq_3f_gripper_visualization/cfg/robotiq-3f-gripper_articulated.urdf',
            'PandaArm': 'models/franka_description/robots/panda_arm.urdf',
            'PlanarRobot': 'models/planar_robot_4_link.xacro',
            'Humanoid': 'models/humanoid.urdf',
            'Bowl': 'models/bowl/small_bowl.stl', 
            'Hook': 'models/triple_hook/triple_hook_vhacd.obj', 
            'Bust': 'models/bust/female_bust.obj',
            'Hourglass': 'models/hourglass/hourglass.obj'
            }

def texture_path_list():
    return {
            'Hook': 'models/triple_hook/metal_texture.png', 
            'Fish': 'models/triple_hook/metal_texture.png', 
            'Bowl': 'models/triple_hook/metal_texture.png', 
            }

def get_non_articulated_objects():
    return ['Donut', 'Hook', 'Bowl', 'Ring', 'Starfish', 'Bust', 'Hourglass']

def get_colors():
    return ['#31a354', '#756bb1', '#2b8cbe', '#f03b20'] # green, purple, blue, red

def flatten_nested_list(input):
    '''Input in the format of [[1], [2, 3], [4, 5, 6, 7]].
        Two nested layers at most.
    '''
    return [num for sublist in input for num in sublist]

def create_convex_vhacd(name_in, name_out, resolution=int(1e5)):
    '''Create concave obj files using pybullet vhacd function
    '''
    name_log = "log.txt"
    p.vhacd(name_in, name_out, name_log, resolution=resolution)

def generate_circle_points(n, rad, z):
    points = []
    angles = list(np.linspace(0, 2*np.pi, n, endpoint=0))
    for i in range(n):
        x = rad * math.cos(angles[i])
        y = rad * math.sin(angles[i])
        points.append([x,y,z])
    points = flatten_nested_list(points)
    return points

def ropeForwardKinematics(state, linkLen, baseDof_=6, ctrlPointDof_=2, TLastRow_=np.array([[0.,0.,0.,1.]])):
    numStateSpace_ = len(state)
    numCtrlPoint_ = int((numStateSpace_-baseDof_) / ctrlPointDof_)
    nextFPosInThisF_ = np.array([[0.], [0.], [linkLen], [1.]]) # 4*1

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
        FiEulInFi_1 = state[6+2*i:6+2*(i+1)]
        FiEulInFi_1.append(0.) # euler
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

#####################################

def pairwise_link_collision(body1, link1, body2, link2=BASE_LINK, max_distance=MAX_DISTANCE):  # 10000
    return len(p.getClosestPoints(bodyA=body1, bodyB=body2, distance=max_distance,
                                  linkIndexA=link1, linkIndexB=link2)) != 0  # getContactPoints

def pairwise_collision(body1, body2, **kwargs):
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
    # TODO: this likely isn't needed anymore
    if links1 is None:
        links1 = get_all_links(body1)
    if links2 is None:
        links2 = get_all_links(body2)
    for link1, link2 in product(links1, links2):
        if (body1 == body2) and (link1 == link2):
            continue
        if pairwise_link_collision(body1, link1, body2, link2, **kwargs):
            # print('body {} link {} body {} link {}'.format(body1, link1, body2, link2))
            return True
    return False

def body_collision(body1, body2, max_distance=MAX_DISTANCE):  # 10000
    # p.stepSimulation()
    return len(p.getClosestPoints(bodyA=body1, bodyB=body2, distance=max_distance)) != 0  # getContactPoints`
    # return len(p.getContactPoints(bodyA=body1, bodyB=body2,)) > 20  # getContactPoints`

def band_collision_raycast(state, rayHitColor=[1,0,0], rayMissColor=[0,1,0], visRays=0):
    '''
    Description:
        check if the lines connecting any two adjacent control points along a band penetrate obstacles.
        Pybullet Raycast method applied here.
    Input: 
        state: list of planner state vector, length is 3*numCtrlPoint
    '''
    # Construct start and end positions of rays
    numCtrlPoint = int(len(state)/3)
    rayFromPositions = [state[3*i:3*i+3] for i in range(numCtrlPoint)]
    rayToPositions = rayFromPositions[1:]
    rayToPositions.append(rayFromPositions[0])
    results = p.rayTestBatch(rayFromPositions, rayToPositions)
    # TODO: double-way raycast

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
    #     p.removeAllUserDebugItems()

    return collisionExists

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
    # is_collision = False
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

    # for i in range(len(results)):
    #     hitObjectUid = results[i][0]
    #     if (hitObjectUid < 0): # collision free
    #         p.addUserDebugLine(rayFromPositions[i], rayToPositions[i], rayMissColor, lineWidth=5, lifeTime=.07) if visRays else None
    #     else: # collision
    #         # hitPosition = results[i][3]
    #         p.addUserDebugLine(rayFromPositions[i], rayToPositions[i], rayHitColor, lineWidth=5, lifeTime=.07) if visRays else None
    #         is_collision = True

    return collisionExists

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


#####################################

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

import numpy as np
import kinpy as kp
from scipy.spatial.transform import Rotation as R
def getGravityEnergy(state, args, path):
    comDof = 6
    numStateSpace = len(state)
    numJoints = numStateSpace - comDof
    numLinks = numJoints + 1
    g = 9.81
    masses = [.1] * numLinks
    chain = kp.build_chain_from_urdf(open(path[args.object]).read())
    
    # extract positions of links
    # jnames = chain.get_joint_parameter_names()
    jointAngles = state[comDof:] # rad
    linkPosesInBase = chain.forward_kinematics(jointAngles) # dictionary
    # print('\njointAngles:', jointAngles)

    # get object's base transform
    basePositionInWorld = state[0:3]
    baseOrientationInWorld = state[3:comDof] # euler
    quat = p.getQuaternionFromEuler(baseOrientationInWorld)
    # r = R.from_euler('zyx', baseOrientationInWorld, degrees=False)
    r = R.from_quat(quat) # BUG: quat to euler translation causes mistakes!
    mat = r.as_matrix() # 3*3
    thirdRow = (mat[2,:].reshape((1,3)), np.array(basePositionInWorld[2]).reshape((1,1)))
    baseTInWorld = np.hstack(thirdRow) # 1*4. 3rd row of Transform matrix
    # baseTransformInWorld = np.vstack(baseTInWorld, np.array([.0, .0, .0, 1.0])) # 4*4

    # get link heights in World frame
    linkPosesInBase = list(linkPosesInBase.values()) # list of kinpy.Transforms
    linkPositionsInBase = [np.array(np.concatenate((i.pos,np.array([1.])))).reshape((4,1)) for i in linkPosesInBase]
    linkZsInWorld = [float(baseTInWorld @ j) for j in linkPositionsInBase] # list of links' heights

    # get links' gravitational potential energy
    linkEnergies = [linkZsInWorld[i] * masses[i] for i in range(numLinks)]
    energyGravity = g * sum(linkEnergies) # sigma(g * m_i * z_i)
    
    return energyGravity
      
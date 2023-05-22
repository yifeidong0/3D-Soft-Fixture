import pybullet as p
import copy
import math
from scipy.spatial.transform import Rotation as R
import time
from utils import *

class ObjectBase():
    '''
    To use with Pb_OMPL. You need to construct a instance of this class and pass to PbOMPL.

    Note:
    This parent class by default assumes that all joints are acutated and should be planned. If this is not your desired
    behaviour, please write your own inheritated class that overrides respective functionalities.
    '''
    def __init__(self, id) -> None:
        # Public attributes
        self.id = id

        # prune fixed joints
        all_joint_num = p.getNumJoints(id)
        all_joint_idx = list(range(all_joint_num))
        joint_idx = [j for j in all_joint_idx if self._is_not_fixed(j)]
        self.num_dim = len(joint_idx)
        self.joint_idx = joint_idx
        print('joint_idx: ', self.joint_idx)
        self.joint_bounds = []

        self.reset()

    def _is_not_fixed(self, joint_idx):
        joint_info = p.getJointInfo(self.id, joint_idx)
        return joint_info[2] != p.JOINT_FIXED

    def get_joint_bounds(self):
        '''
        Get joint bounds.
        By default, read from pybullet
        '''
        for i, joint_id in enumerate(self.joint_idx):
            joint_info = p.getJointInfo(self.id, joint_id)
            low = joint_info[8] # low bounds
            high = joint_info[9] # high bounds
            if low < high:
                self.joint_bounds.append([low, high])
        print("Joint bounds: {}".format(self.joint_bounds))
        return self.joint_bounds

    def get_cur_state(self):
        return copy.deepcopy(self.state)

    def set_state(self, state):
        '''
        Set robot state.
        To faciliate collision checking
        Args:
            state: list[Float], joint values of robot
        '''
        self._set_joint_positions(self.joint_idx, state)
        self.state = state

    def reset(self):
        '''
        Reset robot state
        Args:
            state: list[Float], joint values of robot
        '''
        state = [0] * self.num_dim
        self._set_joint_positions(self.joint_idx, state)
        self.state = state

    def _set_joint_positions(self, joints, positions):
        for joint, value in zip(joints, positions):
            p.resetJointState(self.id, joint, value, targetVelocity=0)


class ObjectFromUrdf(ObjectBase):
    '''
    Object loaded from URDF files. States, joint bounds are defined here.
    '''
    def __init__(self, id) -> None:
        self.id = id
        self.comDof = 3 + 3 # SE3
        self.articulate_num = p.getNumJoints(id)
        self.num_dim = self.comDof + self.articulate_num
        self.joint_idx = []
        # self.reset()

        # self.set_search_bounds() # TODO: why joint bounds appear when running scenario multiple times

    def set_search_bounds(self, vis=1, basePosBounds=[[-2.5, 2.5], [-2.5, 2.5], [0, 5]]):
        self.joint_bounds = basePosBounds # CoM pos
        for i in range(3): # CoM rot
            self.joint_bounds.append([math.radians(-180), math.radians(180)]) # r, p, y
        
        for i in range(self.articulate_num): # articulated joints
            info = p.getJointInfo(self.id, i)
            jointType = info[2]

            # record non-fixed joints' index and bounds
            if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
                bounds = p.getJointInfo(self.id, i)[8:10] # joint limits
                if bounds[0] >= bounds[1]:
                    continue
                self.joint_idx.append(i)
                self.joint_bounds.append(bounds) # joint_0-3
        print('@@@@@@@joint_bounds', self.joint_bounds)
        
        # Visualize Workspace boundaries
        if vis:
            visualizeWorkSpaceBound(basePosBounds)

    def set_bisec_thres(self, zmax):
        self.joint_bounds[2][1] = zmax
        
    def get_joint_bounds(self):
        return self.joint_bounds

    def get_cur_state(self):
        return self.state

    def set_state(self, state):
        pos = state[0:3]
        eulerRot = state[3:6]

        # r = R.from_euler('zyx', eulerRot, degrees=False)
        # quat = r.as_quat()
        quat = p.getQuaternionFromEuler(eulerRot)
        p.resetBasePositionAndOrientation(self.id, pos, quat)
        self._set_joint_positions(self.joint_idx, state[6:])

        self.state = state

    def reset(self):
        pos = [0,0,0]
        quat = [0,0,0,1]
        p.resetBasePositionAndOrientation(self.id, pos, quat)
        self._set_joint_positions(self.joint_idx, [0]*self.articulate_num)
        self.state = [0] * self.num_dim

    def _set_joint_positions(self, joints, positions):
        for joint, value in zip(joints, positions):
            p.resetJointState(self.id, joint, value, targetVelocity=0)


class SnapLock2D(ObjectFromUrdf):
    '''
    An elastic band composed of several control points.
    '''
    def __init__(self, id, basePosBounds) -> None:
        self.id = id # a list
        self.comDof = 2 + 1 # SE2
        self.articulate_num = p.getNumJoints(id)
        self.num_dim = self.comDof + self.articulate_num
        self.joint_idx = []
        self.zeroQuaternion = p.getQuaternionFromEuler([0,0,0])

        self.set_search_bounds(basePosBounds=basePosBounds)

    def set_search_bounds(self, vis=1, basePosBounds=[[-5, 5], [-5, 5]]):
        self.joint_bounds = basePosBounds # origin position
        self.joint_bounds.append([math.radians(-180), math.radians(180)]) # rotation around origin
        
        for i in range(self.articulate_num): # articulated joints
            info = p.getJointInfo(self.id, i)
            jointType = info[2]

            # record non-fixed joints' index and bounds
            if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
                bounds = p.getJointInfo(self.id, i)[8:10] # joint limits
                if bounds[0] >= bounds[1]:
                    continue
                self.joint_idx.append(i)
                self.joint_bounds.append(bounds) # joint_0-3
        
        # Visualize Workspace boundaries
        if vis:
            visualizeWorkSpaceBound(basePosBounds[:2]+[[-0.02,0.02],])

    def set_state(self, state):
        # Get position and quaternion of base origin
        pos = state[:2] + [0.0,]
        eulerRot = [0.0, 0.0] + [state[2],]
        quat = p.getQuaternionFromEuler(eulerRot)

        # Reset base and joint state
        p.resetBasePositionAndOrientation(self.id, pos, quat)
        self._set_joint_positions(self.joint_idx, state[3:])
        self.state = state


class objectElasticBand(ObjectFromUrdf):
    '''
    An elastic band composed of several control points.
    '''
    def __init__(self, id, numCtrlPoint) -> None:
        self.id = id # a list
        self.numCtrlPoint = numCtrlPoint
        self.comDof = 3
        self.joint_idx = []
        self.num_dim = self.comDof * self.numCtrlPoint
        self.zeroQuaternion = p.getQuaternionFromEuler([0,0,0])

        self.set_search_bounds()

    def set_search_bounds(self, vis=1, basePosBounds=[[-1.5, 1.5], [-1.5, 1.5], [0, 2.5]]):
        self.joint_bounds = self.numCtrlPoint * basePosBounds
        
        # Visualize Workspace boundaries
        if vis:
            visualizeWorkSpaceBound(basePosBounds)

    def set_state(self, state):
        self.state = state
        # for i in range(len(self.id)):
        #     p.resetBasePositionAndOrientation(self.id[i], self.state[3*i:3*i+3], self.zeroQuaternion)


class objectElasticJelly(ObjectFromUrdf):
    '''
    An elastic jelly composed of several (4) control points as a volumetric deformable primitive.
    '''
    def __init__(self, id, numCtrlPoint) -> None:
        self.id = id # a list
        self.numCtrlPoint = numCtrlPoint
        self.comDof = 3
        self.joint_idx = []
        self.num_dim = self.comDof * self.numCtrlPoint
        self.zeroQuaternion = p.getQuaternionFromEuler([0,0,0])

        self.set_search_bounds()

    def set_search_bounds(self, vis=1, basePosBounds=[[-1.5, 1.5], [-1.5, 1.5], [0, 2.5]]):
        self.joint_bounds = self.numCtrlPoint * basePosBounds

        # Visualize Workspace boundaries
        if vis:
            visualizeWorkSpaceBound(basePosBounds)

    def set_state(self, state):
        self.state = state


class objectRope(ObjectFromUrdf):
    '''
    A rope composed of base (6 DoF) and several control points (3n DoF).
    '''
    def __init__(self, id, numCtrlPoint, linkLen) -> None:
        self.id = id # a list of spheres' IDs
        self.numCtrlPoint = numCtrlPoint
        self.linkLen = linkLen
        self.baseDof = 6
        self.ctrlPointDof = 2
        self.joint_idx = []
        self.num_dim = self.baseDof + self.ctrlPointDof*self.numCtrlPoint # 6+2n
        self.nodesPositions = None
        self.zeroQuaternion = p.getQuaternionFromEuler([0,0,0])

        self.set_search_bounds()

    def set_search_bounds(self, vis=1, basePosBounds=[[-1.5, 1.5], [-1.5, 1.5], [0, 2.5]]):
        self.joint_bounds = basePosBounds # base pos
        for i in range(self.num_dim-3): # 3+2n rot
            self.joint_bounds.append([math.radians(-180), math.radians(180)]) # r, p, y

        # Visualize Workspace boundaries
        if vis:
            visualizeWorkSpaceBound(basePosBounds)
    
    def set_state(self, state):
        # NODE COLLISION TEST
        self.nodesPositions, _ = ropeForwardKinematics(state, self.linkLen)
        for i in range(len(self.id)):
            p.resetBasePositionAndOrientation(self.id[i], self.nodesPositions[i], self.zeroQuaternion)

        self.state = state


class objectLoopChain(ObjectFromUrdf):
    '''
    A rope composed of base (6 DoF) and several control points (3n DoF).
    '''
    def __init__(self, id, numCtrlPoint, linkLen) -> None:
        self.id = id # a list of spheres' IDs
        self.numCtrlPoint = numCtrlPoint
        self.numLink = numCtrlPoint + 3
        self.linkLen = linkLen
        self.baseDof = 6
        self.ctrlPointDof = 2
        self.joint_idx = []
        self.num_dim = self.baseDof + self.ctrlPointDof*self.numCtrlPoint + 1 # 6+2n
        self.nodesPositions = None
        self.zeroQuaternion = p.getQuaternionFromEuler([0,0,0])

        # self.set_search_bounds()

    def set_search_bounds(self, vis=1, basePosBounds=[[-1.5, 1.5], [-1.5, 1.5], [0, 2.5]]):
        self.joint_bounds = basePosBounds # base pos
        for i in range(3): # 3 base rotation euler
            self.joint_bounds.append([math.radians(-180), math.radians(180)])
        # lbound, ubound = 360 / (self.numLink+1), 360 / (self.numLink-1) 
        bound = 360/(self.numCtrlPoint+3)+15
        for i in range(self.ctrlPointDof*self.numCtrlPoint): # 2n joint rot
            self.joint_bounds.append([math.radians(-bound), math.radians(bound)])
        self.joint_bounds.append([math.radians(-180), math.radians(180)]) # last element that guarantees loop closure

        # Visualize Workspace boundaries
        if vis:
            visualizeWorkSpaceBound(basePosBounds)
    
    def set_state(self, state):
        self.state = state


class obstascle3fGripper(ObjectFromUrdf):
    '''
    A Robotiq 3-finger gripper composed of 3 fingers and 4 DoF on each.
    '''
    def __init__(self, id) -> None:
        self.id = id
        self.comDof = 3 + 3 # SE3
        self.articulate_num = p.getNumJoints(id)
        self.num_dim = self.comDof + self.articulate_num
        self.joint_idx = []
        self.joint_bounds = []

        self.set_joint_bounds()

    def set_joint_bounds(self):
        for i in range(self.articulate_num): # articulated joints
            info = p.getJointInfo(self.id, i)
            jointType = info[2]
            if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
                bounds = p.getJointInfo(self.id, i)[8:10] # joint limits
                if bounds[0] >= bounds[1]:
                    continue
                self.joint_idx.append(i)
                self.joint_bounds.append(bounds) # joint_0-3
        # print('@@@@@@@@@self.joint_bounds', self.joint_bounds)
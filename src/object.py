import pybullet as p
import copy
import math
from scipy.spatial.transform import Rotation as R

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


class ObjectToCage(ObjectBase):
    def __init__(self, id) -> None:
        self.id = id
        self.comDof = 3 + 3 # SE3
        self.articulate_num = p.getNumJoints(id)
        self.num_dim = self.comDof + self.articulate_num
        self.joint_idx = []
        # self.reset()

        self.set_search_bounds()

    def set_search_bounds(self, basePosBounds=[[-2.5, 2.5], [-2.5, 2.5], [0, 5]]):
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


# for articulated obstacle (3fGripper)
class CagingObstacle(ObjectToCage):
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
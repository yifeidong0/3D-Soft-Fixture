import pybullet as p
from scipy.spatial.transform import Rotation as R
import math

from pb_ompl import PbOMPLRobot

class SoftBall(PbOMPLRobot):
    def __init__(self, id) -> None:
        self.id = id
        # self.num_dim = 4
        self.num_dim = 3
        self.joint_idx=[]
        # self.reset()

        self.joint_bounds = []
        self.joint_bounds.append([-2, 2]) # x
        self.joint_bounds.append([-2, 2]) # y
        self.joint_bounds.append([0, 4]) # z
        # self.joint_bounds.append([math.radians(-180), math.radians(180)]) # r
        # self.joint_bounds.append([math.radians(-180), math.radians(180)]) # p
        # self.joint_bounds.append([math.radians(-180), math.radians(180)]) # y
        # self.joint_bounds.append([math.radians(-0), math.radians(0)]) # joint_0

    def set_bisec_thres(self, zmax):
        self.joint_bounds[2][1] = zmax
        
    def get_joint_bounds(self):
        return self.joint_bounds

    def get_cur_state(self):
        return self.state

    def set_state(self, state):
        pos = state[0:3]
        r = R.from_euler('zyx', [0,0,0], degrees=False)
        # r = R.from_euler('zyx', state[3:6], degrees=False)
        quat = r.as_quat()
        p.resetBasePositionAndOrientation(self.id, pos, quat)
        # self._set_joint_positions(self.joint_idx, [state[-1]])

        self.state = state

    def reset(self):
        p.resetBasePositionAndOrientation(self.id, [0,0,0], [0,0,0,1])
        # self._set_joint_positions(self.joint_idx, [0])
        self.state = [0] * self.num_dim

    # def _set_joint_positions(self, joints, positions):
    #     for joint, value in zip(joints, positions):
    #         p.resetJointState(self.id, joint, value, targetVelocity=0)
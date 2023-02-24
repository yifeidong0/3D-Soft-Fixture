import os.path as osp
import pybullet as p
import math
import sys
import pybullet_data
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import numpy as np
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))

import pb_ompl
from pb_ompl import PbOMPLRobot

class MyObject(PbOMPLRobot):
    def __init__(self, id) -> None:
        self.id = id
        self.num_dim = 6
        self.joint_idx=[]
        self.reset()

        self.joint_bounds = []
        self.joint_bounds.append([-2, 2]) # x
        self.joint_bounds.append([-2, 2]) # y
        self.joint_bounds.append([0, 5]) # z
        self.joint_bounds.append([math.radians(-180), math.radians(180)]) # r
        self.joint_bounds.append([math.radians(-180), math.radians(180)]) # p
        self.joint_bounds.append([math.radians(-180), math.radians(180)]) # y
        # self.joint_bounds.append([math.radians(-0), math.radians(0)]) # joint_0

    def set_bisec_thres(self, zmax):
        self.joint_bounds[2][1] = zmax
        
    def get_joint_bounds(self):
        return self.joint_bounds

    def get_cur_state(self):
        return self.state

    def set_state(self, state):
        pos = [state[0], state[1], state[2]]
        # r = R.from_euler('z', state[2])
        r = R.from_euler('zyx', state[3:6], degrees=False)
        quat = r.as_quat()
        p.resetBasePositionAndOrientation(self.id, pos, quat)
        # self._set_joint_positions(self.joint_idx, [state[-1]])

        self.state = state

    def reset(self):
        p.resetBasePositionAndOrientation(self.id, [0,0,0], [0,0,0,1])
        # self._set_joint_positions(self.joint_idx, [0])
        self.state = [0] * self.num_dim

    def _set_joint_positions(self, joints, positions):
        for joint, value in zip(joints, positions):
            p.resetJointState(self.id, joint, value, targetVelocity=0)

class DonutDemo():
    def __init__(self, eps_thres=1e-2):
        self.obstacles = []

        p.connect(p.GUI)
        p.setGravity(0, 0, -9.8)
        p.setTimeStep(1./240.)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")

        # load robot
        robot_id = p.loadURDF("models/triple_hook/triple_hook.urdf", globalScaling=.03)
        self.robot = MyObject(robot_id)
        
        self.start = [0,0,3,0,1,0] # :3 pos // 3: rot [radian]
        self.goal = [0,0,0,0,0,0]
        
        self.max_z_escapes = [] # successful escapes
        self.eps_thres = eps_thres # threshold of search resolution

    def add_obstacles(self):
        self.add_box([0, 0, 2], [1, 1, 0.01]) # add bottom
        self.add_box([1, 0, 2.5], [0.01, 1, .5]) # add outer walls
        self.add_box([-1, 0, 2.5], [0.01, 1, .5])
        self.add_box([0, 1, 2.5], [1, 0.01, .5])
        self.add_box([0, -1, 2.5], [1, 0.01, .5])

    def add_box(self, box_pos, half_box_size):
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

        self.obstacles.append(box_id)
        return box_id

    def visualize_path(self, path):
        path_z = np.array(path)[:,2]
        max_z_escape = np.max(path_z)
        self.max_z_escapes.append(max_z_escape)

    def demo(self):
        self.robot.set_state(self.start)
        res, path = self.pb_ompl_interface.plan(self.goal)
        if res:
            self.pb_ompl_interface.execute(path)
            self.visualize_path(path)
        else:
            self.max_z_escapes.append(np.inf)
        return res, path

    def find_height_thres_escape(self):
        '''Iteratively find the (lowest) threshold of z upper bound that allows a escaping path'''

        zupper = self.robot.joint_bounds[2][1]
        zlower = self.start[2]
        eps = np.inf
        self.zus, self.zls, self.epss = [], [], []
        idx = 0
        self.itercount = []

        # for i in range(NUMITER):
        while eps > self.eps_thres: 
            # data record
            self.zus.append(zupper)
            self.zls.append(zlower)
            self.itercount.append(idx)

            # set upper bound of searching
            self.pb_ompl_interface.reset_robot_state_bound()
            self.pb_ompl_interface.set_planner("RRT")
            
            # start planning
            self.demo()
            
            # update bounds
            curr_max_z = self.max_z_escapes[-1]
            if curr_max_z == np.inf: # no solution
                zlower = zupper
                zupper = np.min(self.max_z_escapes) # except infs, the target z is monotonically decreasing
            else: # solution found
                zupper = (curr_max_z-zlower) / 2. + zlower # greedily search the lower half bounded by current solution
                # zlower = zlower
            eps = abs(zupper - zlower)
            
            # reset z upper bound
            self.robot.set_bisec_thres(zupper)
            idx += 1

            self.epss.append(eps)
            print("----------max_z_escapes: ", self.max_z_escapes)
            print('----------zupper, zlower, eps: ', zupper, zlower, eps)
            print("----------joint_bounds z: ", self.robot.joint_bounds[2])

        # shut down pybullet (GUI)
        p.disconnect()

    def visualize_bisec_search(self):
        '''visualize the convergence of caging depth'''

        escape_zs = [[i, esc] for i, esc in enumerate(self.max_z_escapes) if esc!=np.inf] # no infs
        escape_zs = np.array(escape_zs)
        escape_energy = escape_zs[-1, 1] - self.start[2] # minimum escape_energy
        z_thres = escape_zs[-1, 1]
        iters, escs = escape_zs[:,0], escape_zs[:,1]
        
        _, ax1 = plt.subplots()
        ax1.plot(iters, escs, '-ro', label='max_z successful escapes') # max z's along successful escape paths
        ax1.plot(self.itercount, self.zus, '-b*', label='upper bounds')
        ax1.plot(self.itercount, self.zls, '--b*', label='lower bounds')
        ax1.axhline(y=self.start[2], color='k', alpha=.3, linestyle='--', label='init_z object')
        ax1.set_xlabel('# iterations')
        ax1.set_ylabel('z_world')
        ax1.grid(True)
        ax1.legend()

        ax2 = ax1.twinx()
        ax2.plot(self.itercount, self.epss, '-g.', label='convergence epsilon')
        ax2.axhline(y=self.eps_thres, color='k', alpha=.7, linestyle='--', label='search resolution')
        ax2.set_ylabel('bound bandwidth')
        ax2.set_yscale('log')
        ax2.legend(loc='lower right')

        plt.title('Iterative bisection search of caging depth')
        plt.show()

        return escape_energy, z_thres

if __name__ == '__main__':
    env = DonutDemo(eps_thres=1e-3)
    env.add_obstacles()
    env.pb_ompl_interface = pb_ompl.PbOMPL(env.robot, env.obstacles)

    # iterative height threshold search
    env.find_height_thres_escape()

    # visualization
    escape_energy, z_thres = env.visualize_bisec_search()
    print('final z threshold: {}, escape energy: {}'.format(escape_energy, z_thres))

    # TODO: comapare the results with ground truth (Open3d OBB - donut model)
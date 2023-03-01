import os.path as osp
import pybullet as p
import math
import sys
import pybullet_data
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))
import pb_ompl
import matplotlib.pyplot as plt
import numpy as np
from time import sleep
from scipy.spatial.transform import Rotation as R

OBJID = 0
OBSID = 0
PATHS = ['models/articulate_fish.xacro', 
         'models/triple_hook/triple_hook.urdf', 
         'models/donut/donut.urdf',
         'models/robotiq_3f_gripper_visualization/cfg/robotiq-3f-gripper_articulated.urdf',
         'models/franka_description/robots/panda_arm.urdf',
         'models/planar_robot_4_link.xacro',
         '']

class ObjectToCage(pb_ompl.PbOMPLRobot):
    def __init__(self, id) -> None:
        self.id = id
        self.comDof = 3 + 3 # SE3
        self.articulate_num = p.getNumJoints(id)
        self.num_dim = self.comDof + self.articulate_num
        self.joint_idx = []
        # self.joint_idx = list(range(self.articulate_num))
        # self.reset()

        self.set_search_bounds()

    def set_search_bounds(self):
        self.joint_bounds = [[-2, 2], [-2, 2], [0, 5]] # CoM pos
        for i in range(3): # CoM rot
            self.joint_bounds.append([math.radians(-180), math.radians(180)]) # r, p, y
        
        for i in range(self.articulate_num):
            info = p.getJointInfo(self.id, i)
            jointType = info[2]
            if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
                self.joint_idx.append(i)
                bounds = p.getJointInfo(self.id, i)[8:10] # joint limits
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
        r = R.from_euler('zyx', eulerRot, degrees=False)
        quat = r.as_quat()
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


class ArticulateDemo():
    def __init__(self, eps_thres=1e-2):
        self.obstacles = []

        p.connect(p.GUI)
        # p.setGravity(0, 0, -9.8)
        p.setTimeStep(1./240.)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # load robot
        robot_id = p.loadURDF(PATHS[OBJID], (0,0,0))
        self.robot = ObjectToCage(robot_id)
        
        self.start = [0,-.5,2.5,0,0,0.78] + [0]*self.robot.articulate_num # :3 pos // 3: rot [radian]
        self.goal = [0,0,0,0,0,0] + [0]*self.robot.articulate_num
        
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

    def track_path(self, path):
        path_z = np.array(path)[:,2]
        max_z_escape = np.max(path_z)
        self.max_z_escapes.append(max_z_escape)
        
        depth = np.around(np.max(path_z)-path_z[0], decimals=2)
        plt.plot(path_z)
        plt.xlabel('path node')
        plt.ylabel('height')
        plt.title('Depth of Energy-bounded Caging: {}'.format(depth))
        plt.show()

    def demo(self):
        self.robot.set_state(self.start)
        # sleep(1240.)
        res, path = self.pb_ompl_interface.plan(self.goal)
        if res:
            self.pb_ompl_interface.execute(path)
            self.track_path(path)
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

        while eps > self.eps_thres: 
            # data record
            self.zus.append(zupper)
            self.zls.append(zlower)
            self.itercount.append(idx)

            # set upper bound of searching
            self.pb_ompl_interface.reset_robot_state_bound()
            self.pb_ompl_interface.set_planner("RRTstar")
            
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
    env = ArticulateDemo(eps_thres=1e-1)
    env.add_obstacles()
    env.pb_ompl_interface = pb_ompl.PbOMPL(env.robot, env.start, env.obstacles)

    # iterative height threshold search
    env.find_height_thres_escape()

    # visualization
    escape_energy, z_thres = env.visualize_bisec_search()
    print('final z threshold: {}, escape energy: {}'.format(z_thres, escape_energy))

    # TODO: comapare the results with ground truth (Open3d OBB - donut model)
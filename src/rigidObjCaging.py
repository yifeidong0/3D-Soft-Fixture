import os.path as osp
import pybullet as p
import sys
import pybullet_data
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))
import matplotlib.pyplot as plt
import numpy as np
from time import sleep
from object import ObjectToCage
from utils import path_collector

class RigidObjectCaging():
    def __init__(self, args, eps_thres=1e-2):
        self.args = args
        self.obstacles = []

        if args.visualization:
            vis = p.GUI
        else:
            vis = p.DIRECT
        p.connect(vis)
        # p.setGravity(0, 0, -9.8)
        p.setTimeStep(1./240.)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.load_object()
        self.reset_start_and_goal()

        self.max_z_escapes = [] # successful escapes
        self.eps_thres = eps_thres # bi-section search resolution

    def load_object(self):
        """Load object for caging."""
        self.paths = path_collector()

        self.robot_id = p.loadURDF(self.paths[self.args.object], (0,0,0))
        self.robot = ObjectToCage(self.robot_id)

    def reset_start_and_goal(self, start=None, goal=None):
        # Set start and goal nodes of searching algorithms
        if start is None:
            self.start = [0,0,2.6,1.57,0,0] + [0]*self.robot.articulate_num # :3 pos // 3: rot [radian]
        else:
            self.start = start
        if goal is None:
            self.goal = [0,0,0,0,0,0] + [0]*self.robot.articulate_num
        else:
            self.goal = goal

        # make sure states are within search bounds
        jbounds = self.robot.get_joint_bounds()
        startBools = [self.start[i]>=jbounds[i][0] and self.start[i]<=jbounds[i][1] for i in range(len(jbounds))]
        goalBools = [self.goal[i]>=jbounds[i][0] and self.goal[i]<=jbounds[i][1] for i in range(len(jbounds))]
        if startBools.count(False)>0 or goalBools.count(False)>0: # some bounds restrictions are violated
            print('The start or goal states violates search boundary conditions!')
            return False 
        
        return True # bounds valid check passed

    def add_obstacles(self, pos=[-0.5, 1.5, 0], orn=(1,0,0,1), scale=[.1, .1, .1]):
        if self.args.obstacle == 'Box':
            self.add_box([0, 0, 2], [1, 1, 0.01]) # add bottom
            self.add_box([1, 0, 2.5], [0.01, 1, 1.0]) # add outer walls
            self.add_box([-1, 0, 2.5], [0.01, 1, 1.0])
            self.add_box([0, 1, 2.5], [1, 0.01, 1.0])
            self.add_box([0, -1, 2.5], [1, 0.01, 1.0])
        
        elif self.args.obstacle == 'Bowl' or 'Hook':
            # Upload the mesh data to PyBullet and create a static object
            # mesh_scale = [.1, .1, .1]  # The scale of the mesh
            mesh_collision_shape = p.createCollisionShape(
                shapeType=p.GEOM_MESH,
                fileName=self.paths[self.args.obstacle],
                meshScale=scale,
                flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
            )
            mesh_visual_shape = p.createVisualShape(shapeType=p.GEOM_MESH,
                fileName=self.paths[self.args.obstacle],
                rgbaColor=[1, 1, 1, 1],
                specularColor=[0.4, .4, 0],
                # visualFramePosition=shift,
                meshScale=scale
            )
            # mesh_visual_shape = -1  # Use the same shape for visualization
            mesh_position = pos  # The position of the mesh
            mesh_orientation = orn  # The orientation of the mesh
            self.obstacle_id = p.createMultiBody(
                baseCollisionShapeIndex=mesh_collision_shape,
                baseVisualShapeIndex=mesh_visual_shape,
                basePosition=mesh_position,
                baseOrientation=mesh_orientation,
            )
            self.obstacles.append(self.obstacle_id)

    def add_box(self, box_pos, half_box_size):
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

        self.obstacles.append(box_id)
        return box_id

    def track_path_cost(self, path):
        self.path_z = np.array(path)[:,2]
        max_z_escape = np.max(self.path_z)
        self.max_z_escapes.append(max_z_escape)

    def execute_search(self):
        # sleep(1240.)
        res, path, sol_path_energy, sol_final_cost = self.pb_ompl_interface.plan(self.goal, self.args.runtime)
        if res:
            self.pb_ompl_interface.execute(path)
            if self.args.objective == 'GravityPotential':
                self.track_path_cost(path)
        else:
            self.max_z_escapes.append(np.inf)
        return res, path, sol_path_energy, sol_final_cost

    def energy_minimize_search(self, numIter=1):
        # set upper bound of searching
        self.pb_ompl_interface.reset_robot_state_bound()
        self.robot.set_state(self.start)
        # self.pb_ompl_interface.set_planner(self.args.planner, self.goal)
        
        # start planning
        self.energy_minimize_paths_energies = []
        self.sol_final_costs = []
        solveds = []
        for i in range(numIter):
            self.robot.set_state(self.start)
            self.pb_ompl_interface.set_planner(self.args.planner, self.goal)
            solved, _, sol_path_energy, sol_final_cost = self.execute_search()
            self.energy_minimize_paths_energies.append(sol_path_energy)      
            self.sol_final_costs.append(sol_final_cost)
            solveds.append(solved)
        if solveds.count(True) == 0:
            return False
        return True


    def visualize_energy_minimize_search(self):
        '''visualize the convergence of caging depth'''

        _, ax1 = plt.subplots()
        energy_curves = self.energy_minimize_paths_energies
        for i in range(len(energy_curves)):
            ax1.plot(energy_curves[i], label='path {}'.format(i)) # max z's along successful escape paths
        ax1.set_xlabel('vertices of solution paths')
        ax1.set_ylabel('state energy')
        ax1.grid(True)
        ax1.legend()

        plt.title('Iterative BIT* energy minimize search')
        plt.show()

    def bound_shrink_search(self, useBisecSearch=False):
        '''Iteratively find the (lowest) threshold of z upper bound that allows an escaping path'''

        zupper = self.robot.joint_bounds[2][1]
        zlower = self.start[2]
        eps = np.inf
        self.zus, self.zls, self.epss = [], [], []
        idx = 0
        self.itercount = []
        noinf = []
        moveOn = True

        while moveOn: 
            # data record
            self.zus.append(zupper)
            self.zls.append(zlower)
            self.itercount.append(idx)

            # set upper bound of searching
            self.pb_ompl_interface.reset_robot_state_bound()
            self.robot.set_state(self.start)
            self.pb_ompl_interface.set_planner(self.args.planner, self.goal)
            
            # start planning
            _, _, _, _ = self.execute_search()
            
            # update bounds
            curr_max_z = self.max_z_escapes[-1]
            noinf.append(curr_max_z) if curr_max_z!=np.inf else None

            if useBisecSearch: # greedy but no lower bound guarantee
                if curr_max_z == np.inf: # no solution
                    zlower = zupper
                    zupper = np.min(self.max_z_escapes) # except infs, the target z is monotonically decreasing
                else: # solution found
                    if curr_max_z < zlower: # a solution lower than lower bound found
                        zupper = curr_max_z
                        zlower = self.start[2]
                    else: # solution found within expected bounds
                        zupper = (curr_max_z-zlower) / 2. + zlower # greedily search the lower half bounded by current solution
                    # zlower = zlower
                
                eps = abs(zupper-zlower)
                self.epss.append(eps)
                if eps < self.eps_thres: # terminate
                    moveOn = False

            else: # slower but guaranteed lower bound
                eps = np.inf if len(self.epss)==0 else self.epss[-1]
                if curr_max_z != np.inf: # solution found
                    zupper = curr_max_z
                    
                    # check conditions to break loops
                    if len(noinf) >= 2:
                        eps = noinf[-2] - noinf[-1]

                        # terminate if small eps's appear in a row
                        rowlen = 3
                        if len(self.epss) > rowlen:
                            LastFewEpsIsSmall = [e<self.eps_thres for e in self.epss[-rowlen:]]
                            if LastFewEpsIsSmall.count(True) == rowlen:
                                moveOn = False
                self.epss.append(eps)

                # stop if more than two times invalid search
                maxNumInfs = 2
                if self.max_z_escapes.count(np.inf) > maxNumInfs:
                    moveOn = False

            # reset z upper bound
            self.robot.set_bisec_thres(zupper)
            idx += 1

            print("----------max_z_escapes: ", self.max_z_escapes)
            print('----------zupper, zlower, eps: ', zupper, zlower, eps)
            print("----------joint_bounds z: ", self.robot.joint_bounds[2])

    def visualize_bound_shrink_search(self, useBisecSearch=False):
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
        if useBisecSearch:
            ax2.set_ylabel('bound bandwidth')
        else:
            ax2.set_ylabel('previous heights diff')
        ax2.set_yscale('log')
        ax2.legend(loc='lower right')

        plt.title('Iterative bound shrinking search of caging depth')
        plt.show()

        return escape_energy, z_thres
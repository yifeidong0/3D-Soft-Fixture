import os.path as osp
import pybullet as p
import sys
import pybullet_data
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))
import matplotlib.pyplot as plt
import numpy as np
from time import sleep
from object import ObjectToCage, CagingObstacle
from utils import path_collector
import copy
import datetime
import os
import csv

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
        self.reset_start_and_goal() # TODO

        self.eps_thres = eps_thres # bi-section search resolution

    def load_object(self):
        """Load object for caging."""
        self.paths = path_collector()
        self.object_id = p.loadURDF(self.paths[self.args.object], (0,0,0))
        self.robot = ObjectToCage(self.object_id)

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
        # print('@@@@self.start', self.start)
        # print('@@@@jbounds', jbounds)
        startBools = [self.start[i]>=jbounds[i][0] and self.start[i]<=jbounds[i][1] for i in range(len(jbounds))]
        goalBools = [self.goal[i]>=jbounds[i][0] and self.goal[i]<=jbounds[i][1] for i in range(len(jbounds))]
        if startBools.count(False)>0 or goalBools.count(False)>0: # some bounds restrictions are violated
            print('The start or goal states violates search boundary conditions!')
            return False 
        
        return True # bounds valid check passed

    def add_obstacles(self, pos=[-0.5, 1.5, 0], qtn=(1,0,0,1), scale=[.1, .1, .1], jointPos=None):
        obst = self.args.obstacle
        if  obst == 'Box':
            self.add_box([0, 0, 2], [1, 1, 0.01]) # add bottom
            self.add_box([1, 0, 2.5], [0.01, 1, 1.0]) # add outer walls
            self.add_box([-1, 0, 2.5], [0.01, 1, 1.0])
            self.add_box([0, 1, 2.5], [1, 0.01, 1.0])
            self.add_box([0, -1, 2.5], [1, 0.01, 1.0])
        
        elif obst == 'Bowl' or obst == 'Hook':
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
            mesh_orientation = qtn  # The orientation of the mesh
            self.obstacle_id = p.createMultiBody(
                baseCollisionShapeIndex=mesh_collision_shape,
                baseVisualShapeIndex=mesh_visual_shape,
                basePosition=mesh_position,
                baseOrientation=mesh_orientation,
            )
            self.obstacles.append(self.obstacle_id)

        elif obst == '3fGripper':
            self.obstacle_id = p.loadURDF(self.paths[self.args.obstacle], 
                                          pos, qtn, globalScaling=scale)
            self.obstacle = CagingObstacle(self.obstacle_id)
            self.obstacle._set_joint_positions(self.obstacle.joint_idx, jointPos)
            self.obstacles.append(self.obstacle_id)

    def add_box(self, box_pos, half_box_size):
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

        self.obstacles.append(box_id)
        return box_id

    def track_path_cost(self, path):
        self.path_z = np.array(path)[:,2]
        max_z_escape = np.max(self.path_z)
        self.escape_cost_list.append(max_z_escape-self.start[2])

    def execute_search(self):
        # sleep(1240.)
        res, path, sol_path_energy, best_cost, time_taken = self.pb_ompl_interface.plan(self.goal, self.args.runtime)
        if res:
            if self.args.visualization:
                self.pb_ompl_interface.execute(path)
            if self.args.objective == 'GravityPotential':
                self.track_path_cost(path)
        else:
            if self.args.search == 'BoundShrinkSearch':
               self.escape_cost_list.append(self.escape_cost_list[-1])
        return res, path, sol_path_energy, best_cost, time_taken

    def energy_minimize_search(self, numIter=1):
        # set upper bound of searching
        self.pb_ompl_interface.reset_robot_state_bound()
        # self.robot.set_state(self.start)
        # self.pb_ompl_interface.set_planner(self.args.planner, self.goal)
        
        # start planning
        self.energy_minimize_paths_energies = []
        self.sol_final_costs = []
        self.escape_cost_list = [] # successful escapes
        solveds = []
        for i in range(numIter):
            self.robot.set_state(self.start)
            self.pb_ompl_interface.set_planner(self.args.planner, self.goal)
            solved, _, sol_path_energy, sol_final_cost, time_taken = self.execute_search()
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

    def bound_shrink_search(self, useGreedySearch=False, initSearchBound=None, numIter=2, maxTimeTaken=30):
        '''Iteratively find the (lowest) threshold of z upper bound that allows an escaping path'''
        self.time_taken_list_runs, self.escape_cost_list_runs = [], [] # record over several runs
        initUpperBound = copy.deepcopy(initSearchBound[2][1])
        
        for i in range(numIter):        
            cUpper = initSearchBound[2][1] - self.start[2]
            cLower = 0
            eps = np.inf
            # self.cUppers, self.cLowers = [], []
            self.epss = []
            idx = 0
            self.itercount = []
            self.escape_cost_list = [] # successful escapes
            self.time_taken_list = [] # corresponding time taken
            self.times_of_no_solution = 0
            noinf = []
            moveOn = True

            while moveOn: 
                # data record
                # self.cUppers.append(cUpper)
                # self.cLowers.append(cLower)
                self.itercount.append(idx)

                # set upper bound of searching
                self.pb_ompl_interface.reset_robot_state_bound()
                self.robot.set_state(self.start)
                # print('@@@@@@DIST', len(p.getClosestPoints(bodyA=self.object_id, bodyB=self.obstacle_id, distance=-0.027)))

                self.pb_ompl_interface.set_planner(self.args.planner, self.goal)
                
                # start planning
                res, _, _, _, time_taken = self.execute_search()

                # record times of failure and time 
                if not res:
                    self.times_of_no_solution += 1
                accumulate_time = time_taken + self.time_taken_list[-1] if len(self.time_taken_list)>0 else time_taken
                self.time_taken_list.append(accumulate_time)

                # update bounds
                curr_cost = self.escape_cost_list[-1]
                # curr_max_z = self.max_z_escapes[-1]
                noinf.append(curr_cost) if curr_cost!=np.inf else None

                if useGreedySearch: # greedy but no lower bound guarantee
                    if not res: # no solution
                        cLower = cUpper
                        cUpper = np.min(self.escape_cost_list) # except infs, the target z is monotonically decreasing
                    else: # solution found
                        if curr_cost < cLower: # a solution lower than lower bound found
                            cUpper = curr_cost
                            cLower = 0
                        else: # solution found within expected bounds
                            cUpper = (curr_cost-cLower) / 2. + cLower # greedily search the lower half bounded by current solution
                        # cLower = cLower
                    
                    eps = abs(cUpper-cLower)
                    self.epss.append(eps)
                    if eps < self.eps_thres: # terminate
                        moveOn = False

                else: # slower but guaranteed lower bound
                    # Update upper bound
                    if res: # solution found
                        cUpper = curr_cost

                    # stop if invalid search appears twice
                    # maxNumInfs = 3
                    # if self.times_of_no_solution >= maxNumInfs:
                    #     moveOn = False
                    if self.time_taken_list[-1] > maxTimeTaken:
                        moveOn = False

                # reset z upper bound
                self.robot.set_bisec_thres(cUpper+self.start[2])
                idx += 1

                print("----------escape_cost_list: ", self.escape_cost_list)
                print("----------time_taken_list: ", self.time_taken_list)
                print('----------cUpper, cLower, eps: ', cUpper, cLower, eps)
                # print("----------joint_bounds z: ", self.robot.joint_bounds[2])
            
            # record data over runs
            self.time_taken_list_runs.append(self.time_taken_list) 
            self.escape_cost_list_runs.append(self.escape_cost_list)

            # reset and prepare for the next run
            self.robot.set_bisec_thres(initUpperBound)
            
    def visualize_bound_shrink_search(self, useGreedySearch=False):
        '''visualize the convergence of caging depth'''

        # escape_cost_list_noinf = [self.escape_cost_list[i] if self.escape_cost_list[i] != np.inf else self.escape_cost_list[i-1] for i in range(len(self.escape_cost_list))]
        # escape_costs = [[i, esc] for i, esc in enumerate(self.escape_cost_list)] # no infs
        # escape_costs = np.array(escape_costs)
        # escape_energy = escape_costs[-1, 1] # minimum escape_energy
        # z_thres = escape_costs[-1, 1]
        # iters, escs = escape_costs[:,0], escape_costs[:,1]
        # print('@@@escape_costs', escs)

        _, ax1 = plt.subplots()
        for i in range(len(self.time_taken_list_runs)):
            ax1.plot(self.time_taken_list_runs[i], self.escape_cost_list_runs[i], label='Costs of escapes in run {}'.format(i))
        # ax1.plot(self.time_taken_list, self.cUppers, '-b*', label='Search upper bound')
        # ax1.plot(self.time_taken_list, self.cLowers, '--b*', label='Search lower bound')
        # ax1.axhline(y=self.start[2], color='k', alpha=.3, linestyle='--', label='init_z object')
        ax1.set_xlabel('Time of search / s')
        ax1.set_ylabel('Cost (Energy gain) / J')
        ax1.grid(True)
        ax1.legend()

        # ax2 = ax1.twinx()
        # ax2.plot(self.itercount, self.epss, '-g.', label='convergence epsilon')
        # ax2.axhline(y=self.eps_thres, color='k', alpha=.7, linestyle='--', label='search resolution')
        # if useGreedySearch:
        #     ax2.set_ylabel('bound bandwidth')
        # else:
        #     ax2.set_ylabel('previous heights diff')
        # ax2.set_yscale('log')
        # ax2.legend(loc='lower right')

        plt.title('Iterative bound shrinking search of caging escape energy cost')
        plt.show()

        # return escape_energy, z_thres
    
    
class ArticulatedObjectCaging(RigidObjectCaging):
    def __init__(self, args):
        self.args = args
        self.obstacles = []

        if args.visualization:
            vis = p.GUI
        else:
            vis = p.DIRECT
        p.connect(vis)     

        p.setTimeStep(1./240.)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.load_object()
        self.reset_start_and_goal()

        self.escape_cost_list = [] # successful escapes
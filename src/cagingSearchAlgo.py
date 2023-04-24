import os.path as osp
import pybullet as p
import sys
import pybullet_data
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))
import matplotlib.pyplot as plt
import numpy as np
from time import sleep
from object import *
from utils import *
import copy
from pbOmplInterface import PbOMPL

class RigidObjectCaging():
    def __init__(self, args):
        self.args = args
        self.obstacles = []
        self.rigidObjList = get_non_articulated_objects()

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

        self.eps_thres = 1e-2 # bi-section search resolution

    def load_object(self):
        """Load object for caging."""
        self.paths = path_collector()
        self.object_id = p.loadURDF(self.paths[self.args.object], (0,0,0))
        self.robot = ObjectFromUrdf(self.object_id)

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

    def add_obstacles(self, pos=[0,0,0], qtn=(1,0,0,1), scale=[.1, .1, .1], jointPos=None):
        obst = self.args.obstacle
        if  obst == 'Box':
            self.add_box([0, 0, .7], [.7, .7, 0.2]) # add bottom
            self.add_box([.5, 0, 1.2], [0.2, .7, .4]) # add outer walls
            self.add_box([-.5, 0, 1.2], [0.2, .7, .6])
            self.add_box([0, .5, 1.2], [.7, 0.2, .5])
            self.add_box([0, -.5, 1.2], [.7, 0.2, .7])
        
        elif obst in self.rigidObjList:
            # Upload the mesh data to PyBullet and create a static object
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
            self.obstacle = obstascle3fGripper(self.obstacle_id)
            self.obstacle._set_joint_positions(self.obstacle.joint_idx, jointPos)
            self.obstacles.append(self.obstacle_id)

    def add_box(self, box_pos, half_box_size):
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos)

        self.obstacles.append(box_id)
        return box_id

    def create_ompl_interface(self):
        self.pb_ompl_interface = PbOMPL(self.robot, self.args, self.obstacles)

    # def track_path_cost(self, path):
    #     self.path_z = np.array(path)[:,2]
    #     max_z_escape = np.max(self.path_z)
    #     self.escape_cost_list.append(max_z_escape-self.start[2])

    def track_path_cost(self, path):
        energy_along_path = [get_state_energy(state, self.args.object) for state in path]
        escape_energy = max(energy_along_path) - energy_along_path[0]
        self.escape_cost_list.append(escape_energy)

    def execute_search(self):
        res, path, sol_path_energy, best_cost, time_taken = self.pb_ompl_interface.plan(self.goal, self.args.runtime)
        if res:
            if self.args.visualization:
                self.pb_ompl_interface.execute(path)
            # if self.args.objective == 'GravityPotential':
            self.track_path_cost(path)
        else:
            if self.args.search == 'BoundShrinkSearch':
               self.escape_cost_list.append(self.escape_cost_list[-1])
        return res, path, sol_path_energy, best_cost, time_taken

    def energy_minimize_search(self, numIter=1):
        self.pb_ompl_interface.reset_robot_state_bound()
        self.sol_path_energy_list = []
        self.sol_final_costs = []
        self.escape_cost_list = [] # successful escapes
        solveds = []
        for i in range(numIter):
            self.robot.set_state(self.start)
            self.pb_ompl_interface.set_planner(self.args.planner, self.goal)
            solved, _, sol_path_energy, sol_final_cost, _ = self.execute_search()
            self.sol_path_energy_list.append(sol_path_energy)      
            self.sol_final_costs.append(sol_final_cost)
            solveds.append(solved)
        if solveds.count(True) == 0:
            return False
        return True

    def visualize_energy_minimize_search(self):
        '''visualize the convergence of caging depth
        '''
        _, ax1 = plt.subplots()
        for i in range(len(self.sol_path_energy_list)):
            ax1.plot(self.sol_path_energy_list[i], label='path {}'.format(i)) # max z's along successful escape paths
        ax1.set_xlabel('vertices of solution paths')
        ax1.set_ylabel('state energy')
        ax1.grid(True)
        ax1.legend()
        plt.title('State energy along the escape path in the energy biased search')
        plt.show()

    def energy_bisection_search(self, numIter=1, maxTimeTaken=120, epsThreshold=1e-3):
        '''Iteratively find the (lowest) threshold of z upper bound that allows an escaping path
        '''
        initCUpper = 10
        self.time_taken_list_runs, self.escape_cost_list_runs = [], [] # record over several runs
        startEnergy = get_state_energy(self.start, self.args.object)
        for i in range(numIter):
            cUpper, cLower = initCUpper, 0
            self.escape_cost_list = [] # successful escapes
            self.time_taken_list = [] # corresponding time taken
            moveOn = True

            while moveOn: 
                # set upper bound of searching
                self.pb_ompl_interface.reset_robot_state_bound()
                self.robot.set_state(self.start)
                self.pb_ompl_interface.set_planner(self.args.planner, self.goal)
                
                # Start planning
                res, _, _, _, time_taken = self.execute_search()

                # Record time 
                accumulate_time = time_taken + self.time_taken_list[-1] if len(self.time_taken_list)>0 else time_taken
                self.time_taken_list.append(accumulate_time)

                # update bounds
                curr_cost = self.escape_cost_list[-1]

                # Set lower and upper bounds
                if not res: # no solution
                    cLower = cUpper
                    cUpper = np.min(self.escape_cost_list)
                else: # solution found
                    if curr_cost < cLower: # a solution lower than lower bound found
                        cUpper = curr_cost
                        cLower = 0
                    else: # solution found within expected bounds
                        cUpper = (curr_cost-cLower) / 2. + cLower # greedily search the lower half bounded by current solution
                
                # Conditions of termination
                eps = abs(cUpper-cLower)
                if eps < epsThreshold or accumulate_time > maxTimeTaken:
                    moveOn = False

                # Reset energy threshold
                self.pb_ompl_interface.reset_bisec_energy_thres(cUpper+startEnergy)

                print("----------escape_cost_list: ", self.escape_cost_list)
                print("----------time_taken_list: ", self.time_taken_list)
                print("----------cUpper, cLower: ", cUpper, cLower)
            
            # record data over runs
            self.time_taken_list_runs.append(self.time_taken_list) 
            self.escape_cost_list_runs.append(self.escape_cost_list)

            # reset and prepare for the next run
            self.pb_ompl_interface.reset_bisec_energy_thres(initCUpper)

    def visualize_bound_shrink_search(self):
        '''visualize the convergence of caging depth
        '''
        _, ax1 = plt.subplots()
        for i in range(len(self.time_taken_list_runs)):
            ax1.plot(self.time_taken_list_runs[i], self.escape_cost_list_runs[i], label='Costs of escapes in run {}'.format(i))
        ax1.set_xlabel('Accumulated search time / s')
        ax1.set_ylabel('Escape cost / J')
        ax1.grid(True)
        ax1.legend()
        plt.title('Iterative bound shrinking search of caging escape energy')
        plt.show()


    # def bound_shrink_search(self, useGreedySearch=1, initSearchBound=[[-2,2], [-2,2], [0,3]], numIter=1, maxTimeTaken=30):
    #     '''Iteratively find the (lowest) threshold of z upper bound that allows an escaping path
    #     '''
    #     self.time_taken_list_runs, self.escape_cost_list_runs = [], [] # record over several runs
    #     initUpperBound = copy.deepcopy(initSearchBound[2][1])
        
    #     for i in range(numIter):        
    #         cUpper = initSearchBound[2][1] - self.start[2]
    #         cLower = 0
    #         eps = np.inf
    #         self.epss = []
    #         idx = 0
    #         self.itercount = []
    #         self.escape_cost_list = [] # successful escapes
    #         self.time_taken_list = [] # corresponding time taken
    #         self.times_of_no_solution = 0
    #         noinf = []
    #         moveOn = True

    #         while moveOn: 
    #             # data record
    #             self.itercount.append(idx)

    #             # set upper bound of searching
    #             self.pb_ompl_interface.reset_robot_state_bound()
    #             self.robot.set_state(self.start)

    #             self.pb_ompl_interface.set_planner(self.args.planner, self.goal)
                
    #             # start planning
    #             res, _, _, _, time_taken = self.execute_search()

    #             # record times of failure and time 
    #             if not res:
    #                 self.times_of_no_solution += 1
    #             accumulate_time = time_taken + self.time_taken_list[-1] if len(self.time_taken_list)>0 else time_taken
    #             self.time_taken_list.append(accumulate_time)

    #             # update bounds
    #             curr_cost = self.escape_cost_list[-1]
    #             noinf.append(curr_cost) if curr_cost!=np.inf else None

    #             if useGreedySearch: # greedy but no lower bound guarantee
    #                 if not res: # no solution
    #                     cLower = cUpper
    #                     cUpper = np.min(self.escape_cost_list) # except infs, the target z is monotonically decreasing
    #                 else: # solution found
    #                     if curr_cost < cLower: # a solution lower than lower bound found
    #                         cUpper = curr_cost
    #                         cLower = 0
    #                     else: # solution found within expected bounds
    #                         cUpper = (curr_cost-cLower) / 2. + cLower # greedily search the lower half bounded by current solution
                    
    #                 eps = abs(cUpper-cLower)
    #                 self.epss.append(eps)
    #                 if eps < self.eps_thres: # terminate
    #                     moveOn = False

    #             else: # slower but guaranteed lower bound
    #                 # Update upper bound
    #                 if res: # solution found
    #                     cUpper = curr_cost

    #                 # Cost already zero
    #                 if cUpper <= 0.0:
    #                     moveOn = False

    #                 # Time up
    #                 if self.time_taken_list[-1] > maxTimeTaken:
    #                     moveOn = False

    #             # Reset z upper bound
    #             self.robot.set_bisec_thres(cUpper+self.start[2])
    #             idx += 1

    #             print("----------escape_cost_list: ", self.escape_cost_list)
    #             print("----------time_taken_list: ", self.time_taken_list)
            
    #         # record data over runs
    #         self.time_taken_list_runs.append(self.time_taken_list) 
    #         self.escape_cost_list_runs.append(self.escape_cost_list)

    #         # reset and prepare for the next run
    #         self.robot.set_bisec_thres(initUpperBound)
            



class ArticulatedObjectCaging(RigidObjectCaging):
    def __init__(self, args):
        self.args = args
        self.obstacles = []
        self.rigidObjList = get_non_articulated_objects()

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


class ElasticBandCaging(RigidObjectCaging):
    def __init__(self, args, numCtrlPoint, start, goal):
        self.args = args
        self.obstacles = []
        self.rigidObjList = get_non_articulated_objects()

        if args.visualization:
            vis = p.GUI
        else:
            vis = p.DIRECT
        p.connect(vis)
        p.setTimeStep(1./240.)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.numCtrlPoint = numCtrlPoint
        self.start = start
        self.goal = goal
        self.load_object()
        self.reset_start_and_goal(start, goal)

    def load_object(self):
        """Load object for caging."""
        self.paths = path_collector()
        self.object_id = []
        # for i in range(self.numCtrlPoint):
        #    self.object_id.append(p.loadURDF("sphere_1cm.urdf", (0,0,0), globalScaling=0.01)) # '1cm': diameter

        self.robot = objectElasticBand(self.object_id, self.numCtrlPoint)

    def reset_start_and_goal(self, start=None, goal=None):
        # Set start and goal nodes of searching algorithms
        self.start = [0,0,2]*self.numCtrlPoint if start is None else start
        self.goal = [0,0,3]*self.numCtrlPoint if goal is None else goal

        # make sure states are within search bounds
        jbounds = self.robot.get_joint_bounds()
        startBools = [self.start[i]>=jbounds[i][0] and self.start[i]<=jbounds[i][1] for i in range(len(jbounds))]
        goalBools = [self.goal[i]>=jbounds[i][0] and self.goal[i]<=jbounds[i][1] for i in range(len(jbounds))]
        if startBools.count(False)>0 or goalBools.count(False)>0: # some bounds restrictions are violated
            print('The start or goal states violates search boundary conditions!')
            return False 
        
        return True # bounds valid check passed


class RopeCaging(RigidObjectCaging):
    def __init__(self, args, numCtrlPoint, linkLen, start, goal):
        self.args = args
        self.numCtrlPoint = numCtrlPoint
        self.start = start
        self.goal = goal
        self.linkLen = linkLen
        self.obstacles = []
        self.rigidObjList = get_non_articulated_objects()

        if args.visualization:
            vis = p.GUI
        else:
            vis = p.DIRECT
        p.connect(vis)
        p.setTimeStep(1./240.)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.load_object()
        self.reset_start_and_goal(start, goal)

    def load_object(self):
        """Load object for caging."""
        self.paths = path_collector()
        self.object_id = []
        for i in range(self.numCtrlPoint+2): # number of nodes
           self.object_id.append(p.loadURDF("sphere_1cm.urdf", (0,0,0), globalScaling=.01)) # '1cm': diameter
        # print('@@@@ self.object_id', self.object_id)
        self.robot = objectRope(self.object_id, self.numCtrlPoint, self.linkLen)

    def reset_start_and_goal(self, start=None, goal=None):
        # Set start and goal nodes of searching algorithms
        self.start = [0,0,2,0,0,0] + [0,0]*self.numCtrlPoint if start is None else start
        self.goal = [0,0,0.1,0,0,0] + [0,0]*self.numCtrlPoint if goal is None else goal

        # make sure states are within search bounds
        jbounds = self.robot.get_joint_bounds()
        startBools = [self.start[i]>=jbounds[i][0] and self.start[i]<=jbounds[i][1] for i in range(len(jbounds))]
        goalBools = [self.goal[i]>=jbounds[i][0] and self.goal[i]<=jbounds[i][1] for i in range(len(jbounds))]
        if startBools.count(False)>0 or goalBools.count(False)>0: # some bounds restrictions are violated
            print('The start or goal states violates search boundary conditions!')
            return False 
        
        return True # bounds valid check passed
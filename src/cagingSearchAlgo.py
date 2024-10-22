"""
Title: Preparation of the soft fixturing scenarios.
Author: Yifei Dong
Date: 14/07/2023
Description: The script provides an interface of running our iterative or energy-biased search algorithms 
and analyzing the escape energy within a single frame of various soft fixture cases.
"""

import os.path as osp
import pybullet as p
import sys
import pybullet_data
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))
import matplotlib.pyplot as plt
from time import sleep
from object import *
from utils import *
from pbOmplInterface import PbOMPL
from visualization.visualization import *

class RigidObjectCaging():
    '''
    For rigid objects.
    '''
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
        self.reset_start_and_goal() # TODO

        self.eps_thres = 1e-2 # bi-section search resolution

    def load_object(self, scale=1):
        '''
        Load object for caging.
        '''
        self.paths = path_collector()
        self.object_id = p.loadURDF(self.paths[self.args.object], (0,0,0), globalScaling=scale)
        self.robot = ObjectFromUrdf(self.object_id)

        # FOR DEBUG - URDF TUNE
        # self.robot.set_state([0,0,2,0,0,0]+[0.7,0.8])
        # axiscreator(self.object_id, linkId = -1)
        # axiscreator(self.object_id, linkId = 0) # first joint pose is displayed
        # axiscreator(self.object_id, linkId = 1)

    def reset_start_and_goal(self, start=None, goal=None,):
        '''
        Set start and goal nodes of searching algorithms..
        '''
        if start is None:
            self.start = [0,0,2.6,1.57,0,0] + [0]*self.robot.articulate_num # :3 pos // 3: rot [radian]
        else:
            self.start = start
        if goal is None:
            self.goal = [0,0,0,0,0,0] + [0]*self.robot.articulate_num
        else:
            self.goal = goal
        
        return True # bounds valid check passed

    def add_obstacles(self, pos=[0,0,0], qtn=(1,0,0,1), scale=[.1, .1, .1], jointPos=None, obstacleName=None):
        '''
        Add obstacles to the pybullet environment.
        '''
        if obstacleName is None:
            if self.args.obstacle in ['Box']:
                # if self.args.object in ['Rope']:
                self.add_box([0, 0, .7], [.7, .7, 0.2]) # add bottom
                self.add_box([.5, 0, 1.2], [0.2, .7, .4]) # add outer walls
                self.add_box([-.5, 0, 1.2], [0.2, .7, .6])
                self.add_box([0, .5, 1.2], [.7, 0.2, .5])
                self.add_box([0, -.5, 1.2], [.7, 0.2, .7])
            
            elif self.args.obstacle in ['Hole']:
                self.add_box([1.2, 0, 1.2], [1, 1, .2])
                self.add_box([-1.2, 0, 1.2], [1, 1, .2])
                self.add_box([0, 1.2, 1.2], [1, 1, .2])
                self.add_box([0, -1.2, 1.2], [1, 1, .2])

            elif self.args.obstacle in self.rigidObjList:
                # Upload the mesh data to PyBullet and create a static object
                mesh_collision_shape = p.createCollisionShape(
                    shapeType=p.GEOM_MESH,
                    fileName=self.paths[self.args.obstacle],
                    meshScale=scale,
                    flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
                )
                mesh_visual_shape = -1  # Use the same shape for visualization
                self.obstacle_id = p.createMultiBody(
                    baseCollisionShapeIndex=mesh_collision_shape,
                    baseVisualShapeIndex=mesh_visual_shape,
                    basePosition=pos,
                    baseOrientation=qtn,
                )
                self.obstacles.append(self.obstacle_id)

            elif self.args.obstacle in ['3fGripper', 'ShadowHand']:
                self.obstacle_id = p.loadURDF(self.paths[self.args.obstacle], 
                                            pos, qtn, globalScaling=scale[0])
                self.obstacle = obstascle3fGripper(self.obstacle_id)
                self.obstacles.append(self.obstacle_id)
        elif obstacleName in self.rigidObjList:
            mesh_collision_shape = p.createCollisionShape(
                shapeType=p.GEOM_MESH,
                fileName=self.paths[obstacleName],
                meshScale=scale,
                flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
            )
            self.obstacle_id_new = p.createMultiBody(
                baseCollisionShapeIndex=mesh_collision_shape,
                baseVisualShapeIndex=-1,
                basePosition=pos,
                baseOrientation=qtn,
            )
            self.obstacles.append(self.obstacle_id_new)
        elif obstacleName in ['3fGripper']:
            self.obstacle_id1 = p.loadURDF(self.paths[self.args.obstacle], 
                                        pos, qtn, globalScaling=scale[0])
            self.obstacle1 = obstascle3fGripper(self.obstacle_id1)
            self.obstacles.append(self.obstacle_id1)

    def add_box(self, box_pos, half_box_size, box_qtn=list(p.getQuaternionFromEuler([0,0,0]))):
        '''
        Add a box to the pybullet environment.
        '''
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_box_size)
        box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colBoxId, basePosition=box_pos, baseOrientation=box_qtn)

        self.obstacles.append(box_id)
        return box_id

    def create_ompl_interface(self):
        '''
        Create an interface class.
        '''
        self.pb_ompl_interface = PbOMPL(self.robot, self.args, self.obstacles)

    def track_path_cost(self, path):
        '''
        Calculate and record the escape energy.
        '''
        energy_along_path = [get_state_energy(state, self.args.object) for state in path]
        escape_energy = max(energy_along_path) - energy_along_path[0] # max potential gain TODO: incremental potential gain
        self.escape_energy_list.append(escape_energy)

    def execute_search(self):
        '''
        Start the OMPL planner.
        '''
        res, path, sol_path_energy, best_cost, time_taken = self.pb_ompl_interface.plan(self.goal, self.args.runtime)
        if res:
            if self.args.visualization:
                self.pb_ompl_interface.execute(path)
            if self.args.search == 'BisectionSearch':
                self.track_path_cost(path)
        else:
            if self.args.search == 'BisectionSearch':
               self.escape_energy_list.append(self.escape_energy_list[-1])
        return res, path, sol_path_energy, best_cost, time_taken

    def energy_biased_search(self, numIter=1, save_escape_path=0, get_cost_from_path=0):
        '''
        Preparation and postprocessing of the energy biased search algorithm.
        '''
        self.pb_ompl_interface.reset_robot_state_bound()
        self.sol_path_energy_list = []
        self.sol_final_costs = []
        solveds = []
        for i in range(numIter):
            self.robot.set_state(self.start)
            self.pb_ompl_interface.set_planner(self.args.planner, self.goal)
            solved, path, sol_path_energy, sol_final_cost, _ = self.execute_search()
            
            # Save escape path to csv
            if save_escape_path:
                if self.args.object in ['Chain']:
                    chainNodePath = self.get_chain_node_pos_path(path)
                    save_escape_path_4blender(self.args, chainNodePath)
                else:
                    save_escape_path_4blender(self.args, path)

            # Get escape cost from path for rigid objects
            if get_cost_from_path:
                z = [s[2] for s in path]
                self.cost_from_path = max(z) - z[0]

            # Record data
            self.sol_path_energy_list.append(sol_path_energy)      
            self.sol_final_costs.append(sol_final_cost)
            solveds.append(solved)
            # with open('./Donut60-bowlS-120s.csv', 'a', newline='') as csvfile:
            #     writer = csv.writer(csvfile)
            #     writer.writerow([sol_final_cost])

        if solveds.count(True) == 0:
            return False
        return True

    def visualize_energy_biased_search(self):
        '''
        Visualize the convergence of state energy along the esacpe path and save images with verticle rolling lines.
        '''
        # for i in range(len(self.sol_path_energy_list[0])):
        for i in range(5):
            _, ax = plt.subplots(figsize=(6,6))
            ax.plot(self.sol_path_energy_list[0],color=get_colors()[3]) # max z's along successful escape paths
            ax.axvline(x=i, color='k', linestyle='--', linewidth=1)
            ax.set_xlabel('states along the solution path')
            ax.set_ylabel('state potential energy / J')
            ax.set_xlim(0,len(self.sol_path_energy_list[0]))
            ax.grid(True)
            # plot_escape_energy_from_multi_csv(ax, folderList, isArticulatedObj, axvline=[i*xmax/(numframes-1)])
            plt.savefig("file%03d.png" % i, dpi=300)

    def energy_bisection_search(self, numIter=1, maxTimeTaken=120, epsThreshold=5e-4, useBisectionSearch=0):
        '''
        Iteratively find the (lowest) threshold of escape energy upper bound that allows an escaping path.
        '''
        initCUpper = 10
        self.time_taken_list_runs, self.escape_energy_list_runs = [], [] # record over several runs
        startEnergy = get_state_energy(self.start, self.args.object)
        for i in range(numIter):
            cUpper, cLower, cEs = initCUpper, 0, 0
            self.escape_energy_list = [] # successful escapes
            self.time_taken_list = [] # corresponding time taken
            moveOn = True

            while moveOn: 
                # Set upper bound of searching
                self.pb_ompl_interface.reset_robot_state_bound()
                self.robot.set_state(self.start)
                self.pb_ompl_interface.set_planner(self.args.planner, self.goal)
                
                # Start planning
                res, _, _, _, time_taken = self.execute_search()

                # Record time 
                accumulate_time = time_taken + self.time_taken_list[-1] if len(self.time_taken_list)>0 else time_taken
                self.time_taken_list.append(accumulate_time)

                # Update bounds
                curr_cost = self.escape_energy_list[-1]

                # Set lower and upper bounds
                if useBisectionSearch: # bisection search
                    if res: # found solution
                        cUpper = curr_cost
                        if curr_cost < cLower: # a solution lower than lower bound found
                            cLower = 0
                    else: # solution not found
                        cLower = cEs 

                    # Conditions of termination
                    eps = abs(cUpper-cLower)
                    if eps < epsThreshold or accumulate_time > maxTimeTaken:
                        moveOn = False

                    # Reset energy threshold
                    cEs = (cUpper + cLower) / 2
                    self.pb_ompl_interface.reset_bisec_energy_thres(cEs+startEnergy)

                else: # incremental search - slower but guaranteed lower bound
                    # Update upper bound
                    if res: # solution found
                        cUpper = curr_cost

                    # Termination
                    if cUpper <= 0.0 or self.time_taken_list[-1] > maxTimeTaken:
                        moveOn = False

                    # Reset energy threshold
                    self.pb_ompl_interface.reset_bisec_energy_thres(cUpper+startEnergy)

                # print("----------escape_energy_list: ", self.escape_energy_list)
                # print("----------time_taken_list: ", self.time_taken_list)
            
            # Record data over runs
            self.time_taken_list_runs.append(self.time_taken_list) 
            self.escape_energy_list_runs.append(self.escape_energy_list)

            # Reset and prepare for the next run
            self.pb_ompl_interface.reset_bisec_energy_thres(initCUpper)

    def visualize_bisection_search(self):
        '''
        Visualize the convergence of caging depth.
        '''
        _, ax1 = plt.subplots()
        for i in range(len(self.time_taken_list_runs)):
            ax1.plot(self.time_taken_list_runs[i], self.escape_energy_list_runs[i], label='Costs of escapes in run {}'.format(i))
        ax1.set_xlabel('Accumulated search time / s')
        ax1.set_ylabel('Escape cost / J')
        ax1.grid(True)
        ax1.legend()
        plt.title('Iterative bound shrinking search of caging escape energy')
        plt.show()


class ArticulatedObjectCaging(RigidObjectCaging):
    '''
    For articulated objects, such as fish.
    '''
    def __init__(self, args, objScale):
        self.args = args
        self.obstacles = []
        self.rigidObjList = get_non_articulated_objects()
        self.escape_cost_list = [] # successful escapes

        if args.visualization:
            vis = p.GUI
        else:
            vis = p.DIRECT
        p.connect(vis)     
        p.setTimeStep(1./240.)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.load_object(scale=objScale)
        self.reset_start_and_goal()


class SnapLock2DCaging(RigidObjectCaging):
    '''
    For the 2D snap lock scenario.
    '''
    def __init__(self, args, objScale, basePosBounds):
        self.args = args
        self.basePosBounds = basePosBounds
        self.obstacles = []
        self.rigidObjList = get_non_articulated_objects()
        self.escape_cost_list = [] # successful escapes

        if args.visualization:
            vis = p.GUI
        else:
            vis = p.DIRECT
        p.connect(vis)     
        p.setTimeStep(1./240.)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.load_object(scale=objScale)
        self.reset_start_and_goal()

    def load_object(self, scale=1):
        self.paths = path_collector()
        self.object_id = p.loadURDF(self.paths[self.args.object], (0,0,0), globalScaling=scale)
        self.robot = SnapLock2D(self.object_id, self.basePosBounds)

    def reset_start_and_goal(self, start=None, goal=None):
        if start is None:
            self.start = [0,0,0] + [0]*self.robot.articulate_num
        else:
            self.start = start
        if goal is None:
            self.goal = [1,0,0] + [0]*self.robot.articulate_num
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


class ElasticBandCaging(RigidObjectCaging):
    '''
    For the load-in and set-up of the elastic band represented by control points.
    '''
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
        self.paths = path_collector()
        self.object_id = []
        if self.args.object == 'Band': 
            self.robot = objectElasticBand(self.object_id, self.numCtrlPoint)
        if self.args.object == 'BandHorizon': 
            self.robot = objectBandHorizon(self.object_id, self.numCtrlPoint)

    def reset_start_and_goal(self, start=None, goal=None):
        # Set start and goal nodes of searching algorithms
        self.start = [0,0,2]*self.numCtrlPoint if start is None else start
        self.goal = [0,0,3]*self.numCtrlPoint if goal is None else goal

        return True


class MaskBandCaging(RigidObjectCaging):
    '''
    For the mask wearing scenario.
    '''
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
        self.paths = path_collector()
        self.object_id = []
        self.robot = objectMaskBand(self.object_id, self.numCtrlPoint)

    def reset_start_and_goal(self, start=None, goal=None):
        # Set start and goal nodes of searching algorithms
        self.start = [0,0,0]*self.numCtrlPoint if start is None else start
        self.goal = [0,0,0.5]*self.numCtrlPoint if goal is None else goal
  
        return True

class RopeCaging(RigidObjectCaging):
    '''
    For the load-in and set-up of a rope.
    '''
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
        self.paths = path_collector()
        self.object_id = []
        for i in range(self.numCtrlPoint+2): # number of nodes
           self.object_id.append(p.loadURDF("sphere_1cm.urdf", (0,0,0), globalScaling=.01)) # '1cm': diameter
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


class ChainCaging(RigidObjectCaging):
    '''
    For the load-in and set-up of a rope loop.
    '''
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
        self.paths = path_collector()
        self.object_id = []
        self.robot = objectChain(self.object_id, self.numCtrlPoint, self.linkLen)

    def reset_start_and_goal(self, start=None, goal=None):
        self.start = start
        self.goal = goal
        return 
    
    def get_chain_node_pos_path(self, path):
        chainNodePath = []
        for state in path:
            _, chainNodePos = get_chain_node_pos(state, self.linkLen)
            chainNodePath.append(flatten_nested_list(chainNodePos))
        return chainNodePath
try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'ompl/py-bindings'))
    # sys.path.insert(0, join(dirname(abspath(__file__)), '../whole-body-motion-planning/src/ompl/py-bindings'))
    print(sys.path)
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
import pybullet as p
import utils
import time
from itertools import product
import numpy as np
import sys
import kinpy as kp
from scipy.spatial.transform import Rotation as R
import math
import objective

INTERPOLATE_NUM = 500

class PbStateSpace(ob.RealVectorStateSpace):
    def __init__(self, num_dim) -> None:
        super().__init__(num_dim)
        self.num_dim = num_dim
        self.state_sampler = None

    def allocStateSampler(self):
        '''
        This will be called by the internal OMPL planner
        '''
        # WARN: This will cause problems if the underlying planner is multi-threaded!!!
        if self.state_sampler:
            return self.state_sampler

        # when ompl planner calls this, we will return our sampler
        return self.allocDefaultStateSampler()

    def set_state_sampler(self, state_sampler):
        '''
        Optional, Set custom state sampler.
        '''
        self.state_sampler = state_sampler



class PbOMPL():
    def __init__(self, robot, args, obstacles=[]) -> None:
        '''
        Args
            robot: A PbOMPLRobot instance.
            obstacles: list of obstacle ids. Optional.
        '''
        self.args = args
        self.robot = robot
        self.robot_id = robot.id
        self.obstacles = obstacles
        self.space = PbStateSpace(robot.num_dim)
        self.set_obstacles(self.obstacles)

        # self.reset_robot_state_bound()
  
    def set_obstacles(self, obstacles):
        self.obstacles = obstacles

        # update collision detection
        self.setup_collision_detection(self.robot, self.obstacles)

    def add_obstacles(self, obstacle_id):
        self.obstacles.append(obstacle_id)

    def is_state_valid(self, state):
        # satisfy bounds TODO
        # Should be unecessary if joint bounds is properly set

        # check self-collision
        self.robot.set_state(self.state_to_list(state))
        for link1, link2 in self.check_link_pairs:
            if utils.pairwise_link_collision(self.robot_id, link1, self.robot_id, link2):
                # print(get_body_name(body), get_link_name(body, link1), get_link_name(body, link2))
                return False

        # check collision against environment
        for body1, body2 in self.check_body_pairs:
            if utils.pairwise_collision(body1, body2):
                # print('body collision', body1, body2)
                # print(get_body_name(body1), get_body_name(body2))
                return False
        return True

    def setup_collision_detection(self, robot, obstacles, self_collisions = False):
        self.check_link_pairs = utils.get_self_link_pairs(robot.id, robot.joint_idx) if self_collisions else []
        # moving_links = frozenset(
        #     [item for item in utils.get_moving_links(robot.id, robot.joint_idx) if not item in allow_collision_links])
        moving_bodies = [robot.id] # for deformable ball
        # moving_bodies = [(robot.id, moving_links)] # original 
        # print('moving_bodies: ', moving_bodies)
        self.check_body_pairs = list(product(moving_bodies, obstacles))
      
    def reset_robot_state_bound(self):
        bounds = ob.RealVectorBounds(self.robot.num_dim)
        joint_bounds = self.robot.get_joint_bounds()
        for i, bound in enumerate(joint_bounds):
            bounds.setLow(i, bound[0])
            bounds.setHigh(i, bound[1])
        self.space.setBounds(bounds)

    def set_planner(self, planner_name, goal):
        '''
        Planner setup.
        '''
        # Setup state bounds and collision checker
        self.si = ob.SpaceInformation(self.space)
        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid))
        self.si.setup()
        # self.si.setStateValidityCheckingResolution(0.005)
        
        # Setup sampling-based planner
        if planner_name == "PRM":
            self.planner = og.PRM(self.si)
        elif planner_name == "RRT":
            self.planner = og.RRT(self.si)
        elif planner_name == "RRTConnect":
            self.planner = og.RRTConnect(self.si)
        elif planner_name == "RRTstar":
            self.planner = og.RRTstar(self.si)
        elif planner_name == "EST":
            self.planner = og.EST(self.si)
        elif planner_name == "FMT":
            self.planner = og.FMT(self.si)
        elif planner_name == "BITstar":
            self.planner = og.BITstar(self.si)
            # self.planner.params().setParam("find_approximate_solutions", "1")
            self.planner.params().setParam("samples_per_batch", "1000")
            # self.planner.params().setParam("use_just_in_time_sampling", "1")
            # self.planner.params().setParam("use_strict_queue_ordering", "0")
        elif planner_name == "ABITstar":
            self.planner = og.ABITstar(self.si)
        elif planner_name == "AITstar":
            self.planner = og.AITstar(self.si)
        elif planner_name == "SORRTstar":
            self.planner = og.SORRTstar(self.si)
        elif planner_name == "PRMstar":
            self.planner = og.PRMstar(self.si)
        else:
            print("{} not recognized, please add it first".format(planner_name))
            return

        # Set the start and goal states;
        start = self.robot.get_cur_state()

        s = ob.State(self.space)
        g = ob.State(self.space)
        for i in range(len(start)):
            s[i] = start[i]
            g[i] = goal[i]

        # Setup problem formulation
        self.pdef = ob.ProblemDefinition(self.si)
        self.pdef.setStartAndGoalStates(s, g)
     
        # Set customized optimization objective
        rigidObjs = ['Donut', 'Hook', 'Bowl']
        if self.args.search == 'EnergyMinimizeSearch':
            if self.args.object in rigidObjs: # rigid object caging
                if self.args.objective == 'GravityPotential':
                    potentialObjective = objective.minPathPotentialObjective(self.si, start)
                    self.pdef.setOptimizationObjective(potentialObjective)
                # elif self.args.objective == 'GravityAndElasticPotential': # Not applied
                # else self.args.objective == 'PathLength' # default objective

            elif self.args.objective == 'GravityAndElasticPotential' or 'GravityPotential': # articulated object caging
                potentialObjective = objective.minPathTotalPotentialObjective(self.si, start, self.args)
                self.pdef.setOptimizationObjective(potentialObjective)
            # else: self.args.objective == 'PathLength'
        
        # else self.args.search == 'BoundShrinkSearch':

        self.planner.setProblemDefinition(self.pdef)
        self.planner.setup()

    def plan_start_goal(self, goal, allowed_time=10.0, reached_thres=0.5):
        '''
        plan a path to goal from the given robot start state
        '''
        # print("start_planning")
        print(self.planner.params())

        orig_robot_state = self.robot.get_cur_state()

        # attempt to solve the problem within allowed planning time
        solved = self.planner.solve(allowed_time)
        res = False
        sol_path_list = []
        sol_path_energy, best_cost = None, None
        
        if solved:
            print("Found solution: interpolating into {} segments".format(INTERPOLATE_NUM))
            # solution_path = self.planner.bestPathFromGoalToStart() # TODO: expose the corresponding C++ protected function
            
            sol_path_geometric = self.pdef.getSolutionPath()
            sol_path_states_non_interp = sol_path_geometric.getStates()
            sol_path_geometric.interpolate(INTERPOLATE_NUM)
            sol_path_states = sol_path_geometric.getStates()
            sol_path_list_non_interp = [self.state_to_list(s) for s in sol_path_states_non_interp]
            sol_path_list = [self.state_to_list(s) for s in sol_path_states]
            
            # check if solution path is valid
            for sol_path in sol_path_list:
                self.is_state_valid(sol_path)
                
            # get cost of the solution path
            if self.args.search == 'EnergyMinimizeSearch':
                self.objective = self.pdef.getOptimizationObjective()
                sol_path_energy = [self.objective.stateEnergy(i) for i in sol_path_list_non_interp]
                best_cost = self.planner.bestCost().value() # approximate solution?
                print('!!!!!!!!bestCost', best_cost)
                
                # sol_final_cost = sol_path_geometric.cost(self.objective).value() # exact solution?
                # print('!!!!!!!!sol_final_cost', sol_final_cost)

            # make sure goal is reached
            diff = [sol_path_list[-1][i]-goal[i] for i in range(len(goal))]
            if sum(abs(i) for i in diff) < reached_thres:
                res = True
        else:
            print("No solution found")

        # reset robot state
        self.robot.set_state(orig_robot_state)

        return res, sol_path_list, sol_path_energy, best_cost

    def plan(self, goal, allowed_time=10.0):
        '''
        plan a path to goal from current robot state
        '''
        return self.plan_start_goal(goal, allowed_time)

    def execute(self, path, dynamics=False):
        '''
        Execute a planned plan. Will visualize in pybullet.
        Args:
            path: list[state], a list of state
            dynamics: allow dynamic simulation. If dynamics is false, this API will use robot.set_state(),
                      meaning that the simulator will simply reset robot's state WITHOUT any dynamics simulation. Since the
                      path is collision free, this is somewhat acceptable.
        '''
        for q in path:
            if dynamics:
                # TODO: try tune gravity
                kp = 10
                target_pos = q[:3]

                current_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
                grav = kp * (np.array(target_pos)-np.array(current_pos))
                # p.setGravity(grav[0], grav[1], grav[2])
            else:
                self.robot.set_state(q)
            p.stepSimulation()
            time.sleep(0.01)

    # -------------
    # Configurations
    # ------------

    def set_state_sampler(self, state_sampler):
        self.space.set_state_sampler(state_sampler)

    # -------------
    # Util
    # ------------

    def state_to_list(self, state):
        return [state[i] for i in range(self.robot.num_dim)]
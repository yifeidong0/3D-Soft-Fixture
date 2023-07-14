"""
Title: Pybullet - OMPL interface.
Author: Yifei Dong
Date: 14/07/2023
Description: The script provides an interface between Pybullet and OMPL, including OMPL parameter setup and planner settings.
"""
try:
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'ompl/py-bindings'))
    from ompl import base as ob
    from ompl import geometric as og
import pybullet as p
import utils
import time
from itertools import product
import sys
import objective

INTERPOLATE_NUM = 100

class PbStateSpace(ob.RealVectorStateSpace):
    def __init__(self, num_dim) -> None:
        super().__init__(num_dim)
        self.num_dim = num_dim
        self.state_sampler = None

    def allocStateSampler(self):
        """
        This will be called by the internal OMPL planner
        """
        # WARN: This will cause problems if the underlying planner is multi-threaded!!!
        if self.state_sampler:
            return self.state_sampler

        # When ompl planner calls this, we will return our sampler
        return self.allocDefaultStateSampler()

    def set_state_sampler(self, state_sampler):
        """
        Optional, Set custom state sampler.
        """
        self.state_sampler = state_sampler


class PbOMPL():
    def __init__(self, robot, args, obstacles=[]) -> None:
        self.args = args
        self.robot = robot
        self.robot_id = robot.id
        self.obstacles = obstacles
        self.state_dim = robot.num_dim
        self.space = PbStateSpace(self.state_dim)
        self.set_obstacles()
        self.use_bisection_search = 0
        
        if self.args.planner in ["BITstar", "AITstar",]:
            self.useGoalSpace = 0 # not applicable
        else:
            self.useGoalSpace = 1 # a goal region instead of a single goal configuration

        # spheres of control/node points are moved away to avoid collision while ray-casting
        self.nodeAwayPos = [10]*3
  
    def set_goal_space_bounds(self, goalSpaceBounds):
        '''
        Set the goal space region.
        '''
        self.goalSpaceBounds = goalSpaceBounds

    def set_obstacles(self):
        '''
        Update collision detection.
        '''
        self.setup_collision_detection()

    def add_obstacles(self, obstacle_id):
        '''
        Add an obstacle to the list.
        '''
        self.obstacles.append(obstacle_id)

    def reset_bisec_energy_thres(self, energy_threshold):
        '''
        Reset bisection energy threshold after one iteration.
        '''
        self.use_bisection_search = 1
        self.energy_threshold = energy_threshold

    def set_spring_params(self, springneutralLen, k):
        '''
        Set the stiffness and rest length of springs.
        '''
        self.springneutralLen = springneutralLen
        self.k_spring = k

    def is_state_valid(self, state):
        '''
        Check if a configuration (not in collision, within the search boundaries) is valid for the planner.
        '''
        # Check if the state overshoots the energy threshold in bisection search
        if self.use_bisection_search:
            current_energy = utils.get_state_energy(state, self.args.object)
            if current_energy > self.energy_threshold:
                return False

        # Check self-collision
        stateList = self.state_to_list(state)
        self.robot.set_state(stateList)
        for link1, link2 in self.check_link_pairs:
            if utils.pairwise_link_collision(self.robot_id, link1, self.robot_id, link2):
                return False
            
        # Check collision against environment
        # Scenarios with a band (control points collision check not necessary)
        if self.args.object in ["Band", "BandHorizon"]:
            if utils.band_collision_raycast(stateList, obj=self.args.object):
                return False

        elif self.args.object in ["MaskBand"]:
            if utils.mask_band_collision_raycast(stateList, self.bandFixedV0, self.bandFixedV1):
                return False
            
        elif self.args.object in ["Rope"]:
            # Check if nodes of rope in collision
            for body1, body2 in self.check_body_pairs:
                if utils.pairwise_collision(body1, body2):
                    return False
            for i in range(len(self.robot.id)):
                p.resetBasePositionAndOrientation(self.robot.id[i], self.nodeAwayPos, self.robot.zeroQuaternion)
    
            # Check if links between nodes in collision
            if utils.rope_collision_raycast(stateList, self.robot.linkLen):
                return False
            
        elif self.args.object in ["Chain"]:
            # Check if links between nodes in collision
            if utils.chain_collision_raycast(stateList, self.robot.linkLen):
                return False
        
        # Other scenarios with URDF collision bodies
        else:
            for body1, body2 in self.check_body_pairs:
                if utils.pairwise_collision(body1, body2):
                    # print('collision!')
                    return False

        return True

    def record_fixed_vertex_pos(self, bandFixedV0, bandFixedV1):
        '''
        Record the two fixed control points positions in the mask wearing scenario.
        '''
        self.bandFixedV0 = bandFixedV0
        self.bandFixedV1 = bandFixedV1

    def setup_collision_detection(self, self_collisions = False, allow_collision_links=[]):
        '''
        Check if a configuration (not in collision, within the search boundaries) is valid for the planner.
        '''
        if self.args.object in ["Rope"]:
            self.check_link_pairs = [] # do not check self-collision
            self.check_body_pairs = list(product(self.robot.id, self.obstacles))
        else:
            self.check_link_pairs = utils.get_self_link_pairs(self.robot.id, self.robot.joint_idx) if self_collisions else []
            moving_bodies = [self.robot.id] # for deformable ball
            self.check_body_pairs = list(product(moving_bodies, self.obstacles))

    def reset_robot_state_bound(self):
        '''
        Check if a configuration (not in collision, within the search boundaries) is valid for the planner.
        '''
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
        
        # Setup sampling-based planner
        if planner_name == "PRM":
            self.planner = og.PRM(self.si)
        elif planner_name == "RRT":
            self.planner = og.RRT(self.si)
        elif planner_name == "RRTConnect":
            self.planner = og.RRTConnect(self.si)
            self.planner.params().setParam("range", "0.05")
        elif planner_name == "RRTstar":
            self.planner = og.RRTstar(self.si)
            self.planner.params().setParam("range", "0.1") # controls the maximum distance between a new state and its nearest neighbor in the tree
            self.planner.params().setParam("rewire_factor", "0.01") # controls the radius of the ball used during the rewiring phase 
            self.planner.params().setParam("focus_search", "1")
        elif planner_name == "EST":
            self.planner = og.EST(self.si)
        elif planner_name == "FMT":
            self.planner = og.FMT(self.si)
        elif planner_name == "BITstar":
            self.planner = og.BITstar(self.si)
            # samples_per_batch - small value, faster initial paths, while less accurate (higher final cost)
            self.planner.params().setParam("samples_per_batch", "1000") # fish, starfish, hook
        elif planner_name == "ABITstar":
            self.planner = og.ABITstar(self.si)
        elif planner_name == "InformedRRTstar":
            self.planner = og.InformedRRTstar(self.si)
            self.planner.params().setParam("range", "0.05")
            self.planner.params().setParam("rewire_factor", "0.1") # controls the radius of the ball used during the rewiring phase 
            self.planner.params().setParam("number_sampling_attempts", "1000")
        elif planner_name == "AITstar":
            self.planner = og.AITstar(self.si)
            self.planner.params().setParam("samples_per_batch", "500") # fish, starfish, hook
        elif planner_name == "SORRTstar":
            self.planner = og.SORRTstar(self.si)
            self.planner.params().setParam("range", "0.05")
            self.planner.params().setParam("number_sampling_attempts", "1000")
            self.planner.params().setParam("rewire_factor", "0.1") # controls the radius of the ball used during the rewiring phase 
        elif planner_name == "PRMstar":
            self.planner = og.PRMstar(self.si)
        elif planner_name == "LBTRRT":
            self.planner = og.LBTRRT(self.si)
            self.planner.params().setParam("epsilon", "0.01") # A smaller value for epsilon will result in a denser tree
            self.planner.params().setParam("range", "0.1")
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

        # Set the start and goal states
        if self.useGoalSpace:
            # GoalSpace works with RRT*/PRM*/InformedRRT*, not with BIT*
            goal_space = ob.GoalSpace(self.si)
            ss = ob.RealVectorStateSpace(self.state_dim)
            bounds = ob.RealVectorBounds(self.state_dim)
            for d in range(self.state_dim):
                bounds.setLow(d, self.goalSpaceBounds[d][0])
                bounds.setHigh(d, self.goalSpaceBounds[d][1])
            ss.setBounds(bounds)
            goal_space.setSpace(ss)

            # Set start and goal
            self.pdef.addStartState(s)
            self.pdef.setGoal(goal_space)
        else:
            self.pdef.setStartAndGoalStates(s, g)
     
        # Set customized optimization objective
        rigidObjs = utils.get_non_articulated_objects()
        if self.args.search == 'EnergyBiasedSearch':
            if self.args.object in rigidObjs: # rigid object caging
                self.potentialObjective = objective.GravityPotentialObjective(self.si, start)
            elif self.args.object in ["Fish"]:
                self.potentialObjective = objective.FishPotentialObjective(self.si, start, self.args)
            elif self.args.object in ["Starfish"]:
                self.potentialObjective = objective.StarfishPotentialObjective(self.si, start, self.args)
            elif self.args.object == "Snaplock":
                self.potentialObjective = objective.SnaplockPotentialObjective(self.si, start, self.args)
            elif self.args.object in ["Band", "BandHorizon"]:
                self.potentialObjective = objective.ElasticBandPotentialObjective(self.si, start, self.args, self.springneutralLen, self.k_spring)
            elif self.args.object == "MaskBand":
                self.potentialObjective = objective.MaskBandPotentialObjective(self.si, start, self.args, self.bandFixedV0, self.bandFixedV1)
            elif self.args.object == "Rope":
                self.potentialObjective = objective.RopePotentialObjective(self.si, start, self.robot.linkLen)
            elif self.args.object == "Chain":
                self.potentialObjective = objective.ChainPotentialObjective(self.si, start, self.robot.linkLen)
            elif self.args.object == "2Dlock":
                self.potentialObjective = objective.SnapLock2DPotentialObjective(self.si, start, self.args)
            self.pdef.setOptimizationObjective(self.potentialObjective)

        self.planner.setProblemDefinition(self.pdef)
        self.planner.setup()

    def plan_start_goal(self, goal, allowed_time=10.0, reached_thres=0.5):
        '''
        Plan a path to goal from the given robot start state.
        '''
        print(self.planner.params())
        orig_robot_state = self.robot.get_cur_state()

        # Attempt to solve the problem within allowed planning time
        t0 = time.time()
        solved = self.planner.solve(allowed_time)
        time_taken = time.time() - t0
        res = False
        sol_path_list = []
        sol_path_energy, best_cost = None, None
        
        if solved:
            print("Found solution: interpolating into {} segments".format(INTERPOLATE_NUM))
            sol_path_geometric = self.pdef.getSolutionPath()
            try:
                sol_path_states_non_interp = sol_path_geometric.getStates()
            except AttributeError:
                sol_path_states_non_interp = None
            sol_path_geometric.interpolate(INTERPOLATE_NUM)
            sol_path_states = sol_path_geometric.getStates()
            sol_path_list_non_interp = [self.state_to_list(s) for s in sol_path_states_non_interp]
            sol_path_list = [self.state_to_list(s) for s in sol_path_states]

            # Get cost of the solution path
            if self.args.search == 'EnergyBiasedSearch':
                sol_path_energy = [self.potentialObjective.stateEnergy(i) for i in sol_path_list_non_interp]
                if self.args.planner in ['PRMstar', 'LBTRRT',]:
                    best_cost = float(self.planner.getBestCost()) # getBestCost returns a str
                elif self.args.planner in ['RRTConnect', 'RRT']:
                    best_cost = sol_path_geometric.cost(self.potentialObjective).value() # exact solution?
                else:
                    best_cost = self.planner.bestCost().value() # approximate solution? available for BITstar

            # Make sure goal is reached
            if self.args.planner in ['BITstar']:
                diff = [sol_path_list[-1][i]-goal[i] for i in range(len(goal))]
                if sum(abs(i) for i in diff) < reached_thres:
                    res = True
            else:
                isInsideBounds = [sol_path_list[-1][i]>self.goalSpaceBounds[i][0] and sol_path_list[-1][i]<self.goalSpaceBounds[i][1] for i in range(self.state_dim)]
                if isInsideBounds.count(False) == 0:
                    res = True
        else:
            print("No solution found")

        # Reset robot state
        self.robot.set_state(orig_robot_state)

        return res, sol_path_list, sol_path_energy, best_cost, time_taken

    def plan(self, goal, allowed_time=10.0):
        '''
        Plan a path to goal from current robot state
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
            self.robot.set_state(q)

            # Visualize linear objects using Bullet user debug lines
            if self.args.object == 'Band':
                utils.band_collision_raycast(q, visRays=1)
            elif self.args.object == 'BandHorizon':
                utils.band_collision_raycast(q, visRays=1, obj='BandHorizon')
            elif self.args.object == 'MaskBand':
                utils.mask_band_collision_raycast(q, self.bandFixedV0, self.bandFixedV1, visRays=1)
            elif self.args.object == 'Rope':
                utils.rope_collision_raycast(q, self.robot.linkLen, visRays=1)
            elif self.args.object == 'Chain':
                utils.chain_collision_raycast(q, self.robot.linkLen, visRays=1)

            p.stepSimulation()
            time.sleep(7/240)

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
"""
Title: 2D demonstration
Author: Yifei Dong
Date: 14/07/2023
Description: The script demonstrates energy-biased optimal path search in a 2D C-space.
Adapted from https://github.com/ompl/ompl/blob/main/demos/OptimalPlanning.py
"""
import sys
try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    from os.path import abspath, dirname, join
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
import argparse
import matplotlib.pyplot as plt
import numpy as np

# Hyperparameters
# rads = [[.05, .6], [.05, .3], [.05, .6]]
# centers = [[.2, 0], [.5, .6], [.85, .25]]
useGoalSpace = 0
useIncrementalCost = 0
runtime = 30.0
planner = 'BITstar'

class ValidityChecker(ob.StateValidityChecker):
    def isValid(self, state):
        bools = [self.clearance(state, rads[i], centers[i])>0.0 for i in range(len(rads))]
        return bools.count(False) == 0

    def clearance(self, state, radius=[.2,.2], center=[.5, 0]):
        x, y = state[0], state[1]
        xc, yc = center[0], center[1]
        return (x-xc)**2/radius[0]**2 + (y-yc)**2/radius[1]**2 - 1

class minPathPotentialObjective(ob.OptimizationObjective):
    def __init__(self, si, start, useIncrementalCost):
        super(minPathPotentialObjective, self).__init__(si)
        self.si_ = si
        self.start_ = start
        self.useIncrementalCost_ = useIncrementalCost

    def combineCosts(self, c1, c2):
        if self.useIncrementalCost_:
            return ob.Cost(c1.value() + c2.value())
        else:
            return ob.Cost(max(c1.value(), c2.value()))

    def motionCost(self, s1, s2):
        if self.useIncrementalCost_:
            return ob.Cost(abs(s2[1] - s1[1]))
        else:
            return ob.Cost(s2[1] - self.start_[1])

def getPotentialObjective(si, start, useIncrementalCost):
    obj = minPathPotentialObjective(si, start, useIncrementalCost)
    return obj

def getThresholdPathLengthObj(si):
    obj = ob.PathLengthOptimizationObjective(si)
    obj.setCostThreshold(ob.Cost(1.51))
    return obj

def getBalancedObjective(si, start, useIncrementalCost):
    lengthObj = ob.PathLengthOptimizationObjective(si)
    potentialObj = minPathPotentialObjective(si, start, useIncrementalCost)

    opt = ob.MultiOptimizationObjective(si)
    opt.addObjective(lengthObj, alpha)
    opt.addObjective(potentialObj, 1)
    return opt

def getPathLengthObjective(si):
    return ob.PathLengthOptimizationObjective(si)

def getPathLengthObjWithCostToGo(si):
    obj = ob.PathLengthOptimizationObjective(si)
    obj.setCostToGoHeuristic(ob.CostToGoHeuristic(ob.goalRegionCostToGo))
    return obj

def allocatePlanner(si, plannerType):
    if plannerType.lower() == "bfmtstar":
        return og.BFMT(si)
    elif plannerType.lower() == "bitstar":
        planner = og.BITstar(si)
        planner.params().setParam("rewire_factor", "0.2")
        return planner
    elif plannerType.lower() == "fmtstar":
        return og.FMT(si)
    elif plannerType.lower() == "informedrrtstar":
        planner = og.InformedRRTstar(si)
        # planner.params().setParam("range", "0.01") # controls the maximum distance between a new state and its nearest neighbor in the tree (for max potential gain)
        # planner.params().setParam("rewire_factor", "0.01") # controls the radius of the ball used during the rewiring phase (for max potential gain)
        return planner
    elif plannerType.lower() == "prmstar":
        planner = og.PRMstar(si)
        # planner = og.PRM(si)
        return planner
    elif plannerType.lower() == "rrtstar":
        planner = og.RRTstar(si)
        planner.params().setParam("range", "0.01")
        planner.params().setParam("rewire_factor", "0.01")
        return planner
    elif plannerType.lower() == "sorrtstar":
        return og.SORRTstar(si)
    elif plannerType.lower() == "lbtrrt":
        planner = og.LBTRRT(si)
        # epsilon: smaller values of epsilon tend to explore the space more thoroughly but can be slower, while larger values of epsilon tend to be faster
        planner.params().setParam("epsilon", "0.01")
        return planner
    else:
        ou.OMPL_ERROR("Planner-type is not implemented in allocation function.")

def allocateObjective(si, objectiveType, start, useIncrementalCost):
    if objectiveType.lower() == "pathpotential":
        return getPotentialObjective(si, start, useIncrementalCost)
    elif objectiveType.lower() == "pathlength":
        return getPathLengthObjWithCostToGo(si)
    elif objectiveType.lower() == "thresholdpathlength":
        return getThresholdPathLengthObj(si)
    elif objectiveType.lower() == "weightedlengthandpotential":
        return getBalancedObjective(si, start, useIncrementalCost)
    else:
        ou.OMPL_ERROR("Optimization-objective is not implemented in allocation function.")

def state_to_list(state):
    return [state[i] for i in range(2)]

def plot_ellipse(center, radius, ax):
    u = center[0]    # x-position of the center
    v = center[1]    # y-position of the center
    a = radius[0]    # radius on the x-axis
    b = radius[1]    # radius on the y-axis

    t = np.linspace(0, 2*np.pi, 100)
    ax.plot(u+a*np.cos(t) , v+b*np.sin(t), color='red')

def plot(sol_path_list):
    # Plot the solution path
    xpath = [state[0] for state in sol_path_list]
    ypath = [state[1] for state in sol_path_list]
    fig, ax = plt.subplots()
    ax.plot(xpath, ypath)

    # create a circle object
    for i in range(len(rads)):
        plot_ellipse(centers[i], rads[i], ax)

    # set axis limits and aspect ratio
    ax.set_xlim([0, 1.])
    ax.set_ylim([0, 1.])
    ax.set_aspect('equal')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_title('Escape path - objective of lowest incremental potential energy gain')
    plt.show()

def plan(runTime, plannerType, objectiveType, fname, useIncrementalCost):
    # Construct the robot state space in which we're planning. We're
    # planning in [0,1]x[0,1], a subset of R^2.
    space = ob.RealVectorStateSpace(2)

    # Set the bounds of space to be in [0,1].
    space.setBounds(0.0, 1.0)

    # Construct a space information instance for this state space
    si = ob.SpaceInformation(space)
    si.setup()

    # Set the object used to check which states in the space are valid
    validityChecker = ValidityChecker(si)
    si.setStateValidityChecker(validityChecker)
    si.setup()

    start = ob.State(space)
    start[0], start[1] = 0.0, 0.0

    goal = ob.State(space)
    goal[0], goal[1] = 1, .0

    # Energy of start and goal
    Es, Eg = start[1], goal[1]

    # Create a problem instance
    pdef = ob.ProblemDefinition(si)

    # Set the start and goal states
    if useGoalSpace:
        # GoalSpace works with RRT*/PRM*/InformedRRT*, not with BIT*
        goal_space = ob.GoalSpace(si)
        s = ob.RealVectorStateSpace(2)
        bounds = ob.RealVectorBounds(2)
        bounds.setLow(0, .8)
        bounds.setHigh(0, 1.)
        bounds.setLow(1, .0)
        bounds.setHigh(1, .1)
        s.setBounds(bounds)
        goal_space.setSpace(s)

        # set start and goal
        pdef.addStartState(start)
        # pdef.setGoalState(goal, threshold)
        pdef.setGoal(goal_space)
    else:
        threshold = .001 # TODO: does not seem to impact anything now
        pdef.setStartAndGoalStates(start, goal, threshold)

    # Create the optimization objective specified by our command-line argument.
    pdef.setOptimizationObjective(allocateObjective(si, objectiveType, start, useIncrementalCost))

    # Construct the optimal planner specified by our command line argument.
    optimizingPlanner = allocatePlanner(si, plannerType)
    print(optimizingPlanner.params())

    # Set the problem instance for our planner to solve
    optimizingPlanner.setProblemDefinition(pdef)
    optimizingPlanner.setup()

    # Attempt to solve the planning problem in the given runtime
    solved = optimizingPlanner.solve(runTime)

    if solved:
        # Output the length of the path found
        sol_path_geometric = pdef.getSolutionPath()
        objValue = sol_path_geometric.cost(pdef.getOptimizationObjective()).value()
        sumEnergyGain = (objValue - (Es-Eg)) / 2
        pathLength = sol_path_geometric.length()
        sol_path_states = sol_path_geometric.getStates()
        sol_path_list = [state_to_list(state) for state in sol_path_states]
        sol_path_ys = [state[1] for state in sol_path_states]
        pathPotentialCost = max(sol_path_ys)
        totalCost = alpha*pathLength + max(sol_path_ys)
       
        print("pathPotentialCost: ", pathPotentialCost)
        print("pathLengthCost: ", pathLength)
        print('Cost, c = alpha*pathLengthCost + pathPotentialCost: ', totalCost)
        print('{0} found solution of path length {1:.4f} with an optimization ' \
            'objective value of {2:.4f}'.format( \
            optimizingPlanner.getName(), \
            pathLength, \
            objValue))

        if fname:
            with open(fname, 'w') as outFile:
                outFile.write(pdef.getSolutionPath().printAsMatrix())
    else:
        print("No solution found.")
        return None, None, None
    
    print('===================================')
    return pathPotentialCost, pathLength, totalCost

if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Optimal motion planning demo program.')

    # Add a filename argument
    parser.add_argument('-t', '--runtime', type=float, default=runtime, help=\
        '(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.')
    parser.add_argument('-p', '--planner', default=planner, \
        choices=['LBTRRT', 'BFMTstar', 'BITstar', 'FMTstar', 'InformedRRTstar', 'PRMstar', 'RRTstar', \
        'SORRTstar'], \
        help='(Optional) Specify the optimal planner to use, defaults to RRTstar if not given.')
    parser.add_argument('-o', '--objective', default='WeightedLengthAndPotential', \
        choices=['PathPotential', 'PathLength', 'ThresholdPathLength', \
        'WeightedLengthAndPotential'], \
        help='(Optional) Specify the optimization objective, defaults to PathLength if not given.')
    parser.add_argument('-f', '--file', default=None, \
        help='(Optional) Specify an output path for the found solution path.')
    parser.add_argument('-i', '--info', type=int, default=1, choices=[0, 1, 2], \
        help='(Optional) Set the OMPL log level. 0 for WARN, 1 for INFO, 2 for DEBUG.' \
        ' Defaults to WARN.')

    # Parse the arguments
    args = parser.parse_args()

    # Check that time is positive
    if args.runtime <= 0:
        raise argparse.ArgumentTypeError(
            "argument -t/--runtime: invalid choice: %r (choose a positive number greater than 0)" \
            % (args.runtime,))

    # Set the log level
    if args.info == 0:
        ou.setLogLevel(ou.LOG_WARN)
    elif args.info == 1:
        ou.setLogLevel(ou.LOG_INFO)
    elif args.info == 2:
        ou.setLogLevel(ou.LOG_DEBUG)
    else:
        ou.OMPL_ERROR("Invalid log-level integer.")

    # Solve the planning problem
    alphas = [0.0, 1e-4, 1e-3, 1e-2, 5e-2, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0]
    numOuterLoops = len(alphas)
    numInnerLoops = 8
    normalizedTotalCostMean = []
    normalizedTotalCostStd = []
    for i in range(numOuterLoops):
        alpha = alphas[i]
        normalizedTotalCostsAlpha = []
        for j in range(numInnerLoops):
            rads = np.random.rand(3,2)
            rads[0][0] = rads[0][0]*0.08 + 0.01
            rads[0][1] = rads[0][1]*0.8 + 0.1
            rads[1][0] = rads[1][0]*0.08 + 0.01
            rads[1][1] = rads[1][1]*0.3 + 0.2
            rads[2][0] = rads[2][0]*0.08 + 0.01
            rads[2][1] = rads[2][1]*0.7 + 0.2

            centers = np.random.rand(3,2)
            centers[0][0] = centers[0][0]*0.1 + 0.15
            centers[0][1] = 0
            centers[1][0] = centers[1][0]*0.2 + 0.4
            centers[1][1] = centers[1][1]*0.3 + 0.3
            centers[2][0] = centers[2][0]*0.07 + 0.8
            centers[2][1] = centers[2][1]*0.2 + 0.0

            pathPotentialCost, pathLength, totalCost = plan(args.runtime, args.planner, args.objective, args.file, useIncrementalCost)
            if totalCost is not None:
                normalizedTotalCostsAlpha.append(totalCost/pathPotentialCost)
        mean = sum(normalizedTotalCostsAlpha) / len(normalizedTotalCostsAlpha)
        variance = sum((x - mean) ** 2 for x in normalizedTotalCostsAlpha) / len(normalizedTotalCostsAlpha)
        std = variance ** 0.5
        normalizedTotalCostMean.append(mean)
        normalizedTotalCostStd.append(std)

    # Plot alpha v.s. normalized total cost
    mean = np.asarray(normalizedTotalCostMean)
    std = np.asarray(normalizedTotalCostStd)
    print("alphas: ", alphas)
    print("mean: ", mean)
    print("std: ", std)
    plt.plot(alphas, mean, '-', color='#31a354', linewidth=2)
    plt.fill_between(alphas, mean-std, mean+std, alpha=0.4, color='#31a354')
    plt.gca().set_xscale('log')
    plt.savefig("alpha-normalized-cost.png")
    plt.show()

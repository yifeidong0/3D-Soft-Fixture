import os.path as osp
import sys
import argparse
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))
import pbOmplInterface
from rigidObjCaging import RigidObjectCaging
from articulatedObjCaging import ArticulatedObjectCaging

def argument_parser():
    '''
    Hyperparemeter setup.
    '''
    # Create an argument parser
    parser = argparse.ArgumentParser(description='3D energy-bounded caging demo program.')

    # Add a filename argument
    parser.add_argument('-s', '--search', default='BoundShrinkSearch', \
        choices=['BoundShrinkSearch', 'EnergyMinimizeSearch'], \
        help='(Optional) Specify the sampling-based search method to use, defaults to BoundShrinkSearch if not given.')
    
    parser.add_argument('-p', '--planner', default='RRT', \
        choices=['BFMTstar', 'BITstar', 'FMTstar', 'FMT', 'InformedRRTstar', 'PRMstar', 'RRTstar', \
        'SORRTstar', 'RRT'], \
        help='(Optional) Specify the optimal planner to use, defaults to RRTstar if not given.')
    
    parser.add_argument('-o', '--objective', default='GravityPotential', \
        choices=['PathLength', 'GravityPotential', 'GravityAndElasticPotential', \
        'PotentialAndPathLength'], \
        help='(Optional) Specify the optimization objective, defaults to PathLength if not given.')

    parser.add_argument('-j', '--object', default='Donut', \
        choices=['Fish', 'Humanoid', 'Donut', 'Hook', '3fGripper', 'PlanarRobot', 'PandaArm', 'Bowl'], \
        help='(Optional) Specify the object to cage.')

    parser.add_argument('-t', '--runtime', type=float, default=2.0, help=\
        '(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.')
    
    parser.add_argument('-v', '--visualization', type=bool, default=1, help=\
        '(Optional) Specify whether to visualize the pybullet GUI. Defaults to False and must be False or True.')
    
    parser.add_argument('-f', '--file', default=None, \
        help='(Optional) Specify anoutput path for the found solution path.')
    
    parser.add_argument('-i', '--info', type=int, default=0, choices=[0, 1, 2], \
        help='(Optional) Set the OMPL log level. 0 for WARN, 1 for INFO, 2 for DEBUG.' \
        ' Defaults to WARN.')

    # Parse the arguments
    args = parser.parse_args()

    return args

if __name__ == '__main__':
    args = argument_parser()

    rigidObjs = ['Donut', 'Hook', 'Bowl']
    if args.object in rigidObjs:
        eps_thres = 1e-2 # threshold of loop terminating
        env = RigidObjectCaging(args, eps_thres)
    else:
        env = ArticulatedObjectCaging(args)

    env.add_obstacles()
    env.pb_ompl_interface = pbOmplInterface.PbOMPL(env.robot, args, env.obstacles)

    # Choose different searching methods
    if args.search == 'BoundShrinkSearch':
        useBisecSearch = True # True: bisection search; False: Conservative search
        env.bound_shrink_search(useBisecSearch)
        escape_energy, z_thres = env.visualize_bound_shrink_search(useBisecSearch) # visualize
        print('final z threshold: {}, escape energy: {}'.format(z_thres, escape_energy))
    elif args.search == 'EnergyMinimizeSearch':
        numIter = 4
        env.energy_minimize_search(numIter)
        env.visualize_energy_minimize_search()

    # TODO: comapare the results with ground truth (Open3d OBB - donut model)
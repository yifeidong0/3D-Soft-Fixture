import os.path as osp
import sys
import argparse
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))
import pbOmplInterface
from rigidObjCaging import RigidObjectCaging

def argument_parser():
    '''
    Hyperparemeter setup.
    '''
    # Create an argument parser
    parser = argparse.ArgumentParser(description='3D energy-bounded caging demo program.')

    # Add a filename argument
    parser.add_argument('-s', '--search', default='OneTimeSearch', \
        choices=['BisectionSearch', 'OneTimeSearch'], \
        help='(Optional) Specify the sampling-based search method to use, defaults to BisectionSearch if not given.')
    
    parser.add_argument('-p', '--planner', default='BITstar', \
        choices=['BFMTstar', 'BITstar', 'FMTstar', 'InformedRRTstar', 'PRMstar', 'RRTstar', \
        'SORRTstar'], \
        help='(Optional) Specify the optimal planner to use, defaults to RRTstar if not given.')
    
    parser.add_argument('-o', '--objective', default='GravityPotential', \
        choices=['PathLength', 'GravityPotential', 'GravityAndElasticPotential', \
        'PotentialAndPathLengthCombo'], \
        help='(Optional) Specify the optimization objective, defaults to PathLength if not given.')

    parser.add_argument('-j', '--object', default='Donut', \
        choices=['Fish', 'Humanoid', 'Donut', 'Hook', '3fGripper', 'PlanarRobot', 'PandaArm', ''], \
        help='(Optional) Specify the object to cage.')

    parser.add_argument('-t', '--runtime', type=float, default=10.0, help=\
        '(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.')
    
    parser.add_argument('-v', '--visualization', type=bool, default=True, help=\
        '(Optional) Specify whether to visualize the pybullet GUI. Defaults to False and must be False or True.')
    
    parser.add_argument('-f', '--file', default=None, \
        help='(Optional) Specify an output path for the found solution path.')
    
    parser.add_argument('-i', '--info', type=int, default=0, choices=[0, 1, 2], \
        help='(Optional) Set the OMPL log level. 0 for WARN, 1 for INFO, 2 for DEBUG.' \
        ' Defaults to WARN.')

    # Parse the arguments
    args = parser.parse_args()

    return args

if __name__ == '__main__':
    args = argument_parser()

    env = RigidObjectCaging(args, eps_thres=1e-1)
    env.add_obstacles()
    env.pb_ompl_interface = pbOmplInterface.PbOMPL(env.robot, args, env.obstacles)

    # iterative height threshold search
    if args.search == 'BisectionSearch':
        env.bisection_search()

        # visualization
        if args.visualization == True:
            escape_energy, z_thres = env.visualize_bisec_search()
            print('final z threshold: {}, escape energy: {}'.format(z_thres, escape_energy))
    elif args.search == 'OneTimeSearch':
        env.one_time_search()
    # TODO: comapare the results with ground truth (Open3d OBB - donut model)
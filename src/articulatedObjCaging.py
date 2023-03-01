import os.path as osp
import pybullet as p
import sys
import pybullet_data
sys.path.insert(0, osp.join(osp.dirname(osp.abspath(__file__)), '../'))
import matplotlib.pyplot as plt
import numpy as np
from time import sleep
from object import ObjectToCage
from rigidObjCaging import RigidObjectCaging

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

        self.start = [0,-.5,2.5,0,0,0.78] + [0]*self.robot.articulate_num # :3 pos // 3: rot [radian]
        self.goal = [0,0,0,0,0,0] + [0]*self.robot.articulate_num

        self.max_z_escapes = [] # successful escapes

    # def forward_kinematics(self):

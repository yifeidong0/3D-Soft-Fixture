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
import numpy as np
import sys
import kinpy as kp
from scipy.spatial.transform import Rotation as R
from utils import *
import pybullet as p
import time

class RopePotentialObjective(ob.OptimizationObjective):
    def __init__(self, si, start, linkLen):
        super(RopePotentialObjective, self).__init__(si)
        self.si_ = si
        self.start_ = start
        self.linkLen_ = linkLen # fixed-length rope links

        self.incrementalCost = 1
        self.numStateSpace_ = len(start)
        self.baseDof_ = 6
        self.ctrlPointDof_ = 2
        self.numCtrlPoint_ = int((self.numStateSpace_-self.baseDof_) / self.ctrlPointDof_)
        self.ctrlPointMass_ = .1
        self.g_ = 9.81
        self.TLastRow_ = np.array([[0., 0., 0., 1.]]) # 1*4
        self.nextFPosInThisF_ = np.array([[0.], [0.], [self.linkLen_], [1.]]) # 4*1

        self.startStateEnergy = self.stateEnergy(self.start_)

    def stateEnergy(self, state):
        # Rope forward kinematics
        _, nodeZsInWorld = ropeForwardKinematics(state, self.linkLen_) # no. of zs - numCtrlPoint_+2
        
        # Get links' gravitational potential energy
        energyGravity = self.ctrlPointMass_ * self.g_ * sum(nodeZsInWorld) # sigma(m_i * g * z_i)
        
        return energyGravity
    
    def motionCost(self, state1, state2):
        # RealVectorStateInternal to list
        state1 = [state1[i] for i in range(self.numStateSpace_)]
        state2 = [state2[i] for i in range(self.numStateSpace_)]
        if self.incrementalCost:
            return ob.Cost(abs(self.stateEnergy(state2) - self.stateEnergy(state1)))
        else:
            return ob.Cost(self.stateEnergy(state2) - self.startStateEnergy)

    def combineCosts(self, cost1, cost2):
        '''
        The vertex i cost is expressed as the potential energy gain along 
        the path connecting i and v_start, and formulated as
        v_child.cost = combineCost(v_parent.cost, 
                                   motionCost(v_parent, v_child))
                     = max(v_parent.cost, v_child.energy-v_start.energy)
        '''
        if self.incrementalCost:
            return ob.Cost(cost1.value() + cost2.value())
        else:
            return ob.Cost(max(cost1.value(), cost2.value()))


class ElasticBandPotentialObjective(ob.OptimizationObjective):
    def __init__(self, si, start, args):
        super(ElasticBandPotentialObjective, self).__init__(si)
        self.si_ = si
        self.start_ = start
        self.args_ = args

        # parameters of articulated object
        self.numStateSpace = len(start)
        self.numCtrlPoint = int(self.numStateSpace/3)
        self.stiffnesss = [10] * self.numCtrlPoint
        self.springneutralLen = .8
        
        # calculated energy of initial pose
        self.energyStart = self.stateEnergy(self.start_)

    def getElasticEnergy(self, state):
        # Retrieve control point positions
        ctrlPointPos = []
        for i in range(self.numCtrlPoint):
            ctrlPointPos.append(np.asarray(state[3*i:3*i+3]))

        # Retrieve displacement of control points
        springDisplaces = []
        for i in range(self.numCtrlPoint):
            springDisp = max(np.linalg.norm(ctrlPointPos[i]-ctrlPointPos[i-1])-self.springneutralLen, 0.0)
            springDisplaces.append(springDisp)

        # Get elastic potential energy
        jointEnergies = [self.stiffnesss[i] * springDisplaces[i]**2 for i in range(self.numCtrlPoint)]
        energyElastic = 0.5 * sum(jointEnergies) # sigma(.5 * k_i * q_i^2)

        return energyElastic

    def stateEnergy(self, state):
        return self.getElasticEnergy(state)
    
    def motionCost(self, state1, state2):
        state2 = [state2[i] for i in range(self.numStateSpace)] # RealVectorStateInternal to list
        energyState2 = self.stateEnergy(state2)
        return ob.Cost(energyState2 - self.energyStart)

    def combineCosts(self, cost1, cost2):
        return ob.Cost(max(cost1.value(), cost2.value()))
    

class GravityPotentialObjective(ob.OptimizationObjective):
    def __init__(self, si, start):
        super(GravityPotentialObjective, self).__init__(si)
        self.si_ = si
        self.start_ = start
        self.startStateEnergy = self.stateEnergy(self.start_)
        self.incrementalCost = 1

    def stateEnergy(self, state):
        return state[2]
    
    def motionCost(self, state1, state2):
        # return ob.Cost(self.stateEnergy(state2) - self.startStateEnergy)
        if self.incrementalCost:
            return ob.Cost(abs(self.stateEnergy(state2) - self.stateEnergy(state1)))
        else:
            return ob.Cost(self.stateEnergy(state2) - self.startStateEnergy)
        
    def combineCosts(self, cost1, cost2):
        # return ob.Cost(max(cost1.value(), cost2.value()))
        if self.incrementalCost:
            return ob.Cost(cost1.value() + cost2.value())
        else:
            return ob.Cost(max(cost1.value(), cost2.value()))

class TotalPotentialObjective(ob.OptimizationObjective):
    def __init__(self, si, start, args):
        super(TotalPotentialObjective, self).__init__(si)
        self.si_ = si
        self.start_ = start
        self.args_ = args

        # parameters of articulated object
        self.incrementalCost = 0
        self.numStateSpace = len(start)
        self.comDof = 6
        self.numJoints = self.numStateSpace - self.comDof
        self.numLinks = self.numJoints + 1
        self.g = 9.81
        # TODO:
        self.masses = [.1] * self.numLinks
        self.stiffnesss = [10] * self.numJoints
        self.o = np.array([1.])
        self.path = path_collector()
        self.chain = kp.build_chain_from_urdf(open(self.path[self.args_.object]).read())
        
        # calculated energy of initial pose
        self.energyStart = self.stateEnergy(self.start_)

    def getElasticEnergy(self, state):
        # read joint stiffnesses
        jointAngles = state[self.comDof:]

        # get elastic potential energy
        jointEnergies = [self.stiffnesss[i] * jointAngles[i]**2 for i in range(self.numJoints)]
        energyElastic = 0.5 * sum(jointEnergies) # sigma(.5 * k_i * q_i^2)

        return energyElastic

    def getGravityEnergy(self, state):
        # extract positions of links
        jointAngles = state[self.comDof:] # rad
        linkPosesInBase = self.chain.forward_kinematics(jointAngles) # dictionary

        # get object's base transform
        basePositionInWorld = state[0:3]
        baseEulInWorld = state[3:self.comDof] # euler
        baseQuatInWorld = p.getQuaternionFromEuler(baseEulInWorld)
        # r = R.from_euler('zyx', baseEulInWorld, degrees=False)
        r = R.from_quat(baseQuatInWorld) # BUG: quat to euler translation causes mistakes!
        mat = r.as_matrix() # 3*3
        thirdRow = (mat[2,:].reshape((1,3)), np.array(basePositionInWorld[2]).reshape((1,1)))
        baseTInWorld = np.hstack(thirdRow) # 1*4. 3rd row of Transform matrix
        # baseTransformInWorld = np.vstack(baseTInWorld, np.array([.0, .0, .0, 1.0])) # 4*4

        # get link heights in World frame
        linkPosesInBase = list(linkPosesInBase.values()) # list of kinpy.Transforms
        linkPositionsInBase = [np.array(np.concatenate((i.pos,self.o))).reshape((4,1)) for i in linkPosesInBase]
        linkZsInWorld = [float(baseTInWorld @ j) for j in linkPositionsInBase] # list of links' heights
        
        # get links' gravitational potential energy
        linkEnergies = [linkZsInWorld[i] * self.masses[i] for i in range(self.numLinks)]
        energyGravity = self.g * sum(linkEnergies) # sigma(g * m_i * z_i)
        
        return energyGravity
      
    def stateEnergy(self, state):
        energyElastic = self.getElasticEnergy(state)
        energyGravity = self.getGravityEnergy(state)
        energySum = energyElastic + energyGravity
        return energySum
    
    def motionCost(self, state1, state2):
        state1 = [state1[i] for i in range(self.numStateSpace)] # RealVectorStateInternal to list
        state2 = [state2[i] for i in range(self.numStateSpace)]
        # energyState2 = self.stateEnergy(state2)
        # return ob.Cost(energyState2 - self.energyStart)

        if self.incrementalCost:
            return ob.Cost(abs(self.stateEnergy(state2) - self.stateEnergy(state1)))
        else:
            return ob.Cost(self.stateEnergy(state2) - self.energyStart)
        
    def combineCosts(self, cost1, cost2):
        # return ob.Cost(max(cost1.value(), cost2.value()))
        if self.incrementalCost:
            return ob.Cost(cost1.value() + cost2.value())
        else:
            return ob.Cost(max(cost1.value(), cost2.value()))

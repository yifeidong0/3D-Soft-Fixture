"""
Title: search objectives of asymptotically optimal planners for the scenarios.
Author: Yifei Dong
Date: 14/07/2023
Description: Set up the motion cost and combine cost of various scenarios.
"""

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
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
import numpy as np
import sys
import kinpy as kp
from scipy.spatial.transform import Rotation as R
from utils import *
import pybullet as p


class FishPotentialObjective(ob.OptimizationObjective):
    '''
    For the scooping fish scenario. Gravitational and elastic potential energy are considered for 
    the 15 DoF object with 9 articulated joints.
    '''
    def __init__(self, si, start, args):
        super(FishPotentialObjective, self).__init__(si)
        self.si_ = si
        self.start_ = start
        self.args_ = args
        self.incrementalCost = 0
        self.numStateSpace = len(start)
        self.comDof = 6
        self.numJoints = self.numStateSpace - self.comDof
        self.numLinks = self.numJoints + 1
        self.g = 9.81
        self.masses = [.1] * self.numLinks
        self.stiffnesss = [10] * self.numJoints
        self.o = np.array([1.])
        self.path = path_collector()
        self.chain = kp.build_chain_from_urdf(open(self.path[self.args_.object]).read())
        
        # Calculated energy of initial pose
        self.energyStart = self.stateEnergy(self.start_)

    def getElasticEnergy(self, state):
        # Read joint stiffnesses
        jointAngles = state[self.comDof:]

        # Get elastic potential energy
        jointEnergies = [self.stiffnesss[i] * jointAngles[i]**2 for i in range(self.numJoints)]
        energyElastic = 0.5 * sum(jointEnergies) # sigma(.5 * k_i * q_i^2)

        return energyElastic

    def getGravityEnergy(self, state):
        # Extract positions of links
        jointAngles = state[self.comDof:] # rad
        linkPosesInBase = self.chain.forward_kinematics(jointAngles) # dictionary

        # Get object's base transform
        basePositionInWorld = state[0:3]
        baseEulInWorld = state[3:self.comDof] # euler
        baseQuatInWorld = p.getQuaternionFromEuler(baseEulInWorld)
        r = R.from_quat(baseQuatInWorld) # BUG: quat to euler translation causes mistakes!
        mat = r.as_matrix() # 3*3
        thirdRow = (mat[2,:].reshape((1,3)), np.array(basePositionInWorld[2]).reshape((1,1)))
        baseTInWorld = np.hstack(thirdRow) # 1*4. 3rd row of Transform matrix

        # Get link heights in World frame
        linkPosesInBase = list(linkPosesInBase.values()) # list of kinpy.Transforms
        linkPositionsInBase = [np.array(np.concatenate((i.pos,self.o))).reshape((4,1)) for i in linkPosesInBase]
        linkZsInWorld = [float(baseTInWorld @ j) for j in linkPositionsInBase] # list of links' heights
        
        # Get links' gravitational potential energy
        linkEnergies = [linkZsInWorld[i] * self.masses[i] for i in range(self.numLinks)]
        energyGravity = self.g * sum(linkEnergies) # sigma(g * m_i * z_i)
        
        return energyGravity
      
    def stateEnergy(self, state):
        energyElastic = self.getElasticEnergy(state)
        energyGravity = self.getGravityEnergy(state)
        energySum = energyElastic + energyGravity
        return energySum
    
    def motionCost(self, state1, state2):
        state1 = [state1[i] for i in range(self.numStateSpace)]
        state2 = [state2[i] for i in range(self.numStateSpace)]

        if self.incrementalCost:
            return ob.Cost(abs(self.stateEnergy(state2) - self.stateEnergy(state1)))
        else:
            return ob.Cost(self.stateEnergy(state2) - self.energyStart)
        
    def combineCosts(self, cost1, cost2):
        if self.incrementalCost:
            return ob.Cost(cost1.value() + cost2.value())
        else:
            return ob.Cost(max(cost1.value(), cost2.value()))


class MaskBandPotentialObjective(ob.OptimizationObjective):
    '''
    For the wearing mask scenario. Elastic potential energy is considered for 
    the 18 DoF object with 6 control points along the open band loop.
    '''
    def __init__(self, si, start, args, bandFixedV0, bandFixedV1):
        super(MaskBandPotentialObjective, self).__init__(si)
        self.si_ = si
        self.start_ = start
        self.args_ = args
        self.bandFixedV0 = bandFixedV0
        self.bandFixedV1 = bandFixedV1
        self.incrementalCost = 0

        # Parameters of articulated object
        self.numStateSpace = len(start)
        self.numCtrlPoint = int(self.numStateSpace/3) # 6
        self.stiffnesss = [1.6] * (self.numCtrlPoint+1)
        
        # length (data id): (90), 140, 145, 150, 160, 170, 180, (232)
        # 10 (data id) - 0.1m
        self.springneutralLen = [.5, .05, .05, .1, .1, .1, .52] # list(7)
        
        # Calculated energy of initial pose
        self.energyStart = self.stateEnergy(self.start_)

    def getElasticEnergy(self, state):
        # Retrieve control point positions
        ctrlPointPos = []
        for i in range(self.numCtrlPoint):
            ctrlPointPos.append(np.asarray(state[3*i:3*i+3]))
        ctrlPointPos.insert(0, self.bandFixedV0)
        ctrlPointPos.append(self.bandFixedV1)

        # Retrieve displacement of control points
        springDisplaces = []
        for i in range(1, self.numCtrlPoint+2):
            springDisp = max(np.linalg.norm(ctrlPointPos[i]-ctrlPointPos[i-1])-self.springneutralLen[i-1], 0.0)
            springDisplaces.append(springDisp)

        # Get elastic potential energy
        jointEnergies = [self.stiffnesss[i] * springDisplaces[i]**2 for i in range(self.numCtrlPoint)]
        energyElastic = 0.5 * sum(jointEnergies) # sigma(.5 * k_i * q_i^2)

        return energyElastic

    def stateEnergy(self, state):
        return self.getElasticEnergy(state)
    
    def motionCost(self, state1, state2):
        state2 = [state2[i] for i in range(self.numStateSpace)]
        if self.incrementalCost:
            state1 = [state1[i] for i in range(self.numStateSpace)]
            return ob.Cost(abs(self.stateEnergy(state2) - self.stateEnergy(state1)))
        else:
            return ob.Cost(self.stateEnergy(state2) - self.energyStart)

    def combineCosts(self, cost1, cost2):
        if self.incrementalCost:
            return ob.Cost(cost1.value() + cost2.value())
        else:
            return ob.Cost(max(cost1.value(), cost2.value()))
    

class StarfishPotentialObjective(ob.OptimizationObjective):
    '''
    For the catching starfish scenario. Gravitational potential energy is considered for 
    the 16 DoF object with 10 articulated joints.
    '''
    def __init__(self, si, start, args):
        super(StarfishPotentialObjective, self).__init__(si)
        self.si_ = si
        self.start_ = start
        self.args_ = args

        # Parameters of articulated object
        self.incrementalCost = 0
        self.numStateSpace = len(start)
        self.comDof = 6
        self.numJoints = self.numStateSpace - self.comDof
        self.numLinks = self.numJoints + 1
        self.numArms = 5
        self.g = 9.81
        self.sizeScale = 1/15
        self.masses = [.3,.1,.1,.1,.1,.1]
        self.masses = [x*self.sizeScale for x in self.masses]
        self.o = np.array([1.])
        self.path = path_collector()
        self.chain = kp.build_chain_from_urdf(open(self.path[self.args_.object]).read())
        
        # Calculated energy of initial pose
        self.energyStart = self.stateEnergy(self.start_)

    def getGravityEnergy(self, state):
        # Extract positions of links
        jointAngles = state[self.comDof:] # rad
        linkPosesInBase = self.chain.forward_kinematics(jointAngles) # dictionary

        # Get object's base transform
        basePositionInWorld = state[0:3]
        baseEulInWorld = state[3:self.comDof] # euler
        baseQuatInWorld = p.getQuaternionFromEuler(baseEulInWorld)
        r = R.from_quat(baseQuatInWorld) # BUG: quat to euler translation causes mistakes!
        mat = r.as_matrix() # 3*3
        thirdRow = (mat[2,:].reshape((1,3)), np.array(basePositionInWorld[2]).reshape((1,1)))
        baseTInWorld = np.hstack(thirdRow) # 1*4. 3rd row of Transform matrix

        # Get link heights in World frame
        linkPosesInBase = list(linkPosesInBase.values()) # list of kinpy.Transforms
        linkPositionsInBase = [np.array(np.concatenate((i.pos,self.o))).reshape((4,1)) for i in linkPosesInBase]
        linkZsInWorld = [float(baseTInWorld @ j) for j in linkPositionsInBase] # list of links' heights, 11 links
        armZsInWorld = [2*(linkZsInWorld[2*i+1]-linkZsInWorld[0])+linkZsInWorld[0] for i in range(self.numArms)]

        # Get links' gravitational potential energy
        linkEnergies = [armZsInWorld[i] * self.masses[i+1] for i in range(self.numArms)] # mh of starfish arms
        linkEnergies.append(linkZsInWorld[0]*self.masses[0]) # mh of starfish centrum
        energyGravity = self.g * sum(linkEnergies) # sigma(g * m_i * z_i)
        
        return energyGravity
      
    def stateEnergy(self, state):
        energyGravity = self.getGravityEnergy(state)
        return energyGravity
    
    def motionCost(self, state1, state2):
        state1 = [state1[i] for i in range(self.numStateSpace)]
        state2 = [state2[i] for i in range(self.numStateSpace)]

        if self.incrementalCost:
            return ob.Cost(abs(self.stateEnergy(state2) - self.stateEnergy(state1)))
        else:
            return ob.Cost(self.stateEnergy(state2) - self.energyStart)
        
    def combineCosts(self, cost1, cost2):
        if self.incrementalCost:
            return ob.Cost(cost1.value() + cost2.value())
        else:
            return ob.Cost(max(cost1.value(), cost2.value()))


class GravityPotentialObjective(ob.OptimizationObjective):
    '''
    For all rigid objects. Gravitational potential energy is considered for 
    the 6 DoF object with 0 articulated joints.
    '''
    def __init__(self, si, start):
        super(GravityPotentialObjective, self).__init__(si)
        self.si_ = si
        self.start_ = start
        self.incrementalCost = 0
        self.mg = 1
        self.startStateEnergy = self.stateEnergy(self.start_)

    def stateEnergy(self, state):
        return self.mg * state[2]
    
    def motionCost(self, state1, state2):
        if self.incrementalCost:
            return ob.Cost(abs(self.stateEnergy(state2) - self.stateEnergy(state1)))
        else:
            return ob.Cost(self.stateEnergy(state2) - self.startStateEnergy)
        
    def combineCosts(self, cost1, cost2):
        if self.incrementalCost:
            return ob.Cost(cost1.value() + cost2.value())
        else:
            return ob.Cost(max(cost1.value(), cost2.value()))


class ChainPotentialObjective(ob.OptimizationObjective):
    '''
    For the grabbing rope loop scenario. Gravitational potential energy of the hanging bags is considered for 
    the 15 DoF object with 6 control points along the rope loop.
    '''
    def __init__(self, si, start, linkLen):
        super(ChainPotentialObjective, self).__init__(si)
        self.si_ = si
        self.start_ = start
        self.linkLen_ = linkLen

        self.incrementalCost = 0
        self.numStateSpace_ = len(start)
        self.baseDof_ = 6
        self.ctrlPointDof_ = 2
        self.numCtrlPoint_ = int((self.numStateSpace_-self.baseDof_-1) / self.ctrlPointDof_)
        self.handBagMass_ = 0.04
        self.g_ = 9.81
        self.bagHalfDiag = 1

        self.startStateEnergy = self.stateEnergy(self.start_)

    def stateEnergy(self, state):
        energyGravity = self.handBagMass_ * self.g_ * (state[2]-self.bagHalfDiag)
        return energyGravity
    
    def motionCost(self, state1, state2):
        # RealVectorStateInternal to list
        state2 = [state2[i] for i in range(self.numStateSpace_)]
        if self.incrementalCost:
            state1 = [state1[i] for i in range(self.numStateSpace_)]
            return ob.Cost(abs(self.stateEnergy(state2) - self.stateEnergy(state1)))
        else:
            return ob.Cost(self.stateEnergy(state2) - self.startStateEnergy)

    def combineCosts(self, cost1, cost2):
        if self.incrementalCost:
            return ob.Cost(cost1.value() + cost2.value())
        else:
            return ob.Cost(max(cost1.value(), cost2.value()))
    

class SnaplockPotentialObjective(ob.OptimizationObjective):
    '''
    For the 3D snap lock scenario. Gravitational and elastic potential energy are considered for 
    the 7 DoF object.
    '''
    def __init__(self, si, start, args):
        super(SnaplockPotentialObjective, self).__init__(si)
        self.si_ = si
        self.start_ = start
        self.args_ = args

        # Parameters of articulated object
        self.incrementalCost = 0
        self.numStateSpace = len(start)
        self.comDof = 6
        self.numJoints = self.numStateSpace - self.comDof
        self.mg = 1
        self.stiffnesss = [10] * self.numJoints
        self.path = path_collector()
        
        # Calculated energy of initial pose
        self.energyStart = self.stateEnergy(self.start_)

    def getElasticEnergy(self, state):
        # Read joint stiffnesses
        jointAngles = state[self.comDof:]

        # Get elastic potential energy
        jointEnergies = [self.stiffnesss[i] * jointAngles[i]**2 for i in range(self.numJoints)]
        energyElastic = 0.5 * sum(jointEnergies) # sigma(.5 * k_i * q_i^2)

        return energyElastic

    def getGravityEnergy(self, state):
        return self.mg * state[2]
      
    def stateEnergy(self, state):
        energyElastic = self.getElasticEnergy(state)
        energyGravity = self.getGravityEnergy(state)
        energySum = energyElastic + energyGravity
        return energySum
    
    def motionCost(self, state1, state2):
        state2 = [state2[i] for i in range(self.numStateSpace)] # RealVectorStateInternal to list

        if self.incrementalCost:
            state1 = [state1[i] for i in range(self.numStateSpace)]
            return ob.Cost(abs(self.stateEnergy(state2) - self.stateEnergy(state1)))
        else:
            return ob.Cost(self.stateEnergy(state2) - self.energyStart)
        
    def combineCosts(self, cost1, cost2):
        if self.incrementalCost:
            return ob.Cost(cost1.value() + cost2.value())
        else:
            return ob.Cost(max(cost1.value(), cost2.value()))
        

class SnapLock2DPotentialObjective(ob.OptimizationObjective):
    '''
    For the 2D snap lock scenario. elastic potential energy is considered for 
    the 4 DoF object.
    '''
    def __init__(self, si, start, args):
        super(SnapLock2DPotentialObjective, self).__init__(si)
        self.si_ = si
        self.start_ = start
        self.args_ = args

        # Parameters of articulated object
        self.incrementalCost = 0
        self.numStateSpace = len(start)
        self.comDof = 3
        self.numJoints = self.numStateSpace - self.comDof
        self.stiffnesss = [10] * self.numJoints
        self.restAngle = -0.36
        self.path = path_collector()
        
        # Calculated energy of initial pose
        self.energyStart = self.stateEnergy(self.start_)

    def getElasticEnergy(self, state):
        # Read joint stiffnesses
        jointAngles = state[self.comDof:]

        # Get elastic potential energy
        jointEnergies = [self.stiffnesss[i] * (jointAngles[i]-self.restAngle)**2 for i in range(self.numJoints)]
        energyElastic = 0.5 * sum(jointEnergies) # sigma(.5 * k_i * q_i^2)

        return energyElastic
      
    def stateEnergy(self, state):
        energyElastic = self.getElasticEnergy(state)
        return energyElastic
    
    def motionCost(self, state1, state2):
        state2 = [state2[i] for i in range(self.numStateSpace)]

        if self.incrementalCost:
            state1 = [state1[i] for i in range(self.numStateSpace)]
            return ob.Cost(abs(self.stateEnergy(state2) - self.stateEnergy(state1)))
        else:
            return ob.Cost(self.stateEnergy(state2) - self.energyStart)
        
    def combineCosts(self, cost1, cost2):
        if self.incrementalCost:
            return ob.Cost(cost1.value() + cost2.value())
        else:
            return ob.Cost(max(cost1.value(), cost2.value()))


class ElasticBandPotentialObjective(ob.OptimizationObjective):
    '''
    For the hourglass scenario (quantative analysis). Elastic potential energy is considered for 
    the 18 DoF object with 6 control points along the band loop.
    '''
    def __init__(self, si, start, args, springneutralLen=.1, k=1):
        super(ElasticBandPotentialObjective, self).__init__(si)
        self.si_ = si
        self.start_ = start
        self.args_ = args
        self.incrementalCost = 0

        # Parameters of articulated object
        self.numStateSpace = len(start)
        if args.object == 'Band':
            self.numCtrlPoint = int(len(start)/3)
        if args.object == 'BandHorizon':
            self.numCtrlPoint = int((len(start)-1)/2)

        self.springneutralLen = springneutralLen
        self.k = k
        self.stiffnesss = [self.k] * self.numCtrlPoint
        
        # Calculated energy of initial pose
        self.energyStart = self.stateEnergy(self.start_)

    def getElasticEnergy(self, state):
        # Retrieve control point positions
        if self.args_.object == 'Band':
            ctrlPointPos = [np.asarray(state[3*i:3*i+3]) for i in range(self.numCtrlPoint)]
        if self.args_.object == 'BandHorizon':
            ctrlPointPos = [state[2*i:2*i+2] for i in range(self.numCtrlPoint)]
            ctrlPointPos = [np.asarray(r+[state[-1]]) for r in ctrlPointPos]

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
        state2 = [state2[i] for i in range(self.numStateSpace)]
        if self.incrementalCost:
            state1 = [state1[i] for i in range(self.numStateSpace)]
            return ob.Cost(abs(self.stateEnergy(state2) - self.stateEnergy(state1)))
        else:
            return ob.Cost(self.stateEnergy(state2) - self.energyStart)

    def combineCosts(self, cost1, cost2):
        if self.incrementalCost:
            return ob.Cost(cost1.value() + cost2.value())
        else:
            return ob.Cost(max(cost1.value(), cost2.value()))


class RopePotentialObjective(ob.OptimizationObjective):
    '''
    For the rope stripe scenario. Gravitational potential energy of the hanging bags is considered for 
    the object.
    '''
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
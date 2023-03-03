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

class minPathPotentialObjective(ob.OptimizationObjective):
    def __init__(self, si, start):
        super(minPathPotentialObjective, self).__init__(si)
        self.si_ = si
        self.start_ = start
        self.startStateEnergy = self.stateEnergy(self.start_)

    def stateEnergy(self, state):
        return state[2]
    
    def motionCost(self, s1, s2):
        return ob.Cost(self.stateEnergy(s2) - self.startStateEnergy)

    def combineCosts(self, c1, c2):
        '''
        The vertex i cost is expressed as the potential energy gain along 
        the path connecting i and v_start, and formulated as
        v_child.cost = combineCost(v_parent.cost, 
                                   motionCost(v_parent, v_child))
                     = max(v_parent.cost, v_child.energy-v_start.energy)
        '''
        return ob.Cost(max(c1.value(), c2.value()))


class minPathTotalPotentialObjective(ob.OptimizationObjective):
    def __init__(self, si, start):
        super(minPathTotalPotentialObjective, self).__init__(si)
        self.si_ = si
        self.start_ = start
        
        # parameters of articulated object
        self.numStateSpace = len(start)
        self.comDof = 6
        self.numJoints = self.numStateSpace - self.comDof
        self.numLinks = self.numJoints + 1
        self.g = 9.81
        # TODO:
        self.masses = [.1] * self.numLinks
        self.stiffnesss = [.0] * self.numJoints
        self.o = np.array([1.])
        self.chain = kp.build_chain_from_urdf(open("models/articulate_fish.xacro").read())
        
        # calculated energy of initial pose
        self.energyStart = self.stateEnergy(self.start_)
        print('!!!!!self.energyStart: {}'.format(self.energyStart))

    def getElasticEnergy(self, state):
        # read joint stiffnesses
        jointAngles = state[self.comDof:]

        # get elastic potential energy
        jointEnergies = [self.stiffnesss[i] * jointAngles[i]**2 for i in range(self.numJoints)]
        energyElastic = 0.5*sum(jointEnergies) # sigma(.5 * k_i * q_i^2)

        return energyElastic

    def getGravityEnergy(self, state):
        # extract positions of links
        # jnames = chain.get_joint_parameter_names()
        jointAngles = state[self.comDof:] # rad
        linkPosesInBase = self.chain.forward_kinematics(jointAngles) # dictionary

        # get object's base transform
        basePositionInWorld = state[0:3]
        baseOrientationInWorld = state[3:self.comDof] # euler
        r = R.from_euler('zyx', baseOrientationInWorld, degrees=False)
        mat = r.as_matrix() # 3*3
        thirdRow = (mat[2,:].reshape((1,3)), np.array(basePositionInWorld[2]).reshape((1,1)))
        baseTInWorld = np.hstack(thirdRow) # 1*4. 3rd row of Transform matrix
        # baseTransformInWorld = np.vstack(baseTInWorld, np.array([.0, .0, .0, 1.0])) # 4*4

        # get link heights in World frame
        linkPosesInBase = list(linkPosesInBase.values()) # list of kinpy.Transforms
        linkPositionsInBase = [np.array(np.concatenate((i.pos,self.o))).reshape((4,1)) for i in linkPosesInBase]
        linkZsInWorld = [float(baseTInWorld @ j) for j in linkPositionsInBase] # list of links' heights
        # if [self.start_[i] == state[i] for i in range(len(state))].count(True) == len(state):
        #     print('!!!!!linkZsInWorld: {}'.format(linkZsInWorld))

        # get links' gravitational potential energy
        linkEnergies = [linkZsInWorld[i] * self.masses[i] for i in range(self.numLinks)]
        energyGravity = 0.5 * self.g * sum(linkEnergies) # sigma(.5 * g * m_i * z_i)
        
        return energyGravity
      
    def stateEnergy(self, state):
        energyElastic = self.getElasticEnergy(state)
        energyGravity = self.getGravityEnergy(state)
        energySum = energyElastic + energyGravity
        # print('!!!!!energyElastic: {}, energyGravity: {}'.format(energyElastic, energyGravity))
        return energySum
    
    def motionCost(self, state1, state2):
        state2 = [state2[i] for i in range(self.numStateSpace)] # RealVectorStateInternal to list
        energyState2 = self.stateEnergy(state2)
        return ob.Cost(energyState2 - self.energyStart)

    def combineCosts(self, cost1, cost2):
        return ob.Cost(max(cost1.value(), cost2.value()))
    
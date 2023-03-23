from visualization import *
# from datetime import datetime
import os
import subprocess
import glob
# import csv
import numpy as np
import matplotlib.pyplot as plt
from utils import *
from main import argument_parser
from runScenario import runScenario
import pybullet as p
import pybullet_data
import time

class generateVideo(runScenario):
    def __init__(self, args, path, isArticulatedObj):
        p.connect(p.GUI)
        p.setTimeStep(1./240.)
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setRealTimeSimulation(0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # planeId = p.loadURDF("plane.urdf", [0,0,-1])
        # self.paths = path_collector()
        self.args = args
        self.path = path # results path
        self.isArticulatedObj = isArticulatedObj
        self.gravity = -9.81
        self.endFrame = 800
        self.img_height = 512
        self.img_width = 512

        # pybullet camera matrices param
        fov = 60
        aspect = self.img_width / self.img_height
        near = 0.02
        far = 1

        self.viewMat = p.computeViewMatrix([0, 0, 0.5], [0, 0, 0], [1, 0, 0])
        self.projMat = p.computeProjectionMatrixFOV(fov, aspect, near, far)

        # load object and obstacle
        self.paths = path_collector()
        self.initializeParams()
        self.loadObject()
        self.loadObstacle()
        
        # get results
        self.energy_data, self.indices = get_results_from_csv(self.path, self.isArticulatedObj)
        self.startEnergySce, self.startGEnergySce, self.startEEnergySce, self.escapeEnergyCostSce = self.energy_data

    def runClenchFist(self):
        '''For the task of gripper clenching starfish'''
        i = 0 # sim no.
        k = 0 # results data no.
        lenRes = len(self.indices)
        e_total, e_escape = None, None
        # time.sleep(1)
        self.obstaclePose = self.obstaclePos + self.obstacleEul

        # set initial joint states
        jointPositions,_,_ = self.getJointStates(self.obstacleId) # list(12)
        obstacleJointPos = [jointPositions[i]-450/1000 if (i==1 or i==5 or i==9) else jointPositions[i] for i in range(len(jointPositions))]
        obstacleState = self.obstaclePose + obstacleJointPos
        self.obstacle.set_state(obstacleState)
        
        # start simulation of clenching the fist
        time.sleep(5)
        while (1):
            p.stepSimulation()
            p.setGravity(0, 0, self.gravity)
            time.sleep(1/240.)
            
            # get obstacle joint positions and update them
            jointPositions,_,_ = self.getJointStates(self.obstacleId) # list(12)
            # obstacleJointPos = [j+1/1000 for j in jointPositions]
            obstacleJointPos = [jointPositions[i]+.8/1000 for i in range(len(jointPositions))]
            obstacleState = self.obstaclePose + obstacleJointPos
            self.obstacle.set_state(obstacleState)

            # update energy texts
            if i == self.indices[k]:
                e_total = np.round(self.startEnergySce[k],decimals=3)
                e_escape = np.round(self.escapeEnergyCostSce[k],decimals=3)
                k += 1
            
            # get RGB of camera view
            images = p.getCameraImage(self.img_width, 
                                      self.img_height,
                                    #   viewMatrix=self.viewMat,
                                    #   projectionMatrix=self.projMat,
                                      renderer=p.ER_TINY_RENDERER,
                                      shadow=1,
                                    #   flags=p.ER_USE_PROJECTIVE_TEXTURE,
                                    #   projectiveTextureView=self.viewMat,
                                    #   projectiveTextureProj=self.projMat
                                    )
            rgb_array = np.reshape(images[2], (self.img_height, self.img_width, 4)) * 1. / 255.            

            # plot
            energy_tuple = (None, None, e_total, e_escape)
            self.saveIntermImages(rgb_array, energy_tuple, i)

            # stop iterations
            if k >= lenRes:
                p.disconnect()
                break
            i += 1

    def runDynamicFalling(self):
        '''For the tasks of articulated fish or ring falling'''
        i = 0 # sim no.
        k = 0 # results data no.
        lenRes = len(self.indices)
        e_grav, e_bend, e_total, e_escape = None, None, None, None

        time.sleep(5)
        while (1):
            # print(i)
            p.stepSimulation()
            p.setGravity(0, 0, self.gravity)
            time.sleep(1/240.)
            
            # update energy texts
            if i == self.indices[k]:
                e_total = np.round(self.startEnergySce[k],decimals=3)
                e_escape = np.round(self.escapeEnergyCostSce[k],decimals=3)
                if self.isArticulatedObj:
                    e_grav = np.round(self.startGEnergySce[k],decimals=3)
                    e_bend = np.round(self.startEEnergySce[k],decimals=3)                 
                k += 1
            
            # get RGB of camera view
            images = p.getCameraImage(self.img_width, 
                                      self.img_height,
                                    #   viewMatrix=self.viewMat,
                                    #   projectionMatrix=self.projMat,
                                    #   renderer=p.ER_BULLET_HARDWARE_OPENGL,
                                      ) # list
            rgb_array = np.reshape(images[2], (self.img_height, self.img_width, 4)) * 1. / 255.            

            # plot   
            energy_tuple = (e_grav, e_bend, e_total, e_escape)
            self.saveIntermImages(rgb_array, energy_tuple, i)

            # stop iterations
            if k >= lenRes:
                p.disconnect()
                break
            i += 1

    def saveIntermImages(self, rgb_array, energy_tuple, i):
        e_grav, e_bend, e_total, e_escape = energy_tuple
        plt.title('Caging formation with escape energy')
        if self.isArticulatedObj:
            plt.text(self.img_width+10, self.img_height-60, 'E_grav:{}'.format(e_grav))
            plt.text(self.img_width+10, self.img_height-40, 'E_bend:{}'.format(e_bend))
        plt.text(self.img_width+10, self.img_height-20, 'E_total:{}'.format(e_total))
        plt.text(self.img_width+10, self.img_height, 'E_escape:{}'.format(e_escape),color='red')
        plt.imshow(rgb_array)
        plt.xticks([])
        plt.yticks([])

        plt.savefig(self.path + "file%03d.png" % i)
        plt.close()

    def imagesToVideo(self):
        os.chdir(self.path)
        subprocess.call([
            'ffmpeg', '-framerate', '30', '-i', 'file%03d.png', '-r', '30', '-pix_fmt', 'yuv420p',
            'cage_with_escape_energy.mp4'
        ])
        for file_name in glob.glob("file*"):
            os.remove(file_name)


if __name__ == '__main__':
    args, parser = argument_parser()
    rigidObjectList = get_non_articulated_objects()
    isArticulatedObj = False if args.object in rigidObjectList else True
    # folderName = './results/HookTrapsRing_16-03-2023-09-48-19/'
    folderName = './results/GripperClenchesStarfish_17-03-2023-08-20-10/'
    # folderName = './results/FishFallsInBowl_17-03-2023-01-26-47/'

    # run a dynamic falling scenario and analyze frame-wise escape energy
    sce = generateVideo(args, folderName, isArticulatedObj)
    if args.scenario in ['FishFallsInBowl', 'HookTrapsFish', 'HookTrapsRing']:
        sce.runDynamicFalling()
    elif args.scenario in ['GripperClenchesStarfish']:
        sce.runClenchFist()

    # convert images to video
    sce.imagesToVideo()
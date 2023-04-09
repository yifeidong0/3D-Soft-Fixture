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
import tikzplotlib

class generateVideo(runScenario):
    def __init__(self, args, dataPaths, isArticulatedObj):
        p.connect(p.GUI)
        p.setTimeStep(1./240.)
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setRealTimeSimulation(0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # planeId = p.loadURDF("plane.urdf", [0,0,-1])
        # self.paths = path_collector()
        self.args = args
        self.dataPaths = dataPaths # results path
        self.isArticulatedObj = isArticulatedObj
        self.gravity = -9.81
        self.endFrame = 800
        self.img_height = 1024
        self.img_width = 1024
        self.dpi = 108

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
        os.chdir('./../')
        self.loadObject()
        self.loadObstacle()
        os.chdir('./results/')
        
        # get results
        self.energyDataAnalysis, self.minDataLen = analyze_energy_data(self.dataPaths, self.isArticulatedObj)
        _, self.indices = get_results_from_csv(self.dataPaths[0], self.isArticulatedObj)
        # self.startEnergySce, self.startGEnergySce, self.startEEnergySce, self.escapeEnergyCostSce = self.energy_data

        # get colors
        self.cls = get_colors()

        # Data structure for selected key-frames
        self.rgb_arrays = []
        # self.ids_selected = [6, 18, 41, 62]
        self.ids_selected = [2, 28, 88, 155]

    def runClenchFist(self):
        '''For the task of gripper clenching starfish'''
        i = 0 # sim no.
        k = 0 # results data no.
        last_k = -1
        self.obstaclePose = self.obstaclePos + self.obstacleEul

        # set initial joint states
        jointPositions,_,_ = self.getJointStates(self.obstacleId) # list(12)
        obstacleJointPos = [jointPositions[i]-450/1000 if (i==1 or i==5 or i==9) else jointPositions[i] for i in range(len(jointPositions))]
        obstacleState = self.obstaclePose + obstacleJointPos
        self.obstacle.set_state(obstacleState)
        
        # start simulation of clenching the fist
        time.sleep(10)
        images = p.getCameraImage(self.img_width, 
                            self.img_height,
                            ) # list
        time.sleep(4)
        images = p.getCameraImage(self.img_width, 
                            self.img_height,
                            ) # list
        time.sleep(4)
        while (1):
            p.stepSimulation()
            p.setGravity(0, 0, self.gravity)
            time.sleep(1/240.)
            
            # Get obstacle joint positions and update them
            jointPositions,_,_ = self.getJointStates(self.obstacleId) # list(12)
            obstacleJointPos = [jointPositions[i]+.8/1000 for i in range(len(jointPositions))]
            obstacleState = self.obstaclePose + obstacleJointPos
            self.obstacle.set_state(obstacleState)

            # Update energy texts
            if i == self.indices[k]:
                k += 1
            
            # Stop iterations
            if k >= self.minDataLen:
                p.disconnect()
                break

            # Get RGB of camera view
            images = p.getCameraImage(self.img_width, 
                                      self.img_height,
                                    #   viewMatrix=self.viewMat,
                                    #   projectionMatrix=self.projMat,
                                    #   renderer=p.ER_TINY_RENDERER,
                                      shadow=1,
                                    #   flags=p.ER_USE_PROJECTIVE_TEXTURE,
                                    #   projectiveTextureView=self.viewMat,
                                    #   projectiveTextureProj=self.projMat
                                    )
            rgb_array = np.reshape(images[2], (self.img_height, self.img_width, 4)) * 1. / 255.            

            # Plot
            self.saveIntermImages(rgb_array, i, k)
            i += 1

            # Save intermediate selected object poses
            if k in self.ids_selected and k != last_k:
                self.rgb_arrays.append(rgb_array)
                last_k = k

    def runDynamicFalling(self):
        '''For the tasks of articulated fish or ring falling'''
        i = 0 # sim no.
        k = 0 # results data no.
        last_k = -1

        # get RGB of camera view
        time.sleep(10)
        images = p.getCameraImage(self.img_width, 
                            self.img_height,
                            ) # list
        time.sleep(4)
        images = p.getCameraImage(self.img_width, 
                            self.img_height,
                            ) # list
        time.sleep(4)
        while (1):
            # print(i)
            p.stepSimulation()
            p.setGravity(0, 0, self.gravity)
            time.sleep(1/240.)
            
            # update energy texts
            if i == self.indices[k]:               
                k += 1
            
            # stop iterations
            if k >= self.minDataLen:
                p.disconnect()
                break

            # get RGB of camera view
            images = p.getCameraImage(self.img_width, 
                                      self.img_height,
                                      ) # list
            rgb_array = np.reshape(images[2], (self.img_height, self.img_width, 4)) * 1. / 255.            

            # plot   
            self.saveIntermImages(rgb_array, i, k)
            i += 1

            # Save intermediate selected object poses
            if k in self.ids_selected and k != last_k:
                self.rgb_arrays.append(rgb_array)
                last_k = k
                
    def saveIntermSelectedImages(self):
        ''' Save some selected images for ICRA workshop abstract
        '''
        # e_total,e_grav,e_bend,e_escape,_,_,_,_,_,_,_,_ = self.energyDataAnalysis

        # fig = plt.figure(constrained_layout=True)
        # fig.set_size_inches(16,8)
        # cameraImgNames = ['A', 'B', 'C', 'D']
        # axs = fig.subplot_mosaic([['Left', 'A', 'B'],['Left', 'C', 'D']],
        #                         gridspec_kw={'width_ratios':[2,1,1]},)
        # axs['Left'].set_title('Escape energy plots', fontsize=18)

        # for i, name in enumerate(cameraImgNames):
        #     axs[name].set_title(name, fontsize=18)
        #     axs[name].imshow(self.rgb_arrays[i])
        #     axs[name].set_xticks([])
        #     axs[name].set_yticks([])

        # # # plot data text values
        # # ax1.text(self.img_width-100, self.img_height+20, 'E_total:{}'.format(np.round(e_total[k],decimals=3)), fontsize=14, color=self.cls[0])
        # # e_escape = np.round(e_escape[k],decimals=3) if e_escape[k] is not np.nan else np.inf
        # # ax1.text(self.img_width-100, self.img_height+45, 'E_escape:{}'.format(e_escape), fontsize=14, color=self.cls[3])
        # # if self.isArticulatedObj:
        # #     ax1.text(self.img_width-250, self.img_height+20, 'E_grav:{}'.format(np.round(e_grav[k],decimals=3)), fontsize=14, color=self.cls[1])
        # #     ax1.text(self.img_width-250, self.img_height+45, 'E_bend:{}'.format(np.round(e_bend[k],decimals=3)), fontsize=14, color=self.cls[2])

        # # plot energy curves
        # plot_escape_energy(axs['Left'], self.energyDataAnalysis, self.minDataLen, self.isArticulatedObj, axvline=self.ids_selected)
        # # plt.title('Escape energy plot of energy-bounded caging',fontsize=16)
        # # tikzplotlib.save(self.dataPaths[0] + "ICRAfigure3.tex")
        # plt.savefig(self.dataPaths[0] + "ICRAfigure3.png")
        # plt.close()

        # Save the four subfigures separately
        for i in range(len(self.rgb_arrays)):
            f, ax = plt.subplots(figsize=(800/self.dpi, 800/self.dpi), dpi=self.dpi)
            ax.imshow(self.rgb_arrays[i])
            plt.xticks([]) 
            plt.yticks([]) 
            plt.axis('off')
            f.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=None, hspace=None)
            plt.savefig(self.dataPaths[0] + "ICRA-subfig-{}.png".format(i), bbox_inches='tight', pad_inches=0)
            plt.close()


    def saveIntermImages(self, rgb_array, i, k):
        e_total,e_grav,e_bend,e_escape,_,_,_,_,_,_,_,_ = self.energyDataAnalysis
        _, (ax1, ax2) = plt.subplots(1,2,width_ratios=[1,1.2],figsize=(16,9))
        ax1.imshow(rgb_array)
        ax1.set_xticks([])
        ax1.set_yticks([])

        # plot data text values
        ax1.text(self.img_width-150, self.img_height+50, 'E_total:{}'.format(np.round(e_total[k],decimals=3)), fontsize=16, color=self.cls[0])
        e_escape = np.round(e_escape[k],decimals=3) if e_escape[k] is not np.nan else np.inf
        ax1.text(self.img_width-150, self.img_height+105, 'E_escape:{}'.format(e_escape), fontsize=16, color=self.cls[3])
        if self.isArticulatedObj:
            ax1.text(self.img_width-500, self.img_height+50, 'E_grav:{}'.format(np.round(e_grav[k],decimals=3)), fontsize=16, color=self.cls[1])
            ax1.text(self.img_width-500, self.img_height+105, 'E_elas:{}'.format(np.round(e_bend[k],decimals=3)), fontsize=16, color=self.cls[2])

        # plot energy curves
        plot_escape_energy(ax2, self.energyDataAnalysis, self.minDataLen, self.isArticulatedObj, axvline=[k], addAxvLabel=0)
        plt.title('Quasi-static Analysis of Soft Fixture Escape Energy',fontsize=16)
        plt.savefig(self.dataPaths[0] + "file%03d.png" % i, dpi=self.dpi)
        plt.close()

    def imagesToVideo(self):
        os.chdir(self.dataPaths[0])
        subprocess.call([
            'ffmpeg', '-framerate', '30', '-i', 'file%03d.png', '-r', '30', '-pix_fmt', 'yuv420p',
            'soft-fixture2.mp4'
        ])
        for file_name in glob.glob("file*"):
            os.remove(file_name)
        os.chdir('./../')


if __name__ == '__main__':
    args, parser = argument_parser()
    rigidObjectList = get_non_articulated_objects()
    isArticulatedObj = False if args.object in rigidObjectList else True
    
    # get folders of the same task
    folderList = []
    path = './results/'
    os.chdir(path)
    for folderName in glob.glob(args.scenario + "*"):
        folderList.append(folderName + '/')
    
    # run a dynamic falling scenario and analyze frame-wise escape energy
    sce = generateVideo(args, folderList, isArticulatedObj)
    if args.scenario in ['FishFallsInBowl', 'HookTrapsRing']:
        sce.runDynamicFalling()
    elif args.scenario in ['GripperClenchesStarfish']:
        sce.runClenchFist()

    # convert images to video
    sce.imagesToVideo()

    # Save plot for ICRA workshop
    # sce.saveIntermSelectedImages()



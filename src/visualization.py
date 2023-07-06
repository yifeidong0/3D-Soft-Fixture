from datetime import datetime
import os
# import subprocess
import glob
import csv
import numpy as np
import matplotlib.pyplot as plt
from utils import *
# from main import argument_parser
# import pybullet as p
# import pybullet_data
# import time
from scipy import interpolate
import tikzplotlib

def record_data_init(sce, args, env):
    # get current time
    now = datetime.now()
    dt_string = now.strftime("%d-%m-%Y-%H-%M-%S") # dd/mm/YY H:M:S
    print("date and time =", dt_string)
    folderName = './results/{}_{}'.format(args.scenario, dt_string)
    os.mkdir(folderName)

    # create csv headers
    headersOther = ['start_energy', 'start_gravity_energy', 'start_elastic_energy', 'escape_energy_cost']
    if args.scenario in ['MaskEar']:
        headerObj = ['frameID',]
        for j in range(sce.numCtrlPoint):
            headerObj.append('obj_joint_{}_pos_x'.format(j))
            headerObj.append('obj_joint_{}_pos_y'.format(j))
            headerObj.append('obj_joint_{}_pos_z'.format(j))
        headersObs = ['obs_pos_x', 'obs_pos_y', 'obs_pos_z', 
                        'obj_qtn_x', 'obj_qtn_y', 'obj_qtn_z', 'obj_qtn_w']
        headers = headerObj + headersObs + headersOther
    else:
        objJointNum = len(sce.objJointPosSce[0])
        obsJointNum = len(sce.obsJointPosSce[0])
        headerObjJoint = []
        headerObsJoint = []
        for j in range(objJointNum):
            headerObjJoint.append('obj_joint_{}_pos'.format(j))
        for s in range(obsJointNum):
            headerObsJoint.append('obs_joint_{}_pos'.format(s))

        headersObj = ['index', 'obj_pos_x', 'obj_pos_y', 'obj_pos_z', 
                'obj_qtn_x', 'obj_qtn_y', 'obj_qtn_z', 'obj_qtn_w'] + headerObjJoint
        headersObs = ['obs_pos_x', 'obs_pos_y', 'obs_pos_z', 
                'obj_qtn_x', 'obj_qtn_y', 'obj_qtn_z', 'obj_qtn_w'] + headerObsJoint
        headers = headersObj + headersObs + headersOther

    # write headers to csv
    with open('{}/data.csv'.format(folderName), 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(i for i in headers)
        # writer.writerows(data)

    # save other info to txt
    with open('{}/info.txt'.format(folderName), "w") as text_file:
        if args.scenario in ['MaskEar']:
            text_file.write("*****Search objectGoal: {}\n".format(sce.objectGoal))
        else:
            if args.planner in ['BITstar']:
                text_file.write("*****Search goalCoMPose: {}\n".format(sce.goalCoMPose))
            else:
                text_file.write("*****Search goal region: {}\n".format(sce.goalSpaceBounds))
        text_file.write("*****Search basePosBounds: {}\n".format(sce.basePosBounds))
        text_file.write("*****args: {}\n".format(args))
        text_file.write("*****planner params: {}".format(env.pb_ompl_interface.planner.params()))
        if args.object == 'Fish':
            text_file.write("*****Fish links' mass: {}\n".format(env.pb_ompl_interface.potentialObjective.masses))
            text_file.write("*****Fish joints' stiffness: {}\n".format(env.pb_ompl_interface.potentialObjective.stiffnesss))

    return folderName

def record_data_loop(sce, args, energyData, folderName, i):
    if args.scenario in ['MaskEar']:
        data = flatten_nested_list([
            [sce.frameID[i]], sce.objectStateSce[i], sce.obstaclePos, sce.obstacleQtn, energyData
            ])
    else:
        data = flatten_nested_list([
            [sce.idxSce[i]], sce.objBasePosSce[i], sce.objBaseQtnSce[i],
            sce.objJointPosSce[i], sce.obsBasePosSce[i], sce.obsBaseQtnSce[i],
            sce.obsJointPosSce[i], energyData
            ])
        
    with open('{}/data.csv'.format(folderName), 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(data)

def record_state_data_for_blender(sce, args):
    now = datetime.now()
    dt_string = now.strftime("%d-%m-%Y-%H-%M-%S") # dd/mm/YY H:M:S
    folderName = './results/{}_{}_4blender'.format(args.scenario, dt_string)
    os.mkdir(folderName)

    # create csv headers
    objJointNum = len(sce.objJointPosSce[0])
    headerObjJoint = []
    for j in range(objJointNum):
        headerObjJoint.append('obj_joint_{}_pos'.format(j))

    headersObj = ['index', 'obj_pos_x', 'obj_pos_y', 'obj_pos_z', 
               'obj_qtn_x', 'obj_qtn_y', 'obj_qtn_z', 'obj_qtn_w'] + headerObjJoint
    headers = headersObj

    # write headers to csv
    with open('{}/data.csv'.format(folderName), 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(i for i in headers)

    # write data to csv
    for i in range(len(sce.idxSce)):
        data = flatten_nested_list([
            [sce.idxSce[i]], sce.objBasePosSce[i], sce.objBaseQtnSce[i],
            sce.objJointPosSce[i],
            ])

        with open('{}/data.csv'.format(folderName), 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(data)

def record_dynamics_scene(sce, args):
    # get current time
    now = datetime.now()
    dt_string = now.strftime("%d-%m-%Y-%H-%M-%S") # dd/mm/YY H:M:S
    folderName = './results/{}_{}_dynamics'.format(args.scenario, dt_string)
    os.mkdir(folderName)

    # create csv headers
    objJointNum = len(sce.objJointPosSce[0])
    headerObjJoint = []
    for j in range(objJointNum):
        headerObjJoint.append('obj_joint_{}_pos'.format(j))

    headersObj = ['index', 'obj_pos_x', 'obj_pos_y', 'obj_pos_z', 
               'obj_qtn_x', 'obj_qtn_y', 'obj_qtn_z', 'obj_qtn_w'] + headerObjJoint
    headersObs = ['obs_pos_x', 'obs_pos_y', 'obs_pos_z', 
               'obs_qtn_x', 'obs_qtn_y', 'obs_qtn_z', 'obs_qtn_w']
    
    if args.scenario in ['HookFishHole']:
        headersBox = ['box_pos_x', 'box_pos_y', 'box_pos_z',]
        headers = headersObj + headersObs + headersBox
    elif args.scenario in ['ShovelFish', 'HookTrapsRing']: # rigid object scenarios
        headers = headersObj + headersObs
    elif args.scenario in ['StarfishBowl']:
        headersBox = ['box_pos_x', 'box_pos_y', 'box_pos_z', 'box_eul_x', 'box_eul_y', 'box_eul_z',]
        headers = headersObj + headersObs + headersBox
    elif args.scenario in ['BimanualRubic']:
        obsJointNum = sce.numJoints
        headerObsJoint = []
        for j in range(obsJointNum):
            headerObsJoint.append('obs_joint_{}_pos'.format(j))
        headers = headersObj + (headersObs+headerObsJoint)*2
    elif args.scenario in ['HandbagGripper']:
        headersObj = []
        numChainNode = sce.numCtrlPoint + 3
        for j in range(numChainNode):
            for k in range(3):
               headersObj.append('chain_node_{}_pos_{}'.format(j,k))

        obsJointNum = sce.numJoints
        headerObsJoint = []
        for j in range(obsJointNum):
            headerObsJoint.append('obs_joint_{}_pos'.format(j))
        headers = headersObj + (headersObs+headerObsJoint)

    # write headers to csv
    with open('{}/data.csv'.format(folderName), 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(i for i in headers)

    # write data to csv
    for i in range(len(sce.idxSce)):
        if args.scenario in ['HookFishHole']:
            data = flatten_nested_list([
                [sce.idxSce[i]], sce.objBasePosSce[i], sce.objBaseQtnSce[i],
                sce.objJointPosSce[i], sce.obsBasePosSce[i], sce.obsBaseQtnSce[i], 
                sce.boxBasePosSce[i], 
                ])
        elif args.scenario in ['ShovelFish', 'HookTrapsRing']:
            data = flatten_nested_list([
                [sce.idxSce[i]], sce.objBasePosSce[i], sce.objBaseQtnSce[i],
                sce.objJointPosSce[i], sce.obsBasePosSce[i], sce.obsBaseQtnSce[i], 
                ])
        elif args.scenario in ['StarfishBowl']:
            data = flatten_nested_list([
                [sce.idxSce[i]], sce.objBasePosSce[i], sce.objBaseQtnSce[i],
                sce.objJointPosSce[i], sce.obsBasePosSce[i], sce.obsBaseQtnSce[i], 
                sce.boxBasePosSce[i], sce.boxBaseEulSce[i]               
                ])
        elif args.scenario in ['BimanualRubic']:
            data = flatten_nested_list([
                [sce.idxSce[i]], sce.objBasePosSce[i], sce.objBaseQtnSce[i], sce.objJointPosSce[i], 
                sce.obsBasePosSce[i], sce.obsBaseQtnSce[i], sce.obsJointPosSce[i],
                sce.obsBasePosSce1[i], sce.obsBaseQtnSce1[i], sce.obsJointPosSce1[i], ])
        elif args.scenario in ['HandbagGripper']:
            chainState = sce.objBasePosSce[i] + sce.objBaseEulSce[i] + sce.objJointPosSce[i]
            _, chainNodePos = get_chain_node_pos(chainState, sce.linkLen)
            data = flatten_nested_list([
                 flatten_nested_list(chainNodePos),
                 sce.obsBasePosSce[i], sce.obsBaseQtnSce[i], sce.obsJointPosSce[i],])

        with open('{}/data.csv'.format(folderName), 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(data)


def save_escape_path_4blender(args, path: list) -> None:
    '''Save escape paths from planners as csv files for Blender.
    '''
    # get current time
    now = datetime.now()
    dt_string = now.strftime("%d-%m-%Y-%H-%M-%S") # dd/mm/YY H:M:S
    folderName = './results/{}_{}_escape_path_4blender'.format(args.scenario, dt_string)
    os.mkdir(folderName)

    # write headers to csv
    with open('{}/data.csv'.format(folderName), 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(' ')

    # write data to csv
    for state in path:
        with open('{}/data.csv'.format(folderName), 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(state)

def record_data_benchmark_bound_shrink(costListRuns, timeTakenListRuns, frameId, folderName):
    with open('{}/data.csv'.format(folderName), 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        for i in range(len(costListRuns)):
            dataCost = flatten_nested_list([
                [frameId], [i], costListRuns[i]
                ])
            dataTime = flatten_nested_list([
                [frameId], [i], timeTakenListRuns[i]
                ])
            writer.writerow(dataTime)
            writer.writerow(dataCost)
        
def analyze_energy_data(folderList, isArticulatedObject):
    # Get energy data from multiple csv files
    energyDataList = []
    minDataLen = np.inf
    for f in folderList:
        # Get energy data from single csv file
        energyData, indices = get_results_from_csv(f, isArticulatedObject)
        energyDataList.append(energyData)
        minDataLen = min(minDataLen, len(energyData[3]))

    # unpack the piled datasets
    totalEnergySce = energyDataList[3][0][:minDataLen]
    GEnergySce, EEnergySce, escapeEnergyCostSce = [], [], []
    for d in range(len(energyDataList)):
        _, GEnergy, EEnergy, escapeEnergyCost = energyDataList[d]
        GEnergySce.append(GEnergy[:minDataLen])
        EEnergySce.append(EEnergy[:minDataLen])
        escapeEnergyCostSce.append(escapeEnergyCost[:minDataLen])
    
    # convert to numpy array
    totalEnergySce = np.asarray(totalEnergySce)
    GEnergySce = np.asarray(GEnergySce)
    EEnergySce = np.asarray(EEnergySce)
    escapeEnergyCostSce = np.asarray(escapeEnergyCostSce)

    # replace inf by nan
    GEnergySce[np.isinf(GEnergySce)] = np.nan
    EEnergySce[np.isinf(EEnergySce)] = np.nan
    escapeEnergyCostSce[np.isinf(escapeEnergyCostSce)] = np.nan

    # get min, mean, std
    minTot, minG, minE, minEsc = totalEnergySce, np.nanmin(GEnergySce,axis=0), np.nanmin(EEnergySce,axis=0), np.nanmin(escapeEnergyCostSce,axis=0)
    meanTot, meanG, meanE, meanEsc = None, np.nanmean(GEnergySce,axis=0), np.nanmean(EEnergySce,axis=0), np.nanmean(escapeEnergyCostSce,axis=0)
    stdTot, stdG, stdE, stdEsc = None, np.nanstd(GEnergySce,axis=0), np.nanstd(EEnergySce,axis=0), np.nanstd(escapeEnergyCostSce,axis=0)

    return (minTot, minG, minE, minEsc, meanTot, meanG, meanE, meanEsc, stdTot, stdG, stdE, stdEsc), minDataLen, indices

def plot_escape_energy(ax, energyDataAnalysis, minDataLen, indices,
                       isArticulatedObject=False, axvline=None, addAxvLabel=1, labelNames=['A','B','C','D']):
    # TODO: use broken axis to represent inf.
    # Color codes
    cls = get_colors()
    IterId = np.asarray(list(range(minDataLen)))

    minTot, minG, minE, minEsc, _, meanG, meanE, meanEsc, _, stdG, stdE, stdEsc = energyDataAnalysis

    # Plot min escape energy
    useIndices = 1 # 1 for fish-shovel, rubic-bimanual data with unevenly spaced indices
    if useIndices:
        x = [id / 571.4 for id in indices] # for fish-shovel
        # x = indices
    else:
        x = IterId

    useDoubleAxis = 0 # 1 for fish-shovel
    if useDoubleAxis:
        ax1 = ax.twinx()
    # ax.plot(x, minTot, '-', color=cls[0], linewidth=2, label='total energy')
    # if isArticulatedObject:
    #     ax.plot(x, minG, '-', color=cls[1], label='g-potential energy')
    #     if useDoubleAxis:
    #         ax1.plot(x, minE, '-', color=cls[2], label='e-potential energy')
    #     else:
    #         ax.plot(x, minE, '-', color=cls[2], label='e-potential energy')
    if useDoubleAxis:
        ax1.plot(x, minEsc, '-', color=cls[3], linewidth=2, label='escape energy')
        ax1.fill_between(x, meanEsc-stdEsc, meanEsc+stdEsc, alpha=0.4, color=cls[3])
    else:
        ax.plot(x, meanEsc, '-', color=cls[3], linewidth=2, label='escape energy')
        ax.fill_between(x, meanEsc-stdEsc, meanEsc+stdEsc, alpha=0.4, color=cls[3])
    
    # Optional: plot vertical line of current no. of iteration
    if axvline is not None:
        for i in range(len(axvline)):
            ax.axvline(x=axvline[i], color='k', linestyle='--', linewidth=1)

            # Add A, B, C... labels to vertical lines (For workshop paper)
            if addAxvLabel:
                ax.text(axvline[i]+.5, -.1, labelNames[i], fontsize=18, color='k')
                # ax.text(axvline[i]+.5, -1, labelNames[i], fontsize=18, color='k')

    ax.set_xlabel('# iterations',fontsize=14)
    ax.set_ylabel('energy / J',fontsize=14)
    # ax.set_ylabel('energy-total/grav / J',fontsize=14)
    # ax.set_yscale('log')
    # ax.set_ylim(-.2,3.6)
    if useDoubleAxis:
        ax1.set_ylabel('energy-elas/esc / J',fontsize=14)
        ax1.set_ylim(0,5)
        ax1.legend(fontsize=14,loc='upper right')
    # ax.set_aspect(26.5)
    ax.grid(True)
    # ax.legend(fontsize=14,loc='upper left')
    # ax.set_xticks(np.arange(0,minDataLen,30).astype(int))

def get_results_from_csv(folderName, isArticulatedObject=False):
    indices = []
    start_energy = []
    start_gravity_energy = []
    start_elastic_energy = []
    escape_energy_cost = []

    with open('{}/data.csv'.format(folderName), 'r') as csvfile:
        csvreader = csv.DictReader(csvfile)

        for row in csvreader:
            indices.append(int(row['index']))
            # indices.append(float(row['frameID'])) # MaskEar
            start_energy.append(float(row['start_energy']))
            escape_energy_cost.append(float(row['escape_energy_cost']))
            if isArticulatedObject:
                start_gravity_energy.append(float(row['start_gravity_energy']))
                start_elastic_energy.append(float(row['start_elastic_energy']))
    energy_data = (start_energy, start_gravity_energy, start_elastic_energy, escape_energy_cost)

    return energy_data, indices

def plot_escape_energy_from_multi_csv(ax, folderList, isArticulatedObject=False, axvline=None):
    # Get min, mean, std of data
    energyDataAnalysis, minDataLen, indices = analyze_energy_data(folderList, isArticulatedObject)

    # Escape energy plots
    plot_escape_energy(ax, energyDataAnalysis, minDataLen, indices, isArticulatedObject, axvline)


'''Plot escape cost from multiple csv files with std shading'''
if __name__ == '__main__':
    args, parser = argument_parser()
    rigidObjectList = get_non_articulated_objects()
    isArticulatedObj = False if args.object in rigidObjectList else True
    
    folderList = []
    # path = './results/ICRA2024/Scenario01Hook-Fish'
    # path = './results/ICRA2024/Scenario02Starfish-HandAndBowl'
    # path = 'results/ICRA2024/Scenario03Mask-Ear'
    path = 'results/ICRA2024/Scenario04Fish-Shovel'
    # path = 'results/ICRA2024/Scenario05Rubic-Bimanual'
    # path = 'results/ICRA2024/Scenario06Bag-Gripper'
    os.chdir(path)
    # HookFishHole
    for file_name in glob.glob(args.scenario + "*"):
        folderList.append(file_name)

    _, ax = plt.subplots()
    plot_escape_energy_from_multi_csv(ax, folderList, isArticulatedObj)
    plt.title('Escape energy in a dynamic scenario - {}'.format(args.scenario))
    # plt.show()
    plt.savefig('{}/energy_plot_std.png'.format(folderList[0]))
    tikzplotlib.save('{}/energy_plot_std.tex'.format(folderList[0]))

##################################################################################
###############################For Benchmark Plot#################################
##################################################################################

def get_benckmark_results_from_csv(folderName, cInit, getOnlyOneFrame=True, noIter=15):
    tOneIter = [0.0]
    cOneIter = [float(cInit)]
    timeTickList = []
    escapeEnergyList = []

    with open('{}/data.csv'.format(folderName), 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        i = 0
        for row in csvreader:
            if i % 2 == 0:
                t = [float(j) for j in row[2:]]
                timeTickList.append(tOneIter+t)
            else:
                c = [float(k) for k in row[2:]]
                escapeEnergyList.append(cOneIter+c)
            i += 1

            # Break when the data for next frame appear
            if getOnlyOneFrame and i >= 2*noIter:
                break

    return timeTickList, escapeEnergyList

def test05_get_structured_bits_data(noIter=10):
    folderName = 'results/ICRA2024/Test05HookRingErrorVsRuntime/bit*_2sec_to_600sec'
    t_list_int = [2,6,10,15,22,30,38,50,70,100,200,350,600]
    t_list = [str(t) for t in t_list_int]

    # Read data folder names
    os.chdir(folderName)
    folderListAll = []
    for t in t_list:
        folderList = []
        for folderName in glob.glob(t + "*"):
            folderList.append(folderName + '/')
        folderListAll.append(folderList)
    
    # Record data
    escapeEnergyAll = []
    for i,tim in enumerate(t_list):
        escapeEnergy = []
        for j in range(noIter):
            with open('{}/data.csv'.format(folderListAll[i][j]), 'r') as csvfile:
                csvreader = csv.DictReader(csvfile)
                for row in csvreader:
                    escapeEnergy.append(float(row['escape_energy_cost']))
        escapeEnergyAll.append(escapeEnergy)
    escapeEnergyAll = np.asarray(escapeEnergyAll).T

    # Summarize and write to new file
    for j in range(noIter):
        with open('{}/data.csv'.format(folderName), 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([0,]+[j]+t_list_int)
            writer.writerow([0,]+[j]+escapeEnergyAll[j].tolist())

    os.chdir('./../../../../')

from matplotlib.ticker import FormatStrFormatter

def plot_convergence_test(bits, bin, inc, folderName, groundTruthEscapeEnergy, maxTime=600):
    # Color codes
    cls = get_colors()
    noIter = 10
    noIterBin = 30
    timeTickListBits, escapeEnergyListBits = bits
    timeTickListInc, escapeEnergyListInc = inc
    timeTickListBin, escapeEnergyListBin = bin

    # Create numpy arrays structures for data
    timeTick = np.asarray(list(range(maxTime)))
    escapeEnergyBits = np.zeros((noIter, maxTime))
    escapeEnergyInc = np.zeros((noIter, maxTime))
    escapeEnergyBin = np.zeros((noIterBin, maxTime))

    # Interpolation
    for i in range(noIter):
        f = interpolate.interp1d(timeTickListBits[i], escapeEnergyListBits[i])
        tnew = np.arange(0, maxTime, 1)
        escapeEnergyBits[i,:] = np.asarray(f(tnew))

        f = interpolate.interp1d(timeTickListInc[i], escapeEnergyListInc[i])
        tnew = np.arange(0, maxTime, 1)
        escapeEnergyInc[i,:] = np.asarray(f(tnew))

    for i in range(noIterBin):
        f = interpolate.interp1d(timeTickListBin[i], escapeEnergyListBin[i])
        tnew = np.arange(0, maxTime, 1)
        escapeEnergyBin[i,:] = np.asarray(f(tnew))

    # Retrieve mean and std
    escapeEnergyBits[np.isinf(escapeEnergyBits)] = np.nan
    escapeEnergyBitsMean = np.nanmean(escapeEnergyBits, axis=0)
    escapeEnergyBitsStd = np.nanstd(escapeEnergyBits, axis=0)
    escapeEnergyIncMean = np.mean(escapeEnergyInc, axis=0)
    escapeEnergyIncStd = np.std(escapeEnergyInc, axis=0)
    escapeEnergyBinMean = np.mean(escapeEnergyBin, axis=0)
    escapeEnergyBinStd = np.std(escapeEnergyBin, axis=0)

    # Plot mean escape energy cost
    fig, ax = plt.subplots()
    ax.set_xscale('log')
    # ax.set_yscale('log')
    ax.set_xlim(1,600)
    ax.plot(timeTick, escapeEnergyBitsMean, '-', color=cls[0], linewidth=3, label='BIT* search')
    ax.plot(timeTick, escapeEnergyIncMean, '-', color=cls[1], linewidth=3, label='incremental search')
    ax.plot(timeTick, escapeEnergyBinMean, '-', color=cls[2], linewidth=3, label='binary search')

    # Plot std shade
    ax.fill_between(timeTick, escapeEnergyBitsMean-escapeEnergyBitsStd, escapeEnergyBitsMean+escapeEnergyBitsStd, alpha=0.3, color=cls[0])
    ax.fill_between(timeTick, escapeEnergyIncMean-escapeEnergyIncStd, escapeEnergyIncMean+escapeEnergyIncStd, alpha=0.3, color=cls[1])
    ax.fill_between(timeTick, escapeEnergyBinMean-escapeEnergyBinStd, escapeEnergyBinMean+escapeEnergyBinStd, alpha=0.3, color=cls[2])
    
    # Plot lines parallel to x axis
    # ax.axhline(y=escapeEnergyBmean[-1], color=cls[0], linestyle = '--')
    # ax.axhline(y=escapeEnergyEmean[-1], color=cls[3], linestyle = '--')
    ax.axhline(y=groundTruthEscapeEnergy, color=cls[3], linestyle = '--', label='reference')

    # Settings for plot
    ax.set_xlabel('Time / sec',fontsize=16)
    ax.set_ylabel('Escape energy cost / J',fontsize=16)
    # ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    plt.title('Convergence of search algorithms over time',fontsize=16)
    plt.legend()
    # plt.show()
    # plt.savefig('{}/test05convergence.png'.format(folderName), dpi=200)
    tikzplotlib.save('{}/test05convergence.tex'.format(folderName))

def plot_acurracy_test(CostBiFrames, CostInFrames, SaveFolderName, groundTruthZ, startKeyFrame=18):
    '''Plot the comparisons of two searching algorithms (Bound shrink search and energy minimization search) over several frames.
    '''
    cls = get_colors()
    noIter = 3
    noIterBin = 5

    # Retrieve data of bound shrink
    finalCostBiFrames = [i[-1] for i in CostBiFrames]
    noFrame = int(len(finalCostBiFrames) / noIterBin)
    escapeCostBi = np.asarray(finalCostBiFrames).reshape((noIterBin, noFrame),order='F')
    finalCostInFrames = [i[-1] for i in CostInFrames]
    escapeCostIn = np.asarray(finalCostInFrames).reshape((noIter, noFrame),order='F')
        
    # Get folders of the same task
    args, parser = argument_parser()
    path = './results/ICRA2024/Test01HookRing3BasicSearch/'
    os.chdir(path)
    folderList = []
    for folderName in glob.glob(args.scenario + "*"):
        folderList.append(folderName + '/')
    noFolder = len(folderList)
    
    # Retrieve data of energy minimization
    escapeCostEM = np.zeros((noFolder, noFrame))
    for i,folder in enumerate(folderList):
        # Get energy data from single csv file
        energyData, _ = get_results_from_csv(folder)
        row = energyData[3][startKeyFrame:startKeyFrame+noFrame]
        escapeCostEM[i,:] = np.asarray(row)

        # Get initial energy over frames
        initEnergy = []
        groundTruthEscapeEnergy = []
        with open('{}/data.csv'.format(folder), 'r') as csvfile:
            csvreader = csv.DictReader(csvfile)
            for row in csvreader:
                initEnergy.append(float(row['obj_pos_z']))
                groundTruthEscapeEnergy.append(max(0.0, groundTruthZ-float(row['obj_pos_z'])))
    groundTruthEscapeEnergy = groundTruthEscapeEnergy[:noFrame+1]
    groundTruthEscapeEnergy = np.asarray(groundTruthEscapeEnergy[:-1])
    os.chdir('./../../../')

    # Retrieve indices of keyframes
    id = np.asarray(list(range(startKeyFrame, startKeyFrame+noFrame)))
    xticks = np.asarray(list(range(startKeyFrame, startKeyFrame+noFrame+1, 10)))
    
    # Retrieve mean and std
    escapeCostEM[np.isinf(escapeCostEM)] = np.nan
    escapeCostBimean = np.mean(escapeCostBi, axis=0)
    escapeCostInmean = np.mean(escapeCostIn, axis=0)
    escapeCostEMmean = np.nanmean(escapeCostEM, axis=0)
    escapeCostEMmean = escapeCostEMmean[:noFrame+1]
    escapeCostBistd = np.std(escapeCostBi, axis=0)
    escapeCostInstd = np.std(escapeCostIn, axis=0)
    escapeCostEMstd = np.nanstd(escapeCostEM, axis=0)
    escapeCostEMstd = escapeCostEMstd[:noFrame+1]

    ###### Escape energy plot
    _, ax = plt.subplots()
    # ax.set_yscale('log')
    # ax.set_ylim(0.4, 1.0)
    ax.plot(id, escapeCostBimean, '-', color=cls[0], linewidth=2, label='binary') # (8 min run time for each keyframe)
    ax.plot(id, escapeCostInmean, '-', color=cls[1], linewidth=2, label='incremental') # (8 min run time for each keyframe)
    ax.plot(id, escapeCostEMmean, '-', color=cls[2], linewidth=2, label='BIT*') # (2 min run time for each keyframe)
    ax.plot(id, groundTruthEscapeEnergy, '-', color=cls[-1], linewidth=2, label='reference')

    # Plot std shade
    ax.fill_between(id, escapeCostBimean-escapeCostBistd, escapeCostBimean+escapeCostBistd, alpha=0.3, color=cls[0])
    ax.fill_between(id, escapeCostInmean-escapeCostInstd, escapeCostInmean+escapeCostInstd, alpha=0.3, color=cls[1])
    ax.fill_between(id, escapeCostEMmean-escapeCostEMstd, escapeCostEMmean+escapeCostEMstd, alpha=0.3, color=cls[2])

    # Settings for plot
    ax.set_xticks(xticks.astype(int))
    ax.set_xlim(-0.02, 26)
    ax.set_xlabel('# iterations',fontsize=14)
    ax.set_ylabel('Escape energy / J',fontsize=14)
    # plt.title('Accuracy of search algorithms over frames',fontsize=16)
    plt.legend()
    plt.grid(linestyle='--')
    # plt.show()
    plt.savefig('{}/test01-cost.png'.format(SaveFolderName), dpi=200)
    tikzplotlib.save('{}/test01-cost.tex'.format(SaveFolderName))

    ###### Error plot
    escapeCostBiErr = (escapeCostBi-groundTruthEscapeEnergy)/groundTruthEscapeEnergy
    escapeCostInErr = (escapeCostIn-groundTruthEscapeEnergy)/groundTruthEscapeEnergy
    escapeCostEMErr = (escapeCostEM-groundTruthEscapeEnergy)/groundTruthEscapeEnergy

    escapeCostBiErrMean = np.mean(escapeCostBiErr, axis=0)
    escapeCostInErrMean = np.mean(escapeCostInErr, axis=0)
    escapeCostEMErrMean = np.nanmean(escapeCostEMErr, axis=0)
    escapeCostEMErrMean = escapeCostEMErrMean[:noFrame+1]
    
    escapeCostBiErrStd = np.std(escapeCostBiErr, axis=0)
    escapeCostInErrStd = np.std(escapeCostInErr, axis=0)
    escapeCostEMErrStd = np.nanstd(escapeCostEMErr, axis=0)
    escapeCostEMErrStd = escapeCostEMErrStd[:noFrame+1]
    # print(np.std(escapeCostBiErrStd))
    # print(np.std(escapeCostInErrStd))
    # print(np.std(escapeCostEMErrStd))

    _, ax = plt.subplots()
    ax.plot(id, escapeCostBiErrMean, '-', color=cls[0], linewidth=2, label='Binary search') # (8 min run time for each keyframe)
    ax.plot(id, escapeCostInErrMean, '-', color=cls[1], linewidth=2, label='Incremental search') # (8 min run time for each keyframe)
    ax.plot(id, escapeCostEMErrMean, '-', color=cls[2], linewidth=2, label='BIT* search') # (2 min run time for each keyframe)

    # Plot std shade
    ax.fill_between(id, escapeCostBiErrMean-escapeCostBiErrStd, escapeCostBiErrMean+escapeCostBiErrStd, alpha=0.3, color=cls[0])
    ax.fill_between(id, escapeCostInErrMean-escapeCostInErrStd, escapeCostInErrMean+escapeCostInErrStd, alpha=0.3, color=cls[1])
    ax.fill_between(id, escapeCostEMErrMean-escapeCostEMErrStd, escapeCostEMErrMean+escapeCostEMErrStd, alpha=0.3, color=cls[2])

    # Settings for plot
    ax.set_xticks(xticks.astype(int))
    ax.set_ylim(-0.02, 0.12)
    ax.set_xlim(-0.02, 26)
    ax.set_xlabel('Index of frames',fontsize=16)
    ax.set_ylabel('Error',fontsize=16)
    plt.title('Error of search algorithms w.r.t. measured reference',fontsize=16)
    plt.legend()
    # plt.show()
    plt.savefig('{}/test01-error.png'.format(SaveFolderName), dpi=200)


def plot_test02(SaveFolderName, groundTruthZ, ):
    '''Plot the comparisons of two searching algorithms (Bound shrink search and energy minimization search) over several frames.
    '''
    cls = get_colors()
    noFrame = 8
    noFolder = 5
    # planners = ['informedRRTstar', 'RRTstar', 'AITstar', 'RRTConnect', 'RRT']
    planners = ['informedRRTstar', '_RRTstar', 'AITstar']
        
    # Get folder names
    os.chdir(SaveFolderName)
    folderList = []
    for p in planners:
        f = []
        for folderName in glob.glob("*" + p):
            f.append(folderName + '/')
        folderList.append(f)
    
    # Retrieve data of energy minimization
    gtRecorded = False
    escapeCosts = []
    for j,planner in enumerate(planners):
        escapeCostEM = np.zeros((noFolder, noFrame))
        for i,folder in enumerate(folderList[j][:noFolder]):
            # Get energy data from single csv file
            energyData, _ = get_results_from_csv(folder)
            row = energyData[3]
            escapeCostEM[i,:] = np.asarray(row)

            # Get initial energy over frames
            if not gtRecorded:
                initEnergy = []
                groundTruthEscapeEnergy = []
                with open('{}/data.csv'.format(folder), 'r') as csvfile:
                    csvreader = csv.DictReader(csvfile)
                    for row in csvreader:
                        initEnergy.append(float(row['obj_pos_z']))
                        groundTruthEscapeEnergy.append(max(0.0, groundTruthZ-float(row['obj_pos_z'])))
                gtRecorded = 1
        escapeCosts.append(escapeCostEM)
    os.chdir('./../../../')

    # Retrieve indices of keyframes
    id = np.asarray(list(range(0, noFrame)))
    xticks = np.asarray(list(range(0, noFrame+1, 1)))
    
    # Retrieve mean and std
    escapeCostsMean = [np.nanmean(e, axis=0) for e in escapeCosts]
    escapeCostsStd = [np.nanstd(e, axis=0) for e in escapeCosts]

    ###### Escape energy plot
    _, ax = plt.subplots()
    for i,e in enumerate(escapeCosts):
        ax.plot(id, escapeCostsMean[i], '-', color=cls[i], linewidth=2, label=planners[i])
        # ax.fill_between(id, escapeCostsMean[i]-escapeCostsStd[i], escapeCostsMean[i]+escapeCostsStd[i], alpha=0.3, color=cls[i])
        ax.errorbar(id, escapeCostsMean[i], yerr=escapeCostsStd[i], color=cls[i], alpha=0.8, capsize=3, capthick=1)
    ax.plot(id, groundTruthEscapeEnergy, '-', color=cls[-1], linewidth=2, label='Measured reference')

    # Settings for plot
    # ax.set_xticks(xticks.astype(int))
    # ax.set_xlim(-0.02, noFrame-1)
    # ax.set_xlabel('Index of frames',fontsize=16)
    # ax.set_ylabel('Escape energy / J',fontsize=16)
    # plt.title('Accuracy of search algorithms over frames',fontsize=16)
    # plt.legend()
    # plt.savefig('{}test02-cost.png'.format(SaveFolderName), dpi=200)

    ###### Error plot
    escapeCostsErr = [(e-groundTruthEscapeEnergy)/groundTruthEscapeEnergy for e in escapeCosts]
    escapeCostsMeanErr = [np.nanmean(e, axis=0) for e in escapeCostsErr]
    escapeCostsStdErr = [np.nanstd(e, axis=0) for e in escapeCostsErr]

    _, ax = plt.subplots()
    for i,e in enumerate(escapeCostsErr):
        print(planners[i])
        print(np.mean(escapeCostsMeanErr[i]))
        print(np.std(escapeCostsStdErr[i]))
        ax.plot(id, escapeCostsMeanErr[i], '-', color=cls[i], linewidth=2, label=planners[i])
        # ax.fill_between(id, escapeCostsMeanErr[i]-escapeCostsStdErr[i], escapeCostsMeanErr[i]+escapeCostsStdErr[i], alpha=0.3, color=cls[i])
        plt.errorbar(id, escapeCostsMeanErr[i], yerr=escapeCostsStdErr[i], fmt='o', color=cls[i], alpha=0.8, capsize=3, capthick=1)

    # # Settings for plot
    # ax.set_xticks(xticks.astype(int))
    # ax.set_ylim(-0.02, 0.52)
    # ax.set_xlim(-0.02, noFrame-1)
    # ax.set_xlabel('Index of frames',fontsize=16)
    # ax.set_ylabel('Error',fontsize=16)
    # plt.title('Error of search algorithms w.r.t. measured reference',fontsize=16)
    # plt.legend()
    # plt.savefig('{}test02-error.png'.format(SaveFolderName), dpi=200)


def calculate_test04():
    '''Plot the comparisons of two searching algorithms (Bound shrink search and energy minimization search) over several frames.
    '''
    cls = get_colors()

    # calculate true escape energy
    r1 = 0.21
    r2 = .2717
    startDim = 3
    n = np.asarray(list(range(startDim,10)))
    dim = 2*n + 1
    k = np.asarray(list(range(startDim,10)))
    alpha = 180 / n
    alpha = alpha / 180 * np.pi
    R1 = r1 / np.cos(alpha)
    R2 = r2 / np.cos(alpha)
    s1 = 2 * r1 * np.tan(alpha)
    s2 = 2 * r2 * np.tan(alpha)
    E2 = 0.5 * k * n * (s2-s1)**2

    # Get folder names
    os.chdir("results/ICRA2024/Test04BandHourglassNumCtrlPnt")
    error_list = []
    with open('bitstar_300sec.csv', 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        for k,row in enumerate(csvreader):
            escape_energy_estimate = [float(j) for j in row[1:]]
            error = np.asarray(escape_energy_estimate)
            # error = np.asarray(escape_energy_estimate) - E2[k]
            error_list.append(error)

    # Retrieve mean and std
    error_mean = np.asarray([np.nanmean(e, axis=0) for e in error_list])
    error_std = np.asarray([np.nanstd(e, axis=0) for e in error_list])

    # Plot
    _, ax = plt.subplots()
    ax1 = ax.twinx()
    ax.plot(n, E2, '-o', color=cls[0], linewidth=2, label='ref. energy')
    ax.plot(n, error_mean, '-o', color=cls[1], linewidth=2, label='est. energy')
    ax.fill_between(n, error_mean-error_std, error_mean+error_std, alpha=0.3, color=cls[1])
    # plt.errorbar(n, escapeCostsMeanErr[i], yerr=escapeCostsStdErr[i], fmt='o', color=cls[i], alpha=0.8, capsize=3, capthick=1)
    ax.plot(n, dim+100, '-*', color=cls[2], linewidth=2, label='dimension')
    ax1.plot(n, dim, '-*', color=cls[2], linewidth=2, label='dimension')

    # Settings for plot
    # ax.set_xticks(xticks.astype(int))
    ax.set_ylim(0.05, 0.7)
    ax.set_xlim(3,9)
    ax.set_xlabel('# control points',fontsize=14)
    ax.set_ylabel('escape energy',fontsize=14)
    ax1.set_ylabel('dim',fontsize=14)
    # ax.set_yscale('log')
    # ax.set_yticks([0.1,0.5])
    # plt.title('Error of escape energy w.r.t state space dimensionality',fontsize=16)
    ax.legend(fontsize=14,loc='upper left')
    # ax1.legend()
    plt.savefig('test04-band_pnt_dim.png', dpi=200)
    tikzplotlib.save('test04-band_pnt_dim.tex')
    os.chdir('./../../../')


'''Compare the convergence time of BIT* search and bisectional search over 8min and 3min search time, respectively, in one frame'''
'''Compare the accuracy of BIT* search and bisectional search over dozens of frames'''
# if __name__ == '__main__':
#     # Insert initial escape energy cost
#     initZ = 1.7285293405317828
#     groundTruthZ = 2.32
#     cInit = 2.7 - initZ

#     # Read from csv
#     folderName = 'results/ICRA2024/Test05HookRingErrorVsRuntime'
#     folderNameIncre = 'results/ICRA2024/Test05HookRingErrorVsRuntime/17-06-2023-08-29-07_incremental_600_75'
#     folderNameBinary = 'results/ICRA2024/Test05HookRingErrorVsRuntime/20-06-2023-14-47-44_binary_new_600_50'
#     folderNameBITs = 'results/ICRA2024/Test05HookRingErrorVsRuntime/bitstar_2sec_to_600sec'
#     inc = get_benckmark_results_from_csv(folderNameIncre, cInit, getOnlyOneFrame=1, noIter=10)
#     bin = get_benckmark_results_from_csv(folderNameBinary, cInit, getOnlyOneFrame=1, noIter=30)
#     bits = get_benckmark_results_from_csv(folderNameBITs, cInit, getOnlyOneFrame=1, noIter=10)

#     # Plot search algorithms convergence in 1 keyframe (frame 144 / keyframe 18 in Hook traps ring case)
#     groundTruthEscapeEnergy = groundTruthZ - initZ
#     # test05_get_structured_bits_data()
#     # plot_convergence_test(bits, bin, inc, folderName, groundTruthEscapeEnergy)

#     # Plot comparison of two algos over several keyframes (starting from frame 144 / keyframe 18)
#     startKeyFrame = 0
#     cInit = 10
#     folderNameBi = 'results/ICRA2024/Test01HookRing3BasicSearch/binary/20-06-2023-12-53-30_binary_new_200_35'
#     folderNameIn = 'results/ICRA2024/Test01HookRing3BasicSearch/11-06-2023-00-35-33_incremental_200_50'
#     _, CostBiFrames = get_benckmark_results_from_csv(folderNameBi, cInit, getOnlyOneFrame=0)
#     _, CostInFrames = get_benckmark_results_from_csv(folderNameIn, cInit, getOnlyOneFrame=0)
#     # print('@@@@finalCostBSFrames', finalCostBSFrames)
#     folderName = 'results/ICRA2024/Test01HookRing3BasicSearch'
#     # plot_acurracy_test(CostBiFrames, CostInFrames, folderName, groundTruthZ, startKeyFrame,)

#     # Plot test02
#     folderName02 = './results/ICRA2024/Test02HookRingPlannersComp/raw_data/'
#     plot_test02(folderName02, groundTruthZ,)
#     # calculate_test04()
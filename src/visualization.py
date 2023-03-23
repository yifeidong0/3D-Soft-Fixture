from datetime import datetime
import os
# import subprocess
import glob
import csv
import numpy as np
from utils import flatten_nested_list
import matplotlib.pyplot as plt
from utils import *
from main import argument_parser
# import pybullet as p
# import pybullet_data
# import time

def record_data_init(sce, args, env):
    # get current time
    now = datetime.now()
    dt_string = now.strftime("%d-%m-%Y-%H-%M-%S") # dd/mm/YY H:M:S
    print("date and time =", dt_string)
    folderName = './results/{}_{}'.format(args.scenario, dt_string)
    os.mkdir(folderName)

    # create csv headers
    objJointNum = len(sce.objJointPosSce[0])
    obsJointNum = len(sce.obsJointPosSce[0])
    headerObjJoint = []
    headerObsJoint = []
    for j in range(objJointNum):
        headerObjJoint.append('obj_joint_{}_pos'.format(j))
    for s in range(obsJointNum):
        headerObsJoint.append('obs_joint_{}_pos'.format(s))

    headersObj = ['index', 'obj_pos_x', 'obj_pos_y', 'obj_pos_z', 
               'obj_qtn_0', 'obj_qtn_1', 'obj_qtn_2', 'obj_qtn_3'] + headerObjJoint
    headersObs = ['obs_pos_x', 'obs_pos_y', 'obs_pos_z', 
               'obs_qtn_0', 'obs_qtn_1', 'obs_qtn_2', 'obs_qtn_3'] + headerObsJoint
    headersOther = ['start_energy', 'start_gravity_energy', 'start_elastic_energy', 'escape_energy_cost']
    headers = headersObj + headersObs + headersOther

    # write headers to csv
    with open('{}/data.csv'.format(folderName), 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(i for i in headers)
        # writer.writerows(data)

    # save other info to txt
    with open('{}/info.txt'.format(folderName), "w") as text_file:
        text_file.write("*****Search goalCoMPose: {}\n".format(sce.goalCoMPose))
        text_file.write("*****Search basePosBounds: {}\n".format(sce.basePosBounds))
        text_file.write("*****args: {}\n".format(args))
        text_file.write("*****BIT params: {}".format(env.pb_ompl_interface.planner.params()))
        if args.object == 'Fish':
            text_file.write("*****Fish links' mass: {}\n".format(env.pb_ompl_interface.potentialObjective.masses))
            text_file.write("*****Fish joints' stiffness: {}\n".format(env.pb_ompl_interface.potentialObjective.stiffnesss))

    return folderName

def record_data_loop(sce, energyData, folderName, i):
    data = flatten_nested_list([
        [sce.idxSce[i]], sce.objBasePosSce[i], sce.objBaseQtnSce[i],
        sce.objJointPosSce[i], sce.obsBasePosSce[i], sce.obsBaseQtnSce[i],
        sce.obsJointPosSce[i], energyData
        ])

    with open('{}/data.csv'.format(folderName), 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(data)

def analyze_energy_data(folderList, isArticulatedObject):
    # Get energy data from multiple csv files
    energyDataList = []
    minDataLen = np.inf
    for f in folderList:
        # Get energy data from single csv file
        energyData, _ = get_results_from_csv(f, isArticulatedObject)
        energyDataList.append(energyData)
        minDataLen = min(minDataLen, len(energyData[3]))

    # unpack the piled datasets
    totalEnergySce = energyDataList[0][0][:minDataLen]
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

    # get min, mean, std
    minTot, minG, minE, minEsc = totalEnergySce, np.min(GEnergySce,axis=0), np.min(EEnergySce,axis=0), np.min(escapeEnergyCostSce,axis=0)
    meanTot, meanG, meanE, meanEsc = None, np.nanmean(GEnergySce,axis=0), np.nanmean(EEnergySce,axis=0), np.nanmean(escapeEnergyCostSce,axis=0)
    stdTot, stdG, stdE, stdEsc = None, np.nanstd(GEnergySce,axis=0), np.nanstd(EEnergySce,axis=0), np.nanstd(escapeEnergyCostSce,axis=0)
    
    return (minTot, minG, minE, minEsc, meanTot, meanG, meanE, meanEsc, stdTot, stdG, stdE, stdEsc), minDataLen

def plot_escape_energy(ax, energyDataAnalysis, minDataLen, isArticulatedObject=False, axvline=None):
    # TODO: use broken axis to represent inf.
    # Color codes
    cls = get_colors()
    IterId = np.asarray(list(range(minDataLen)))

    minTot, minG, minE, minEsc, _, meanG, meanE, meanEsc, _, stdG, stdE, stdEsc = energyDataAnalysis

    # Plot min escape energy
    ax.plot(IterId, minTot, '-', color=cls[0], linewidth=2, label='Total energy')
    if isArticulatedObject:
        ax.plot(IterId, minG, '--', color=cls[1], label='Gravity potential energy')
        ax.plot(IterId, minE, '--', color=cls[2], label='Elastic potential energy')
    ax.plot(IterId, minEsc, '-', color=cls[3], linewidth=2, label='Escape energy cost')

    # Plot std shade
    if isArticulatedObject:
        ax.fill_between(IterId, meanG-stdG, meanG+stdG, alpha=0.4, color=cls[1])
        ax.fill_between(IterId, meanE-stdE, meanE+stdE, alpha=0.4, color=cls[2])
    ax.fill_between(IterId, meanEsc-stdEsc, meanEsc+stdEsc, alpha=0.4, color=cls[3])
    
    # Optional: plot vertical line of current no. of iteration
    if axvline is not None:
        ax.axvline(x=axvline, color='k')

    ax.set_xlabel('# iterations',fontsize=14)
    ax.set_ylabel('Potential energy',fontsize=14)
    ax.set_aspect(24)
    ax.grid(True)
    ax.legend()
    ax.set_xticks(np.arange(0,minDataLen,10).astype(int))

def get_results_from_csv(folderName, isArticulatedObject=False):
    indices = []
    start_energy = []
    start_gravity_energy = []
    start_elastic_energy = []
    escape_energy_cost = []

    with open('{}/data.csv'.format(folderName), 'r') as csvfile:
        csvreader = csv.DictReader(csvfile)

        for row in csvreader:
            indices.append(float(row['index']))
            start_energy.append(float(row['start_energy']))
            escape_energy_cost.append(float(row['escape_energy_cost']))
            if isArticulatedObject:
                start_gravity_energy.append(float(row['start_gravity_energy']))
                start_elastic_energy.append(float(row['start_elastic_energy']))
    energy_data = (start_energy, start_gravity_energy, start_elastic_energy, escape_energy_cost)

    return energy_data, indices

# def plot_escape_energy_from_csv(args, folderName, isArticulatedObject=False):
#     '''Plot escape energy after running the algorithm.
#     '''
#     # get energy data from csv
#     energy_data, _ = get_results_from_csv(folderName, isArticulatedObject)
    
#     # escape energy plots
#     plot_escape_energy(energy_data, args, folderName, isArticulatedObject)

def plot_escape_energy_from_multi_csv(ax, folderList, isArticulatedObject=False, axvline=None):
    '''Plot escape energy after running the algorithm.
    '''
    # Get min, mean, std of data
    energyDataAnalysis, minDataLen = analyze_energy_data(folderList, isArticulatedObject)

    # Escape energy plots
    plot_escape_energy(ax, energyDataAnalysis, minDataLen,isArticulatedObject, axvline)


if __name__ == '__main__':
    args, parser = argument_parser()
    rigidObjectList = get_non_articulated_objects()
    isArticulatedObj = False if args.object in rigidObjectList else True
    
    folderList = []
    path = './results/'
    os.chdir(path)
    for file_name in glob.glob("GripperClenchesStarfish*"):
        folderList.append(file_name)

    _, ax = plt.subplots()
    plot_escape_energy_from_multi_csv(ax, folderList, isArticulatedObj)
    plt.title('Escape energy in a dynamic scenario - {}'.format(args.scenario))
    # plt.show()
    plt.savefig('{}/energy_plot_std.png'.format(folderList[0]))
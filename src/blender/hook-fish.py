
from curses import keyname
import bpy
import math
import csv
import numpy as np
import pybullet as p

'''
Insert keyframes for a movement primitive of a hooking fish scenario. The data came from Pybullet-OMPL algorithms.
Run the script in Blender background mode:
/snap/bin$ ./blender -b ~/Documents/blender-models/rope.blend -P ~/Documents/KTH/git/3D-Energy-Bounded-Caging/src/blenderScript.py
'''

def add_a_keyframe(objects: list, 
                   data: list, 
                   i: int, 
                   ) -> None:
    '''
    Add pose information of all moving components in a keyframe. 
        i:
            int, frame ID.
    '''
    # Update body pose
    fish, hook, hand = objects
    fish.location = tuple(data[1:4])
    xyzw = tuple(data[4:8])
    fish.rotation_quaternion = (xyzw[-1],) + xyzw[:3]

    hook.location = tuple(data[8:11])
    xyzw = tuple(data[11:15])
    hook.rotation_quaternion = (xyzw[-1],) + xyzw[:3]

    hand.location = tuple(data[15:])

    # Insert keyframes
    fish.keyframe_insert(data_path="location", frame=i)
    fish.keyframe_insert(data_path="rotation_quaternion", frame=i)
    hook.keyframe_insert(data_path="location", frame=i)
    hook.keyframe_insert(data_path="rotation_quaternion", frame=i)
    hand.keyframe_insert(data_path="location", frame=i)

    # Update the viewport
    bpy.context.view_layer.update()

def add_keyframes(dataFolderPath: str, objNames: list[str]) -> None:
    '''
    Read data from source csv and invoke per-frame keyframe adding function.
    '''
    objs = [bpy.data.objects[objNames[i]] for i in range(len(objNames))]
    objs[0].rotation_mode = 'QUATERNION'
    objs[1].rotation_mode = 'QUATERNION'

    with open('{}/data.csv'.format(dataFolderPath), 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        i = 0
        for row in csvreader:
            if i == 0: # skip the headers
                i += 1
                continue
            data = [float(d) for d in row]
            add_a_keyframe(objs, data, i)
            i += 1


'''Main'''
objNames = ['fish', 'hook', 'hand']
dataFolderPath = '/home/yif/Documents/KTH/git/3D-Energy-Bounded-Caging/results/HookFishHole_15-05-2023-18-24-46_dynamics'
add_keyframes(dataFolderPath, objNames)

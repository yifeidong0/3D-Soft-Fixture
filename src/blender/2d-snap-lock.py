
import bpy
import math
import csv
# from blenderUtils import *
'''Run the script in Blender background mode:
/snap/bin$ ./blender -b ~/Documents/blender-models/rope.blend -P ~/Documents/KTH/git/3D-Energy-Bounded-Caging/src/blenderScript.py
'''

def add_a_keyframe(objects: list, data: list, i: int) -> None:
    '''Add pose information of all moving components in a keyframe. 
        data: 
            list[x,y,theta,joint_angle], containing pose of base link and the joint angle.
        i:
            int, frame ID.
    '''
    # Update lattice points coordinates and insert keyframes
    objects[0].location = tuple(data[:2]) + (0.0,)
    objects[0].rotation_euler = (0.0, 0.0,) + (data[2],)
    objects[1].location = tuple(data[:2]) + (0.0,)
    objects[1].rotation_euler = (0.0, 0.0,) + (data[2]+math.radians(90)+data[3],)

    # Insert keyframes
    objects[0].keyframe_insert(data_path="location", frame=i)
    objects[0].keyframe_insert(data_path="rotation_euler", frame=i)
    objects[1].keyframe_insert(data_path="location", frame=i)
    objects[1].keyframe_insert(data_path="rotation_euler", frame=i)

    # Update the viewport
    bpy.context.view_layer.update()

def add_keyframes(dataFolderPath: str, objNames: list[str]) -> None:
    '''Read data from source csv and invoke per-frame keyframe adding function.
    '''
    objs = [bpy.data.objects[objNames[i]] for i in range(len(objNames))]

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

def set_2dkey_pose(objName: str, pose: list[3]) -> None:
    obj = bpy.data.objects[objName]
    obj.location = tuple(pose[:2]) + (0.0,)
    obj.rotation_euler = (0.0, 0.0,) + (pose[2],)


'''Main loop'''
lockNames = ['p1.001', 'p2.001']
keyName = 'p3.001'
keyPose = [1.25, -2.9, 3.7]
dataFolderPath = '/home/yif/Documents/KTH/git/3D-Energy-Bounded-Caging/results/2DSnapLock_05-05-2023-12-17-57_escape_path_4blender'

set_2dkey_pose(keyName, keyPose)
add_keyframes(dataFolderPath, lockNames)

# reset jelly
# zstart = 0
# reset_jelly(objName, zstart)

from curses import keyname
import bpy
import csv

'''
Insert keyframes for a movement primitive of a catching ring scenario. The data came from Pybullet-OMPL algorithms.
Run the script in Blender background mode:
/snap/bin$ ./blender -b ~/Documents/blender-models/rope.blend -P ~/Documents/KTH/git/3D-Energy-Bounded-Caging/src/blenderScript.py
'''

def add_a_keyframe(obj, 
                   data: list, 
                   i: int, 
                   ) -> None:
    '''
    Add pose information of all moving components in a keyframe. 
        data: 
            list[x,y,z,r,p,y,joint_angle], containing pose of base link and the joint angle.
        i:
            int, frame ID.
    '''
    # Update body pose
    obj.location = tuple(data[:3])
    obj.rotation_euler = tuple(data[3:])

    # Insert keyframes
    obj.keyframe_insert(data_path="location", frame=i)
    obj.keyframe_insert(data_path="rotation_euler", frame=i)

    # Update the viewport
    bpy.context.view_layer.update()

def add_keyframes(dataFolderPath: str, objNames: str) -> None:
    '''
    Read data from source csv and invoke per-frame keyframe adding function.
    '''
    obj = bpy.data.objects[objNames]

    with open('{}/data.csv'.format(dataFolderPath), 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        i = 0
        for row in csvreader:
            if i == 0: # skip the headers
                i += 1
                continue
            data = [float(d) for d in row]
            add_a_keyframe(obj, data, i)
            i += 1

def set_hook_pose(objName: str, pose: list[6]) -> None:
    obj = bpy.data.objects[objName]
    obj.location = tuple(pose[:3])
    obj.rotation_euler = tuple(pose[3:])

hookName = 'triple_hook'
ringName = 'ring2'
hookPose = [0, 0, 2, 1.57, -0.3, 0]
dataFolderPath = '/home/yif/Documents/KTH/git/3D-Energy-Bounded-Caging/results/HookTrapsRing_08-05-2023-09-49-14_escape_path_4blender'
ratio = 0.6

set_hook_pose(hookName, hookPose)
add_keyframes(dataFolderPath, ringName)
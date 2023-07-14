
from curses import keyname
import bpy
import math
import csv
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R
'''
Insert keyframes for an escape path demo of a 3D snap lock scenario. The data came from Pybullet-OMPL algorithms.
Run the script in Blender background mode:
/snap/bin$ ./blender -b ~/Documents/blender-models/rope.blend -P ~/Documents/KTH/git/3D-Energy-Bounded-Caging/src/blenderScript.py
'''

def get_mesh_data(objectName: str) -> None:
    # Get a reference to the active object in the scene
    obj = bpy.data.objects[objectName]

    # Get the total number of vertices in the object
    num_vertices = len(obj.data.vertices)

    # Get the total number of faces in the object
    num_faces = len(obj.data.polygons)

    # Print the results
    print("Total vertices:", num_vertices)
    print("Total faces:", num_faces)

def cut_num_polygon(ratio: float, objectName: str) -> None:
    # Get the active object
    obj = bpy.data.objects[objectName]

    # Add a Decimate modifier to the object
    decimate_mod = obj.modifiers.new(name='Decimate', type='DECIMATE')

    # Set the Decimate ratio to 0.5 (50% reduction in polygon count)
    decimate_mod.ratio = ratio

    # Apply the modifier
    bpy.ops.object.modifier_apply(modifier=decimate_mod.name)


def add_a_keyframe(objects: list, 
                   data: list, 
                   i: int, 
                   rotOffset: tuple = (math.radians(-15),0,0), 
                   jointPosInBody: np.ndarray = np.array([[0.],[-0.66],[0.39],[1]]),
                   transformLastRow: np.ndarray = np.array([[0,0,0,1]])
                   ) -> None:
    '''
    Add pose information of all moving components in a keyframe. 
        data: 
            list[x,y,z,r,p,y,joint_angle], containing pose of base link and the joint angle.
        i:
            int, frame ID.
    '''
    # Update body pose
    objects[0].location = tuple(data[:3])
    objects[0].rotation_euler = tuple([data[3:6][i]+rotOffset[i] for i in range(3)])
    
    # Calculate arm frame (joint frame) origin position in world frame     
    baseQuatInWorld = p.getQuaternionFromEuler(data[3:6])
    r = R.from_quat(baseQuatInWorld)
    mat: np.ndarray = r.as_matrix() # 3*3, 
    mat = np.hstack((mat, np.asarray(data[:3]).reshape((3,1)))) # 4*3
    transformWorld2Body = np.vstack((mat, transformLastRow)) # 4*4
    jointPosInWorld = transformWorld2Body @ jointPosInBody # 4*1

    # Update arm pose
    objects[1].location = tuple(jointPosInWorld.flatten())[:3]
    jointRotOffset = (data[-1], 0.0, 0.0)
    objects[1].rotation_euler = tuple([data[3:6][i]+rotOffset[i]+jointRotOffset[i] for i in range(3)])

    # Insert keyframes
    objects[0].keyframe_insert(data_path="location", frame=i)
    objects[0].keyframe_insert(data_path="rotation_euler", frame=i)
    objects[1].keyframe_insert(data_path="location", frame=i)
    objects[1].keyframe_insert(data_path="rotation_euler", frame=i)

    # Update the viewport
    bpy.context.view_layer.update()

def add_keyframes(dataFolderPath: str, objNames: list[str]) -> None:
    '''
    Read data from source csv and invoke per-frame keyframe adding function.
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

def set_ring_pose(objName: str, pose: list[6]) -> None:
    obj = bpy.data.objects[objName]
    obj.location = tuple(pose[:3])
    obj.rotation_euler = tuple(pose[3:])


'''Main'''
lockNames = ['body', 'arm']
ringName = 'ring2'
ringPose = [-.5, 0, 3, 0, 0, 0]
dataFolderPath = '/home/yif/Documents/KTH/git/3D-Energy-Bounded-Caging/results/3DSnapLock_05-05-2023-17-59-32_escape_path_4blender'
ratio = 0.6

set_ring_pose(ringName, ringPose)
add_keyframes(dataFolderPath, lockNames)

# Set 3D cursor position
# bpy.context.scene.cursor.location = (0.0,-0.66,0.39)

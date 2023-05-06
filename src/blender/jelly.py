
import bpy
import math
import csv
# from blenderUtils import *
'''Run the script in Blender background mode:
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
    # Get the object
    obj = bpy.data.objects[objectName]

    # Add a Decimate modifier to the object
    decimate_mod = obj.modifiers.new(name='Decimate', type='DECIMATE')

    # Set the Decimate ratio to 0.5 (50% reduction in polygon count)
    decimate_mod.ratio = ratio

    # Apply the modifier
    bpy.ops.object.modifier_apply(modifier=decimate_mod.name)

def reset_jelly(objName, zstart):
    # Get the lattice object
    lattice_obj = bpy.data.objects[objName]

    # Get the lattice data
    lattice = lattice_obj.data

    original_positions = [(-.5,-.5,zstart-.5,), (.5,-.5,zstart-.5,), (-.5,.5,zstart-.5,), (.5,.5,zstart-.5,), 
                          (-.5,-.5,zstart+.5,), (.5,-.5,zstart+.5,), (-.5,.5,zstart+.5,), (.5,.5,zstart+.5,)]
    for k in range(8):
        point = lattice.points[k]
        point.co_deform = original_positions[k]

    # Update the viewport
    bpy.context.view_layer.update()

def add_a_keyframe(lattice, data: list, i: int, 
                   point_indices: list=[0,4,5,6], follower_indices: list=[1,2,3,7]) -> None:
    ''' 
        data: 
            list[x1,y1,z1,...,xn,yn,zn], positions of jelly tetrahedron control points.
        point_indices:
            list, indices of lattice corner points (8 in total) that corresponds to jelly control points.
        i:
            int, frame ID.
    '''
    # Update lattice points coordinates and insert keyframes
    order = 0
    for k in point_indices:
        point = lattice.points[k]
        point.co_deform = tuple(data[3*order:3*order+3])
        point.keyframe_insert(data_path="co_deform", frame=i)
        order += 1
    
    # Update lattice follower points coordinates
    for k in follower_indices:
        point = lattice.points[k]
        if k == 7:
            point.co_deform = [(data[6:9][i]+data[9:][i])/2 for i in range(3)] # follow mid of point 5 and 6
        else:
            point.co_deform = tuple(data[:3]) # follow the point 0
        point.keyframe_insert(data_path="co_deform", frame=i)

    # Update the viewport
    bpy.context.view_layer.update()

def add_keyframes(dataFolderPath: str, objName: str) -> None:
    # Get the lattice object
    lattice_obj = bpy.data.objects[objName]

    # Get the lattice data
    lattice = lattice_obj.data

    with open('{}/data.csv'.format(dataFolderPath), 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        i = 0
        for row in csvreader:
            if i == 0: # skip the headers
                i += 1
                continue
            data = [float(d) for d in row]
            add_a_keyframe(lattice, data, i)
            i += 1

'''Main loop'''
objName = 'maze_bottle'
ratio = 0.1
# get_mesh_data(objName)
# cut_num_polygon(ratio, objName)

objName = 'Lattice'

# data file foe 4-control-point jello monkey 
dataFolderPath = '/home/yif/Documents/KTH/git/3D-Energy-Bounded-Caging/results/JellyMaze_04-05-2023-11-46-33_escape_path_4blender'
add_keyframes(dataFolderPath, objName)

# reset jelly
zstart = 0
# reset_jelly(objName, zstart)
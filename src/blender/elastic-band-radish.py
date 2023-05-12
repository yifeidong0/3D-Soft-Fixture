
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

def remove_animation():
    # Get the current scene
    scene = bpy.context.scene

    # Loop through all the objects in the scene
    for obj in scene.objects:
        # Clear animation data
        obj.animation_data_clear()


def add_a_keyframe(points, pointPositions: list, i: int, numCtrlPnt=6) -> None:
    '''pointPositions: 
            list[x1,y1,z1,...,xn,yn,zn], positions of band control points.
    '''
    for k,point in enumerate(points):
        point.co = tuple(pointPositions[3*k:3*k+3])
        # point.handle_left_type = 'VECTOR'
        # point.handle_right_type = 'VECTOR'
        point.keyframe_insert(data_path="co", frame=i)

        # Set the position of the handles for the control point
        prev_point = points[k-1]
        if k == numCtrlPnt-1:
            next_point = points[0]
        else:
            next_point = points[k+1]
        handle_position = ((next_point.co - point.co) + (point.co - prev_point.co)) * 0.165
        point.handle_left = point.co - handle_position
        point.handle_right = point.co + handle_position
        point.handle_left_type = 'FREE'
        point.handle_right_type = 'FREE'

        point.keyframe_insert(data_path="handle_left", frame=i)
        point.keyframe_insert(data_path="handle_right", frame=i)
        point.keyframe_insert(data_path="handle_left_type", frame=i)
        point.keyframe_insert(data_path="handle_right_type", frame=i)


def add_keyframes(dataFolderPath: str, curveName: str) -> None:
    # Get a reference to the curve object
    obj = bpy.data.objects[curveName]
    bpy.context.view_layer.objects.active = obj
    curve = obj.data
    points = curve.splines[0].bezier_points

    with open('{}/data.csv'.format(dataFolderPath), 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        i = 0
        for row in csvreader:
            if i == 0: # skip the headers
                i += 1
                continue
            pointPositions = [float(d) for d in row]
            add_a_keyframe(points, pointPositions, i)
            i += 1

'''Main loop'''
# objName = 'hourglass'
# ratio = 0.8
# get_mesh_data(objName)
# cut_num_polygon(ratio, objName)

curveName = 'BezierCircle'
remove_animation()

# data file foe 6-control-point elastic band 
dataFolderPath = '/home/yif/Documents/KTH/git/3D-Energy-Bounded-Caging/results/4blender/band-radish/BandRadish_12-05-2023-15-22-20_escape_path_4blender'
add_keyframes(dataFolderPath, curveName)
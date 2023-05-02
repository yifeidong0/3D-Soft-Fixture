
import bpy
import math
import csv

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
    # Get the active object
    obj = bpy.data.objects[objectName]

    # Add a Decimate modifier to the object
    decimate_mod = obj.modifiers.new(name='Decimate', type='DECIMATE')

    # Set the Decimate ratio to 0.5 (50% reduction in polygon count)
    decimate_mod.ratio = ratio

    # Apply the modifier
    bpy.ops.object.modifier_apply(modifier=decimate_mod.name)

def get_bones_and_set_mode(armature) -> list:
    # Get a reference to the pose bones in the armature
    bone_names = ['Bone01', 'Bone02', 'Bone03', 'Bone04', 'Bone05', 
                  'Bone06', 'Bone07']
    bones = [armature.pose.bones[n] for n in bone_names]
    bones[0].rotation_mode = 'XYZ' # tail/base component
    for bone in bones[1:]:
        bone.rotation_mode = 'XYZ'

    return bones

def reset_rope(armatureName):
    # Get the active object
    armature = bpy.data.objects[armatureName]

    # Get the animation data for the object
    anim_data = armature.animation_data

    # If the animation data exists, remove all the previous keyframes
    if anim_data is not None:
        for fcurve in anim_data.action.fcurves:
            fcurve.keyframe_points.clear()
            
    # Enter pose mode for the armature
    bpy.context.view_layer.objects.active = armature
    bpy.ops.object.mode_set(mode='POSE')

    # Get bones and set rotational modes
    bones = get_bones_and_set_mode(armature)

    bones[0].location = (0.0, 0.0, 0.0)
    bones[0].rotation_euler = (0.0, 0.0, 0.0)
    for k,bone in enumerate(bones[1:]):
        bone.rotation_euler = (0,0,0)

def add_a_keyframe(armature, position: list, rotation_euler: list, joint_positions: list, i: int) -> None:
    '''joint_positions: 
            list[x1,z1,x2,z2...,xn,zn], xi, zi are the 1st and 3rd euler angle at joint i
    '''
    # Enter pose mode for the armature
    bpy.ops.object.mode_set(mode='POSE')

    # Get bones and set rotational modes
    bones = get_bones_and_set_mode(armature)
    
    # Rotate the bone
    bones[0].rotation_euler = tuple(rotation_euler)
    bones[0].location = tuple(position)
    for k,bone in enumerate(bones[1:]):
        bone.rotation_euler = (joint_positions[2*k],0,joint_positions[2*k+1])

    # Insert a keyframe
    bpy.ops.object.mode_set(mode='OBJECT')
    bones[0].keyframe_insert(data_path="rotation_euler" ,frame=i)
    bones[0].keyframe_insert(data_path="location" ,frame=i)
    for bone in bones[1:]:
        bone.keyframe_insert(data_path="rotation_euler" ,frame=i)

def add_keyframes(dataFolderPath: str, armatureName: str) -> None:
    # Get a reference to the armature object
    armature = bpy.data.objects[armatureName]
    
    # Set the transform orientation to global
    bpy.context.scene.transform_orientation_slots[0].type = 'GLOBAL'
    
    # Enter pose mode for the armature
    bpy.context.view_layer.objects.active = armature

    with open('{}/data.csv'.format(dataFolderPath), 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        i = 1
        for row in csvreader:
            if i == 1: # skip the headers
                i += 1
                continue
            data = [float(d) for d in row]
            position = data[:3]
            rotation_euler = data[3:6]
            joint_positions = data[6:]
            add_a_keyframe(armature, position, rotation_euler, joint_positions, i)
            i += 1


'''Main loop'''
objName = 'bucket'
ratio = 0.8
# get_mesh_data(objName)
# cut_num_polygon(ratio, objName)

armatureName = 'Armature'
reset_rope(armatureName)

# data file foe 6-control-point rope (7 bones) 
dataFolderPath = '/home/yif/Documents/KTH/git/3D-Energy-Bounded-Caging/results/RopeBucket_02-05-2023-11-52-53_escape_path_4blender'

add_keyframes(dataFolderPath, armatureName)

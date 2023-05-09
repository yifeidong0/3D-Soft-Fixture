
import bpy
import math
import csv
import mathutils
'''Run the script in Blender background mode:
/snap/bin$ ./blender -b ~/Documents/blender-models/rope.blend -P ~/Documents/KTH/git/3D-Energy-Bounded-Caging/src/blenderScript.py
'''
BASE_BONES = ['Bone11', 'Bone21', 'Bone31', 'Bone41', 'Bone51']
SECOND_BONES = ['Bone12', 'Bone22', 'Bone32', 'Bone42', 'Bone52']

# Get the transformations from starfish base frame to base bone frame 
Z_ANGLES = [0, -68.7374, 79.3758, -148.038, 150.714]
T12s = [mathutils.Matrix.Rotation(math.radians(i), 4, 'Z') for i in Z_ANGLES]

def get_bones_and_set_mode(armature):
    # Get a reference to the pose bones in the armature
    base_bones = [armature.pose.bones[n] for n in BASE_BONES]
    second_bones = [armature.pose.bones[n] for n in SECOND_BONES]
    for bone in base_bones:
        bone.rotation_mode = 'QUATERNION'
    for bone in second_bones:
        bone.rotation_mode = 'XYZ'

    return base_bones, second_bones

def reset_fish(armatureName):
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
    base_bones, second_bones = get_bones_and_set_mode(armature)

    # Reset bone poses
    for k,bone in enumerate(base_bones):
        bone.location = (0.0, 0.0, 0.0)
        bone.rotation_quaternion = (1,0,0,0)
    for k,bone in enumerate(second_bones):
        bone.rotation_euler = (0,0,0)

def add_a_keyframe(armature, position, quaternion_xyzw, joint_positions, i):
    # Enter pose mode for the armature
    bpy.ops.object.mode_set(mode='POSE')

    # Get bones and set rotational modes
    base_bones, second_bones = get_bones_and_set_mode(armature)
    
    # Set base bone pose
    xyzw = tuple(quaternion_xyzw)
    q = mathutils.Quaternion((xyzw[-1],) + xyzw[:3])
    Tq = q.to_matrix().to_4x4()
    Tt = mathutils.Matrix.Translation(tuple(position))
    Tw1 = Tt @ Tq # world to starfish base frame

    for k,bone in enumerate(base_bones):
        Tw2 = Tw1 @ T12s[k]
        bone.matrix = Tw2

    # Set second bone rotation
    for k,bone in enumerate(second_bones):
        bone.rotation_euler = (joint_positions[2*k+1], 0, joint_positions[2*k])

    # Insert a keyframe
    bpy.ops.object.mode_set(mode='OBJECT')
    for bone in base_bones:
        bone.keyframe_insert(data_path="rotation_quaternion" ,frame=i)
        bone.keyframe_insert(data_path="location" ,frame=i)
        # bone.keyframe_insert(data_path="matrix" ,frame=i)
    for bone in second_bones:
        bone.keyframe_insert(data_path="rotation_euler" ,frame=i)

def add_keyframes(dataFolderPath, armatureName):
    # Get a reference to the armature object
    armature = bpy.data.objects[armatureName]
    
    # Set the transform orientation to global
    # bpy.context.scene.transform_orientation_slots[0].type = 'GLOBAL'
    
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
            position = data[1:4]
            quaternion_xyzw = data[4:8]
            joint_positions = data[8:]
            add_a_keyframe(armature, position, quaternion_xyzw, joint_positions, i)
            i += 1


'''Main loop'''
# objName = 'starfish_lod3'
# ratio = 0.5
# get_mesh_data(objName)
# cut_num_polygon(ratio, objName)

armatureName = 'Armature'
# reset_fish(armatureName)

dataFolderPath = '/home/yif/Documents/KTH/git/3D-Energy-Bounded-Caging/results/StarfishSplashBowl_09-05-2023-13-14-58_4blender'

add_keyframes(dataFolderPath, armatureName)




################ Test

#import bpy
#import math
#import csv
#import mathutils

#BASE_BONES = ['Bone11', 'Bone21', 'Bone31', 'Bone41', 'Bone51']
#SECOND_BONES = ['Bone12', 'Bone22', 'Bone32', 'Bone42', 'Bone52']

#def get_bones_and_set_mode(armature):
#    # Get a reference to the pose bones in the armature
#    base_bones = [armature.pose.bones[n] for n in BASE_BONES]
#    second_bones = [armature.pose.bones[n] for n in SECOND_BONES]
#    for bone in base_bones:
#        bone.rotation_mode = 'XYZ'
#    for bone in second_bones:
#        bone.rotation_mode = 'XYZ'

#    return base_bones, second_bones

#armatureName = 'Armature'
#armature = bpy.data.objects[armatureName]

#bpy.ops.object.mode_set(mode='POSE')

#bpy.context.view_layer.objects.active = armature

#base_bones, second_bones = get_bones_and_set_mode(armature)


## Create a rotation matrix around the X axis
#R12 = mathutils.Matrix.Rotation(math.radians(0), 4, 'Z') # given

#Rw1 = mathutils.Matrix.Rotation(math.radians(0), 4, 'X') # data

## Set the bone's matrix to the rotation matrix
#base_bones[4].matrix = Rw1 @ R12 # Rw2

##base_bones[1].rotation_euler.rotate_axis('X', math.radians(45))

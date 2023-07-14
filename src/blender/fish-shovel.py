
import bpy
import math
import csv
'''
Insert keyframes for a movement primitive of a scooping fish scenario. The data came from Pybullet-OMPL algorithms.
Run the script in Blender background mode:
/snap/bin$ ./blender -b ~/Documents/blender-models/rope.blend -P ~/Documents/KTH/git/3D-Energy-Bounded-Caging/src/blenderScript.py
'''

NUM_FISH_JOINT = 9

def get_bones_and_set_mode(armature):
    # Get a reference to the pose bones in the armature
    bone_names = ['Bone01', 'Bone02', 'Bone03', 'Bone04', 'Bone05', 
                  'Bone06', 'Bone07', 'Bone08', 'Bone09', 'Bone10']
    bones = [armature.pose.bones[n] for n in bone_names]
    bones[0].rotation_mode = 'QUATERNION' # tail/base component
    for bone in bones[1:]:
        bone.rotation_mode = 'XYZ'

    return bones

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
    bones = get_bones_and_set_mode(armature)

    bones[0].location = (0.0, 0.0, 0.0)
    bones[0].rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
    for k,bone in enumerate(bones[1:]):
        bone.rotation_euler = (0,0,0)

def add_a_keyframe(fishArmature, shovel, data, i):
    # Get pose info of fish and shovel
    posFish = data[1:4]
    quatFish = data[4:8]
    jointFish = data[8:8+NUM_FISH_JOINT]
    posShovel = data[8+NUM_FISH_JOINT:8+NUM_FISH_JOINT+3]
    quatShovel = data[8+NUM_FISH_JOINT+3:]

    # Set shovel pose
    shovel.location = tuple(posShovel)
    xyzw = tuple(quatShovel)
    shovel.rotation_quaternion = (xyzw[-1],) + xyzw[:3]

    # Enter pose mode for the armature
    bpy.ops.object.mode_set(mode='POSE')

    # Get bones and set rotational modes
    bones = get_bones_and_set_mode(fishArmature)
    
    # Rotate the bone - Pybullet - XYZW, Blender - WXYZ
    xyzw = tuple(quatFish)
    bones[0].rotation_quaternion = (xyzw[-1],) + xyzw[:3]
    bones[0].location = tuple(posFish)
    for k,bone in enumerate(bones[1:]):
        bone.rotation_euler = (jointFish[k],0,0)

    # Insert a keyframe
    bpy.ops.object.mode_set(mode='OBJECT')
    bones[0].keyframe_insert(data_path="rotation_quaternion" ,frame=i)
    bones[0].keyframe_insert(data_path="location" ,frame=i)
    for bone in bones[1:]:
        bone.keyframe_insert(data_path="rotation_euler" ,frame=i)
    shovel.keyframe_insert(data_path="location", frame=i)
    shovel.keyframe_insert(data_path="rotation_quaternion", frame=i)

def add_keyframes(dataFolderPath: str, objNames: list[str]) -> None:
    '''
    Read data from source csv and invoke per-frame keyframe adding function.
    '''
    fishArmature, shovel = [bpy.data.objects[objNames[i]] for i in range(len(objNames))]
    shovel.rotation_mode = 'QUATERNION'

    with open('{}/data.csv'.format(dataFolderPath), 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        i = 1
        for row in csvreader:
            if i == 1: # skip the headers
                i += 1
                continue
            data = [float(d) for d in row]
            add_a_keyframe(fishArmature, shovel, data, i)
            i += 1

def set_hand_pose(objName: str, pose: list[6]) -> None:
    obj = bpy.data.objects[objName]
    obj.location = tuple(pose[:3])
    obj.rotation_euler = tuple(pose[3:])

'''Main'''
armatureName = 'fishArmature'
shovelName = 'shovel'
handName = 'hand'
objNames = [armatureName, shovelName]
handPose = [.5,-1.6,1.8] + [-1.57,.2,-2.2]
dataFolderPath = '/home/yif/Documents/KTH/git/3D-Energy-Bounded-Caging/results/ShovelFish_17-05-2023-15-46-51_dynamics'

reset_fish(armatureName)
set_hand_pose(handName, handPose)
add_keyframes(dataFolderPath, objNames)

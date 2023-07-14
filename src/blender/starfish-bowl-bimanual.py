
import bpy
import math
import csv
import mathutils
'''
Insert keyframes for a movement primitive of a catching starfish scenario. The data came from Pybullet-OMPL algorithms.
Run the script in Blender background mode:
/snap/bin$ ./blender -b ~/Documents/blender-models/rope.blend -P ~/Documents/KTH/git/3D-Energy-Bounded-Caging/src/blenderScript.py
'''

BASE_BONES = ['Bone11', 'Bone21', 'Bone31', 'Bone41', 'Bone51']
SECOND_BONES = ['Bone12', 'Bone22', 'Bone32', 'Bone42', 'Bone52']
NUM_FISH_JOINT = 10

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
    # Get the current scene
    scene = bpy.context.scene

    # Loop through all the objects in the scene
    for obj in scene.objects:
        # Clear animation data
        obj.animation_data_clear()
            
    # Enter pose mode for the armature
    armature = bpy.data.objects[armatureName]
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

def add_a_keyframe(fishArmature, bowl, rightHand, data, i):
    # Extract info from data
    posFish = data[1:4]
    quatFish = data[4:8]
    jointFish = data[8:8+NUM_FISH_JOINT]
    posBowl = data[8+NUM_FISH_JOINT:8+NUM_FISH_JOINT+3]
    quatBowl = data[8+NUM_FISH_JOINT+3:8+NUM_FISH_JOINT+7]
    posRightHand = data[8+NUM_FISH_JOINT+7:8+NUM_FISH_JOINT+10]
    eulRightHand = data[8+NUM_FISH_JOINT+10:]

    # Set bowl pose
    bowl.location = tuple(posBowl)
    xyzw = tuple(quatBowl)
    bowl.rotation_quaternion = (xyzw[-1],) + xyzw[:3]

    # Set right hand pose
    rightHand.location = tuple(posRightHand)
    rightHand.rotation_euler = tuple(eulRightHand)

    # Enter pose mode for the armature
    bpy.ops.object.mode_set(mode='POSE')

    # Get bones and set rotational modes
    base_bones, second_bones = get_bones_and_set_mode(fishArmature)
    
    # Set base bone pose
    xyzw = tuple(quatFish)
    q = mathutils.Quaternion((xyzw[-1],) + xyzw[:3])
    Tq = q.to_matrix().to_4x4()
    Tt = mathutils.Matrix.Translation(tuple(posFish))
    Tw1 = Tt @ Tq # world to starfish base frame

    for k,bone in enumerate(base_bones):
        Tw2 = Tw1 @ T12s[k]
        bone.matrix = Tw2

    # Set second bone rotation
    for k,bone in enumerate(second_bones):
        bone.rotation_euler = (jointFish[2*k+1], 0, jointFish[2*k])

    # Insert a keyframe
    bpy.ops.object.mode_set(mode='OBJECT')
    for bone in base_bones:
        bone.keyframe_insert(data_path="rotation_quaternion" ,frame=i)
        bone.keyframe_insert(data_path="location" ,frame=i)
        # bone.keyframe_insert(data_path="matrix" ,frame=i)
    for bone in second_bones:
        bone.keyframe_insert(data_path="rotation_euler" ,frame=i)
    bowl.keyframe_insert(data_path="location", frame=i)
    bowl.keyframe_insert(data_path="rotation_quaternion", frame=i)
    rightHand.keyframe_insert(data_path="location", frame=i)
    rightHand.keyframe_insert(data_path="rotation_euler", frame=i)

def add_keyframes(dataFolderPath, objNames):
    # Get a reference to the armature object
    fishArmature, bowl, rightHand = [bpy.data.objects[objNames[i]] for i in range(len(objNames))]
    bowl.rotation_mode = 'QUATERNION'
    rightHand.rotation_mode = 'XYZ'

    with open('{}/data.csv'.format(dataFolderPath), 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        i = 1
        for row in csvreader:
            if i == 1: # skip the headers
                i += 1
                continue
            data = [float(d) for d in row]
            add_a_keyframe(fishArmature, bowl, rightHand, data, i)
            i += 1


'''Main'''
armatureName = 'Armature'
bowlName = 'joinedBowl'
handName = 'rightHand'
objNames = [armatureName, bowlName, handName]
reset_fish(armatureName)

dataFolderPath = '/home/yif/Documents/KTH/git/3D-Energy-Bounded-Caging/results/StarfishBowl_18-05-2023-20-54-19_dynamics'
add_keyframes(dataFolderPath, objNames)

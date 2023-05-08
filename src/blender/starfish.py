
import bpy
import math
import csv

'''Run the script in Blender background mode:
/snap/bin$ ./blender -b ~/Documents/blender-models/rope.blend -P ~/Documents/KTH/git/3D-Energy-Bounded-Caging/src/blenderScript.py
'''
def get_mesh_data(objectName):
    # Get a reference to the active object in the scene
    obj = bpy.data.objects[objectName]

    # Get the total number of vertices in the object
    num_vertices = len(obj.data.vertices)

    # Get the total number of faces in the object
    num_faces = len(obj.data.polygons)

    # Print the results
    print("Total vertices:", num_vertices)
    print("Total faces:", num_faces)

def cut_num_polygon(ratio, objectName):
    # Get the active object
    obj = bpy.data.objects[objectName]

    # Add a Decimate modifier to the object
    decimate_mod = obj.modifiers.new(name='Decimate', type='DECIMATE')

    # Set the Decimate ratio to 0.5 (50% reduction in polygon count)
    decimate_mod.ratio = ratio

    # Apply the modifier
    bpy.ops.object.modifier_apply(modifier=decimate_mod.name)

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

def add_a_keyframe(armature, position, quaternion_xyzw, joint_positions, i):
    # Enter pose mode for the armature
    bpy.ops.object.mode_set(mode='POSE')

    # Get bones and set rotational modes
    bones = get_bones_and_set_mode(armature)
    
    # Rotate the bone - Pybullet - XYZW, Blender - WXYZ
    xyzw = tuple(quaternion_xyzw)
    bones[0].rotation_quaternion = (xyzw[-1],) + xyzw[:3]
    bones[0].location = tuple(position)
    for k,bone in enumerate(bones[1:]):
        bone.rotation_euler = (joint_positions[k],0,0)

    # Insert a keyframe
    bpy.ops.object.mode_set(mode='OBJECT')
    bones[0].keyframe_insert(data_path="rotation_quaternion" ,frame=i)
    bones[0].keyframe_insert(data_path="location" ,frame=i)
    for bone in bones[1:]:
        bone.keyframe_insert(data_path="rotation_euler" ,frame=i)

def add_keyframes(dataFolderPath, armatureName):
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
            position = data[1:4]
            quaternion_xyzw = data[4:8]
            joint_positions = data[8:]
            add_a_keyframe(armature, position, quaternion_xyzw, joint_positions, i)
            i += 1


'''Main loop'''
objName = 'starfish_lod3'
ratio = 0.5
get_mesh_data(objName)
cut_num_polygon(ratio, objName)

armatureName = 'ArmatureOriginal'
# reset_fish(armatureName)

# data file foe 3-link fish
# dataFolderPath = '/home/yif/Documents/KTH/git/3D-Energy-Bounded-Caging/results/FishFallsInBowl_30-04-2023-13-38-45_4blender/'
# data file foe 10-link fish
dataFolderPath = '/home/yif/Documents/KTH/git/3D-Energy-Bounded-Caging/results/FishFallsInBowl_01-05-2023-10-35-08_4blender'

# add_keyframes(dataFolderPath, armatureName)

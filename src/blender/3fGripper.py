
import bpy
import math
import csv
import mathutils
'''
Insert keyframes for a movement primitive of a Robotiq gripper. The data came from Pybullet-OMPL algorithms.
Run the script in Blender background mode:
/snap/bin$ ./blender -b ~/Documents/blender-models/rope.blend -P ~/Documents/KTH/git/3D-Energy-Bounded-Caging/src/blenderScript.py
'''

LINK_NAMES = ['finger_1_link_0', 'finger_1_link_1', 'finger_1_link_2', 'finger_1_link_3', 
              'finger_2_link_0', 'finger_2_link_1', 'finger_2_link_2', 'finger_2_link_3', 
              'finger_middle_link_0', 'finger_middle_link_1', 'finger_middle_link_2', 'finger_middle_link_3', ]

def reset_gripper():
    # Get the current scene
    scene = bpy.context.scene

    # Loop through all the objects in the scene
    for obj in scene.objects:
        # Clear animation data
        obj.animation_data_clear()

    # Reset the starfish
    links = [bpy.data.objects[n] for n in LINK_NAMES]
    for k, link in enumerate(links):                
        # Enter pose mode for the armature
        bpy.context.view_layer.objects.active = link
        bpy.ops.object.mode_set(mode='POSE')

        # Get bones and set rotational modes
        bone = link.pose.bones['Bone']
        bone.rotation_mode = 'XYZ'

        # Reset bone poses
        bone.rotation_euler = (0,0,0)

    # Reset the starfish
    starfish = bpy.data.objects['starfish2']
    bpy.context.view_layer.objects.active = starfish
    starfish.location = (0,0,0)
    starfish.rotation_mode = 'QUATERNION'
    starfish.rotation_quaternion = (1,0,0,0)

def add_a_starfish_keyframe(starfish, position, quaternion_xyzw, numFrame):
    # Enter pose mode for the armature
    bpy.context.view_layer.objects.active = starfish
    
    # Set bone rotation
    starfish.location = tuple(position)
    starfish.rotation_mode = 'QUATERNION'
    xyzw = tuple(quaternion_xyzw)
    starfish.rotation_quaternion = (xyzw[-1],) + xyzw[:3]

    # Insert a keyframe
    bpy.ops.object.mode_set(mode='OBJECT')
    starfish.keyframe_insert(data_path="location", frame=numFrame)
    starfish.keyframe_insert(data_path="rotation_quaternion", frame=numFrame)


def add_a_gripper_keyframe(links, joint_positions, numFrame):
    for k, link in enumerate(links):
        # Enter pose mode for the armature
        bpy.context.view_layer.objects.active = link
        bpy.ops.object.mode_set(mode='POSE')

        # Get bones and set rotational modes
        bone = link.pose.bones['Bone']
        
        # Set bone rotation
        bone.rotation_euler = (0, joint_positions[k], 0,)

        # Insert a keyframe
        bpy.ops.object.mode_set(mode='OBJECT')
        bone.keyframe_insert(data_path="rotation_euler", frame=numFrame)

def add_keyframes(dataFolderPath):
    # Get a reference to the link object
    links = [bpy.data.objects[n] for n in LINK_NAMES]
    starfish = bpy.data.objects['starfish2']
    
    with open('{}/data.csv'.format(dataFolderPath), 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        i = 1
        for row in csvreader:
            if i == 1: # skip the headers
                i += 1
                continue
            data = [float(d) for d in row[:27]]
            position = data[1:4]
            quaternion_xyzw = data[4:8]
            joint_positions = data[15:27]

            # Add keyframes for starfish and gripper
            add_a_gripper_keyframe(links, joint_positions, i)
            add_a_starfish_keyframe(starfish, position, quaternion_xyzw, i)
            i += 1


'''Main'''
reset_gripper()
dataFolderPath = '/home/yif/Documents/KTH/git/3D-Energy-Bounded-Caging/results/GripperClenchesStarfish_23-03-2023-19-03-42'
add_keyframes(dataFolderPath)


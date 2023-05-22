
import bpy
import math
import csv
import mathutils
'''Run the script in Blender background mode:
/snap/bin$ ./blender -b ~/Documents/blender-models/rope.blend -P ~/Documents/KTH/git/3D-Energy-Bounded-Caging/src/blenderScript.py
'''
LINK_NAMES_LEFT = ['finger_1_link_0', 'finger_1_link_1', 'finger_1_link_2', 'finger_1_link_3', 
              'finger_2_link_0', 'finger_2_link_1', 'finger_2_link_2', 'finger_2_link_3', 
              'finger_middle_link_0', 'finger_middle_link_1', 'finger_middle_link_2', 'finger_middle_link_3', 
              'palm'] # 3*4 + 1
LINK_NAMES_RIGHT = [l + '.001' for l in LINK_NAMES_LEFT]
NUM_GRIPPER_JOINTS = 12
GRIPPER_SIZE_SCALE = 10

def reset_gripper():
    # Get the current scene
    scene = bpy.context.scene

    # Loop through all the objects in the scene
    for obj in scene.objects:
        # Clear animation data
        obj.animation_data_clear()

    # Reset the grippers
    links_all = [[bpy.data.objects[n] for n in LINK_NAMES_LEFT]]
    links_all.append([bpy.data.objects[n] for n in LINK_NAMES_RIGHT])
    for links in links_all:
        for k, link in enumerate(links):                
            # Enter pose mode for the armature
            bpy.context.view_layer.objects.active = link
            bpy.ops.object.mode_set(mode='POSE')

            # Get bones and set rotational modes
            bone = link.pose.bones['Bone']
            if k < NUM_GRIPPER_JOINTS:
                bone.rotation_mode = 'XYZ'
                bone.rotation_euler = (0,0,0)
            else:
                bone.location = (0,0,0)
                bone.rotation_mode = 'QUATERNION'
                bone.rotation_quaternion = (1,0,0,0)

    # Reset the cube
    cube = bpy.data.objects['cube-o']
    bpy.context.view_layer.objects.active = cube
    cube.location = (0,0,0)
    cube.rotation_mode = 'QUATERNION'
    cube.rotation_quaternion = (1,0,0,0)

def add_a_cube_keyframe(cube, data, numFrame):
    position = data[1:4]
    quaternion_xyzw = data[4:8]
    
    # Set bone rotation
    cube.location = tuple(position)
    cube.rotation_mode = 'QUATERNION'
    xyzw = tuple(quaternion_xyzw)
    cube.rotation_quaternion = (xyzw[-1],) + xyzw[:3]

    # Insert a keyframe
    bpy.ops.object.mode_set(mode='OBJECT')
    cube.keyframe_insert(data_path="location", frame=numFrame)
    cube.keyframe_insert(data_path="rotation_quaternion", frame=numFrame)


def add_a_gripper_keyframe(links, data, numFrame):
    base_pos = data[:3]
    base_pos = [p/GRIPPER_SIZE_SCALE for p in base_pos]
    base_qtn = data[3:7]
    joint_positions = data[7:]
    for k, link in enumerate(links):
        # Enter pose mode for the armature
        bpy.context.view_layer.objects.active = link
        bpy.ops.object.mode_set(mode='POSE')

        # Get bones and set rotational modes
        bone = link.pose.bones['Bone']
        
        if k < NUM_GRIPPER_JOINTS: # finger links
            bone.rotation_euler = (0, joint_positions[k], 0,)

            # Insert a keyframe
            bpy.ops.object.mode_set(mode='OBJECT')
            bone.keyframe_insert(data_path="rotation_euler", frame=numFrame)

        else: # base (palm) link
            bone.location = tuple(base_pos)
            xyzw = tuple(base_qtn)
            bone.rotation_quaternion = (xyzw[-1],) + xyzw[:3]

            bpy.ops.object.mode_set(mode='OBJECT')
            bone.keyframe_insert(data_path="location", frame=numFrame)
            bone.keyframe_insert(data_path="rotation_quaternion", frame=numFrame)

def add_keyframes(dataFolderPath):
    # Get a reference to the link object
    links_left = [bpy.data.objects[n] for n in LINK_NAMES_LEFT]
    links_right = [bpy.data.objects[n] for n in LINK_NAMES_RIGHT]
    cube = bpy.data.objects['cube-o']
    
    with open('{}/data.csv'.format(dataFolderPath), 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        i = 1
        for row in csvreader:
            if i == 1: # skip the headers
                i += 1
                continue
            data = [float(d) for d in row]
            left_hand_data = data[8:8+3+4+NUM_GRIPPER_JOINTS]
            right_hand_data = data[8+3+4+NUM_GRIPPER_JOINTS:]

            # Add keyframes for cube and gripper
            add_a_gripper_keyframe(links_left, left_hand_data, i)
            add_a_gripper_keyframe(links_right, right_hand_data, i)
            add_a_cube_keyframe(cube, data, i)
            i += 1


'''Main loop'''

reset_gripper()

dataFolderPath = '/home/yif/Documents/KTH/git/3D-Energy-Bounded-Caging/results/BimanualRubic_19-05-2023-21-01-44_dynamics'

add_keyframes(dataFolderPath)

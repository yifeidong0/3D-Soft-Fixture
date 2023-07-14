
import bpy
import math
import csv
'''
Insert keyframes for a movement primitive of a grabbing rope loop scenario. The data came from Pybullet-OMPL algorithms.
Run the script in Blender background mode:
/snap/bin$ ./blender -b ~/Documents/blender-models/rope.blend -P ~/Documents/KTH/git/3D-Energy-Bounded-Caging/src/blenderScript.py
'''

LINK_NAMES_LEFT = ['finger_1_link_0', 'finger_1_link_1', 'finger_1_link_2', 'finger_1_link_3', 
              'finger_2_link_0', 'finger_2_link_1', 'finger_2_link_2', 'finger_2_link_3', 
              'finger_middle_link_0', 'finger_middle_link_1', 'finger_middle_link_2', 'finger_middle_link_3', 
              'palm'] # 3*4 + 1
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
    links = [bpy.data.objects[n] for n in LINK_NAMES_LEFT]
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


def add_a_keyframe_chain(points, bag, pointPositions: list, i: int, numCtrlPnt=7) -> None:
    '''
    pointPositions: 
            list[x1,y1,z1,...,xn,yn,zn], positions of chain control points.
    '''
    # Set cube state
    bag.location = tuple(pointPositions[:3])
    bag.keyframe_insert(data_path="location", frame=i)

    for k,point in enumerate(points):
        point.co = tuple(pointPositions[3*k:3*k+3])
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


def add_keyframes_chain(dataFolderPath: str, curveName: str, bagName: str, numCtrlPnt: int=7) -> None:
    # Get a reference to the curve object
    obj = bpy.data.objects[curveName]
    bag = bpy.data.objects[bagName]
    # bpy.context.view_layer.objects.active = obj
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
            add_a_keyframe_chain(points, bag, pointPositions, i, numCtrlPnt)
            i += 1


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


def add_keyframes_gripper(dataFolderPath, numCtrlPnt):
    # Get a reference to the link object
    links = [bpy.data.objects[n] for n in LINK_NAMES_LEFT]
    
    with open('{}/data.csv'.format(dataFolderPath), 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        i = 0
        for row in csvreader:
            if i == 0: # skip the headers
                i += 1
                continue
            data = [float(d) for d in row]
            gripper_data = data[3*numCtrlPnt:3*numCtrlPnt+3+4+NUM_GRIPPER_JOINTS]

            # Add keyframes for cube and gripper
            add_a_gripper_keyframe(links, gripper_data, i)
            i += 1


'''Main'''
curveName = 'BezierCircle'
bagName = 'bbag'
numCtrlPnt = 7
reset_gripper()

# data file for escape path
dataFolderPath = '/home/yif/Documents/KTH/git/3D-Energy-Bounded-Caging/results/HandbagGripper_24-05-2023-19-39-39_escape_path_4blender'

# data file for dynamic scenario
dataFolderPath1 = '/home/yif/Documents/KTH/git/3D-Energy-Bounded-Caging/results/HandbagGripper_25-05-2023-10-15-48_dynamics'

add_keyframes_chain(dataFolderPath1, curveName, bagName, numCtrlPnt)
add_keyframes_gripper(dataFolderPath1, numCtrlPnt)
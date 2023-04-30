
import bpy
import math

def reset_fish():
    # Get the active object
    obj = bpy.context.active_object

    # Get the animation data for the object
    anim_data = obj.animation_data

    # If the animation data exists, remove all the previous keyframes
    if anim_data is not None:
        for fcurve in anim_data.action.fcurves:
            fcurve.keyframe_points.clear()
            
    # Get a reference to the armature object
    armature = bpy.data.objects['Armature']

    # Enter pose mode for the armature
    bpy.context.view_layer.objects.active = armature
    bpy.ops.object.mode_set(mode='POSE')

    # Get a reference to the pose bones in the armature
    bone = armature.pose.bones['Bone']
    bone001 = armature.pose.bones['Bone.001']
    bone002 = armature.pose.bones['Bone.002']

    bone.rotation_mode = 'XYZ'
    bone001.rotation_mode = 'XYZ'
    bone002.rotation_mode = 'XYZ'

    bone.location = (0.0, 0.0, 0.0)
    bone.rotation_euler = (0.0, 0.0, 0.0)
    bone001.rotation_euler = (0,0,0)
    bone002.rotation_euler = (0,0,0)

def get_mesh_data():
    # Get a reference to the active object in the scene
    obj = bpy.data.objects['bowl']

    # Get the total number of vertices in the object
    num_vertices = len(obj.data.vertices)

    # Get the total number of faces in the object
    num_faces = len(obj.data.polygons)

    # Print the results
    print("Total vertices:", num_vertices)
    print("Total faces:", num_faces)

def cut_num_polygon():
    # Get the active object
    obj = bpy.data.objects['bowl']

    # Add a Decimate modifier to the object
    decimate_mod = obj.modifiers.new(name='Decimate', type='DECIMATE')

    # Set the Decimate ratio to 0.5 (50% reduction in polygon count)
    decimate_mod.ratio = 0.5

    # Apply the modifier
    bpy.ops.object.modifier_apply(modifier=decimate_mod.name)

def add_keyframes():
    # Get a reference to the armature object
    armature = bpy.data.objects['Armature']
    
    # Set the transform orientation to global
    bpy.context.scene.transform_orientation_slots[0].type = 'GLOBAL'
    
    # Enter pose mode for the armature
    bpy.context.view_layer.objects.active = armature
    bpy.ops.object.mode_set(mode='POSE')

    # Get a reference to the pose bones in the armature
    #pose_bones = armature.pose.bones
    bone = armature.pose.bones['Bone']
    bone001 = armature.pose.bones['Bone.001']
    bone002 = armature.pose.bones['Bone.002']
    bone.rotation_mode = 'QUATERNION'
    bone001.rotation_mode = 'XYZ'
    bone002.rotation_mode = 'XYZ'
    
    # Rotate the bone
#    bone.rotation_euler = (0,0,0)
    # Pybullet - XYZW, Blender - WXYZ
    xyzw = (-0.09634363969349324, 0.2278741366312202, 0.3773122691048194, 0.892427438242549)
    bone.rotation_quaternion = (xyzw[-1],) + xyzw[:3]
    bone.location = (-0.0, 0.0, 3.8)
    bone001.rotation_euler = (0,0,0)
    bone002.rotation_euler = (0,0,0.0)

#    bone001.rotation_mode = 'XYZ'
#    axis = 'Z'
#    bone001.rotation_euler.rotate_axis(axis, math.radians(0))

    #insert a keyframe
    bpy.ops.object.mode_set(mode='OBJECT')
    bone.keyframe_insert(data_path="rotation_quaternion" ,frame=1)
    bone.keyframe_insert(data_path="location" ,frame=1)
    bone001.keyframe_insert(data_path="rotation_euler" ,frame=1)
    bone002.keyframe_insert(data_path="rotation_euler" ,frame=1)

    #### A new keyframe
    bpy.context.view_layer.objects.active = armature
    bpy.ops.object.mode_set(mode='POSE')

    # Rotate the bone
#    bone.rotation_euler = (0,0,1.57)
    xyzw = (0.0678753016671461, -0.09251900858011555, 0.35990045472575954, 0.9259075759292273)
    bone.rotation_quaternion = (xyzw[-1],) + xyzw[:3]
    bone.location = (0.12476876503735303, 0.26639682612923915, -0.34633731946807855)
    bone001.rotation_euler = (1.2410960138431641,0,0)
    bone002.rotation_euler = (0.15560453399317326,0,0)

#    bone001.rotation_euler.rotate_axis(axis, math.radians(60))
    
    bpy.ops.object.mode_set(mode='OBJECT')
    bone.keyframe_insert(data_path="rotation_quaternion" ,frame=150)
    bone.keyframe_insert(data_path="location" ,frame=150)
    bone001.keyframe_insert(data_path="rotation_euler" ,frame=150)
    bone002.keyframe_insert(data_path="rotation_euler" ,frame=150)


#cut_num_polygon()

reset_fish()

#get_mesh_data()

add_keyframes()
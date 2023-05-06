import bpy

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

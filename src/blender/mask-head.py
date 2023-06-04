import bpy
import csv

def flatten_nested_list(input):
    '''Input in the format of [[1], [2, 3], [4, 5, 6, 7]].
        Two nested layers at most.
    '''
    return [num for sublist in input for num in sublist]

def record_stripe_data(data):
    folderName = '/home/yif/Documents/KTH/git/3D-Energy-Bounded-Caging/results/4blender/mask-head'

    # create csv headers
    headers = ['frameID', 
               'vertex005fixed-x', 'vertex005fixed-y', 'vertex005fixed-z', 
               'vertex140-x', 'vertex140-y', 'vertex140-z', 
               'vertex145-x', 'vertex145-y', 'vertex145-z', 
               'vertex150-x', 'vertex150-y', 'vertex150-z', 
               'vertex160-x', 'vertex160-y', 'vertex160-z', 
               'vertex170-x', 'vertex170-y', 'vertex170-z', 
               'vertex180-x', 'vertex180-y', 'vertex180-z', 
               'vertex190fixed-x', 'vertex190fixed-y', 'vertex190fixed-z',]

    # write headers to csv
    with open('{}/data.csv'.format(folderName), 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(i for i in headers)

    # write data to csv
    for i in range(len(data)):
        with open('{}/data.csv'.format(folderName), 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(data[i])

# Get the active object
obj = bpy.data.objects['band2-keyframe-0.02distance']

# interested frames - [0137, 0154]
# length: (90), 140, 145, 150, 160, 170, 180, (232)
# vertex ID: 5(fixed), 140, 145, 150, 160, 170, 180, 190(fixed)
frameBoundLeft, frameBoundRight = 5, 250
dataId = [5, 140, 145, 150, 160, 170, 180, 190]
data = []
for i in range(frameBoundLeft, frameBoundRight):
    subdata = [[i],]
    for d in dataId:
        frameStr = "frame_0" + str(i).zfill(3)
        v = obj.data.shape_keys.key_blocks[frameStr].data[d]
        co_final = obj.matrix_world @ v.co
        subdata.append(list(co_final))
    data.append(flatten_nested_list(subdata))

record_stripe_data(data)

# # Create an empty object for visualization
# obj_empty = bpy.data.objects.new("Test", None)
# bpy.context.collection.objects.link(obj_empty)
# obj_empty.location = co_final
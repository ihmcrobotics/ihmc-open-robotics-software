import numpy as np
import json


def copy_object(object):
    new_object = {}
    new_object['type'] = object['type']
    new_object['x'] = object['x']
    new_object['y'] = object['y']
    new_object['z'] = object['z']
    new_object['qx'] = object['qx']
    new_object['qy'] = object['qy']
    new_object['qz'] = object['qz']
    new_object['qs'] = object['qs']
    return new_object

def copy_and_shift(objects, offset):
    shifted_terrain_objects = []

    for obj in objects:

        if obj['type'] == 'RDXSmallCinderBlockRoughed' \
            or obj['type'] == 'RDXMediumCinderBlockRoughed' \
                or obj['type'] == 'RDXLargeCinderBlockRoughed' \
                    or obj['type'] == 'RDXPalletObject':
            
            new_obj = copy_object(obj)
            new_obj['x'] = obj['x'] + offset[0]
            new_obj['y'] = obj['y'] + offset[1]
            new_obj['z'] = obj['z'] + offset[2]
            shifted_terrain_objects.append(new_obj)

    return shifted_terrain_objects

def count_objects_by_type(objects):
    dict = {}
    for obj in objects:
        if obj['type'] in dict:
            dict[obj['type']] += 1
        else:
            dict[obj['type']] = 1

    return dict

def create_shifted_copies(filename):
    offset = np.array([0.2, 2.2, 0])

    shifted_terrain_objects_1 = copy_and_shift(objects, offset)
    shifted_terrain_objects_2 = copy_and_shift(shifted_terrain_objects_1, offset)
    shifted_terrain_objects_3 = copy_and_shift(shifted_terrain_objects_2, offset)
    shifted_terrain_objects_4 = copy_and_shift(shifted_terrain_objects_3, offset)

    # add all shifted objects to objects
    shifted_terrain_objects = objects + shifted_terrain_objects_1 + shifted_terrain_objects_2 + shifted_terrain_objects_3 + shifted_terrain_objects_4

    # save the new terrain to a file with name "FootstepPlannerTrainingTerrainGenerated.json"
    with open(filename, 'w') as outfile:
        # replace data['objects'] with shifted_terrain_objects
        data['objects'] = shifted_terrain_objects
        json.dump(data, outfile)

    counts = count_objects_by_type(shifted_terrain_objects)

    for count, type in counts.items():
        print(type, count)


if __name__ == "__main__":
    # Read JSON file
    path = "/home/quantum/Workspace/Code/IHMC/repository-group/ihmc-open-robotics-software/ihmc-high-level-behaviors/src/libgdx/resources/environments/"
    filename = "LookAndStepWide.json"
    f = open(path + filename, 'r')
    
    data = json.load(f)

    # Print the type of data variable
    objects = data['objects']

    # create_shifted_copies(path + "FootstepPlannerTrainingTerrainGenerated_2.json")

    


# Types:  {'RDXPointLightObject', 
# 'RDXLargeCinderBlockRoughed', 
# 'RDXSmallCinderBlockRoughed', 
# 'RDXPalletObject', 
# 'RDXMediumCinderBlockRoughed', 
# 'RDXLabFloorObject'}



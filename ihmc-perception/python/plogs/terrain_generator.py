import numpy as np
import json
from terrain_map_analyzer import *
from plotting.height_map_tools import *
from hdf5_generator import *

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

def create_shifted_copies(objects, filename):
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
        src_data['objects'] = shifted_terrain_objects
        json.dump(src_data, outfile)

    counts = count_objects_by_type(shifted_terrain_objects)

    for count, type in counts.items():
        print(type, count)


def load_rdx_environment():
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


def plot_height_maps(height_maps):
    number = 0
    for height_map in height_maps:

        plot_and_compute_stats(height_map, display=True)        
        compute_pattern_stats(height_map)
        
        print("\n\n")
        number += 1

def filter_height_maps(maps):
    filtered_maps = []
    for height_map in maps:
        if np.mean(height_map) > 0.001:
            filtered_maps.append(height_map)
    return filtered_maps

def load_height_maps_from_source(src_data):
    height_maps = load_height_maps(src_data, 10000)
    return height_maps

if __name__ == "__main__":
    
    log = True

    home = os.path.expanduser("~")
    src_path = home + "/Downloads/HeightMap_Datasets/one_step.hdf5"
    dst_path = home + "/Downloads/HeightMap_Datasets/input_compressed.hdf5"
    src_data = h5py.File(src_path, "r")
    dst_data = h5py.File(dst_path, "w")

    height_maps = generate_stair_height_maps()
    # height_maps = load_height_maps_from_source(src_data)
    height_maps = filter_height_maps(height_maps)

    if log == False:
        plot_height_maps(height_maps)

    else:
        log_height_maps(dst_data, height_maps, "cropped/height/")

        print("Total Final Maps: ", len(height_maps))
        print("Source File: ", src_path)
        print("Destination File: ", dst_path)

        src_data.close()
        dst_data.close()
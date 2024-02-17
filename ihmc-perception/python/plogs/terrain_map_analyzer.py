import numpy as np
import cv2
from matplotlib import pyplot as plt

import os.path
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from plotting.height_map_tools import *

import h5py
from hdf5_reader import *

def get_roughness(mean_height, stddev_height, psd_height, fft_height):
    roughness = np.sum(psd_height) / (psd_height[0] * psd_height[1])
    return roughness

def get_terrain_stats(height_map, terrain_cost, contact_map):

    # Compute mean and standard deviation for the contact map
    mean_contact = np.mean(contact_map)
    stddev_contact = np.std(contact_map)

    mean_height = np.mean(height_map)
    stddev_height = np.std(height_map)
    fft_height = np.fft.fft2(height_map)
    psd_height = np.abs(fft_height) ** 2
    
    return mean_height, stddev_height, psd_height, fft_height, mean_contact, stddev_contact

def compute_roughness(mean_height, stddev_height, psd_height, fft_height, mean_contact, stddev_contact):
    psd_avg = np.sum(psd_height) / (201 * 201)
    roughness = (psd_avg * (stddev_height / mean_contact))
    return roughness, psd_avg

def plot_and_compute_stats(height_map, debug=True, display=True, label="Terrain Map"):

    terrain_cost = compute_terrain_cost_map(height_map)
    contact_map = compute_contact_map(terrain_cost)
    mean_height, stddev_height, psd_height, fft_height, mean_contact, stddev_contact = get_terrain_stats(height_map, terrain_cost, contact_map)

    # compute roughness of terrain as a scalar
    roughness, psd_avg = compute_roughness(mean_height, stddev_height, psd_height, fft_height, mean_contact, stddev_contact)

    if debug:
        # print the mean, standard deviation, and roughness
        print("Mean Height:", mean_height)
        print("Mean Contact:", mean_contact)
        print("Standard Deviation of Height:", stddev_height)
        print("Standard Deviation of Contact:", stddev_contact)
        print("Roughness of Terrain:", roughness)
        print("Average Power Spectral Density:", psd_avg)

    if display:

        # height_map = cv2.convertScaleAbs(height_map, alpha=(255.0/65535.0))
        # height_map = np.minimum(height_map * 10, 255)
        
        # plot_terrain_maps(height_map, contact_map, contact_map)

        height_map_display = height_map.copy()
        height_map_display = cv2.cvtColor(height_map_display, cv2.COLOR_GRAY2RGB)
        height_map_display = cv2.resize(height_map_display, (1000, 1000))

        contact_map = np.stack([contact_map, contact_map, contact_map], axis=2).astype(np.uint8)
        contact_map[:, :, 1] = contact_map[:, :, 0]
        contact_map[:, :, 0] = 0
        contact_map[:, :, 2] = 0
        contact_map = cv2.resize(contact_map, (1000, 1000))


        print("Height Map Shape:", height_map_display.shape, "Contact Map Shape:", contact_map.shape)    

        # compute scale factor
        scale = 1000 / height_map.shape[0]

        stacked_image = np.hstack((height_map_display, contact_map))

        # Create a resizeable window and resize by scale factor
        cv2.namedWindow(label, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(label, 1000, 1000)
        cv2.imshow(label, stacked_image)

        code = cv2.waitKeyEx(0)
        print("Code:", code)

        if code == ord('q'):
            cv2.destroyAllWindows()
            exit()

        return code


        # # enlarge image for plotting
        # height_map = cv2.resize(height_map, (800, 800), interpolation=cv2.INTER_NEAREST)

        # # Visualize the height map using OpenCV plotting function
        # cv2.imshow("Height Map", height_map)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

    # Plot the height map and power spectral density
    # plt.subplot(1, 2, 1)
    # plt.imshow(height_map, cmap='gray')
    # plt.title("Height Map")
    # plt.colorbar()

    # plt.subplot(1, 2, 2)
    # plt.imshow(np.log1p(psd_height), cmap='gray') 
    # plt.title("Power Spectral Density")
    # plt.colorbar()

    # plt.show()

def synthetic_analyzer_main():

    maps = generate_height_map()
    number = 0
    for name, height_map in maps.items():
        print("Number", number, "Terrain:", name)


        plot_and_compute_stats(height_map, display=False)        
        
        compute_pattern_stats(height_map)
        
        print("\n\n")
        number += 1


if __name__ == "__main__":
    synthetic_analyzer_main()

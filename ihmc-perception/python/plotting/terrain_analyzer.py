import numpy as np
import cv2
from matplotlib import pyplot as plt

def get_roughness(mean_height, stddev_height, psd_height, fft_height):
    roughness = np.sum(psd_height) / (200 * 200)
    return roughness

def get_terrain_stats(height_map):
    
    mean_height = np.mean(height_map)
    stddev_height = np.std(height_map)
    fft_height = np.fft.fft2(height_map)
    psd_height = np.abs(fft_height) ** 2
    
    return mean_height, stddev_height, psd_height, fft_height

def plot_sample_height_stats():

    # Create a 200x200 numpy float grid of height values
    height_map = np.zeros((200, 200), dtype=np.float32)

    # Add three small rectangle patches with height 0.1
    height_map[50:70, 50:70] = 0.1
    height_map[100:120, 100:120] = 0.1
    height_map[150:170, 150:170] = 0.1

    # add a square with slanted height
    for i in range(200):
        for j in range(200):
            if i > 20 and i < 50 and j > 140 and j < 170:
                height_map[i, j] = 0.1 + 0.01 * (j - 150)
                # height_map[i, j] = 0.1

    # Compute mean and standard deviation for the full grid
    mean_height = np.mean(height_map)
    stddev_height = np.std(height_map)

    # Compute the FFT of the height map
    fft_height = np.fft.fft2(height_map)

    # Compute the power spectral density
    psd_height = np.abs(fft_height) ** 2

    # compute roughness of terrain as a scalar
    roughness = np.sum(psd_height) / (200 * 200)

    # print the mean, standard deviation, and roughness
    print("Mean Height:", mean_height)
    print("Standard Deviation of Height:", stddev_height)
    print("Roughness of Terrain:", roughness)

    # Visualize the height map using OpenCV plotting function
    cv2.imshow("Height Map", height_map)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Plot the height map and power spectral density
    plt.subplot(1, 2, 1)
    plt.imshow(height_map, cmap='gray')
    plt.title("Height Map")
    plt.colorbar()

    plt.subplot(1, 2, 2)
    plt.imshow(np.log1p(psd_height), cmap='gray') 
    plt.title("Power Spectral Density")
    plt.colorbar()

    plt.show()
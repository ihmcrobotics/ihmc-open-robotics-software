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

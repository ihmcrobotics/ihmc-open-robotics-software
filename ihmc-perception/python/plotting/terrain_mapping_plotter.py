
import numpy as np
import re
import os

# TerrainMapStatistics:
#  [
    # cpu_processing_time:0.125, 
    # depth_upload_time:0.375, 
    # terrain_map_download_time:0.250, 
    # gpu_processing_time:10.875, 
    # total_time:11.375, 
    # extraction_time:0.000, 
# ]

def plot_log(log_data):

    # Split the string into individual lines
    lines = log_data.splitlines()

    # Create empty numpy arrays for each data field
    cpu_processing_time = np.array([])
    depth_upload_time = np.array([])
    terrain_map_download_time = np.array([])
    gpu_processing_time = np.array([])
    total_time = np.array([])

    # Loop through each line of the log file and extract the relevant data
    for line in lines:
        match = re.search(r'cpu_processing_time:(\d+\.\d+)', line)
        if match:
            cpu_processing_time = np.append(cpu_processing_time, float(match.group(1)))
        match = re.search(r'depth_upload_time:(\d+\.\d+)', line)
        if match:
            depth_upload_time = np.append(depth_upload_time, float(match.group(1)))
        match = re.search(r'terrain_map_download_time:(\d+\.\d+)', line)
        if match:
            terrain_map_download_time = np.append(terrain_map_download_time, float(match.group(1)))
        match = re.search(r'gpu_processing_time:(\d+\.\d+)', line)
        if match:
            gpu_processing_time = np.append(gpu_processing_time, float(match.group(1)))
        match = re.search(r'total_time:(\d+\.\d+)', line)
        if match:
            total_time = np.append(total_time, float(match.group(1)))


    # Store the numpy arrays in a dictionary with descriptive keys
    data_dict = {
        'cpu_processing_time': cpu_processing_time,
        'depth_upload_time': depth_upload_time,
        'terrain_map_download_time': terrain_map_download_time,
        'gpu_processing_time': gpu_processing_time,
        'total_time': total_time
    }

    # Print the data dictionary
    # for key in data_dict:
    #     print(key, data_dict[key])

    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size': 8})

    # ... (code to read in data_dict from log file)

    # Create a figure with 3 rows and 4 columns of subplots
    fig, axs = plt.subplots(2, 3, figsize=(12, 8))

    # Adjust the layout to add some space between subplots
    plt.subplots_adjust(hspace=0.5)  # You can adjust the value as needed

    # Plot each data field in a separate subplot
    axs[0, 0].plot(data_dict['cpu_processing_time'])
    axs[0, 0].set_title('Central Processing Unit Time')
    axs[0, 0].set_xlabel('Iteration')
    axs[0, 0].set_ylabel('Total Displacement (m)')

    axs[0, 1].plot(data_dict['depth_upload_time'])
    axs[0, 1].set_title('Depth Upload Time')
    axs[0, 1].set_xlabel('Iteration')
    axs[0, 1].set_ylabel('Number of Steps')

    axs[0, 2].plot(data_dict['terrain_map_download_time'])
    axs[0, 2].set_title('Terrain Map Download Time')
    axs[0, 2].set_xlabel('Iteration')
    axs[0, 2].set_ylabel('Planning time')

    axs[1, 0].plot(data_dict['gpu_processing_time'])
    axs[1, 0].set_title('GPU _processing TIme')
    axs[1, 0].set_xlabel('Iteration')
    axs[1, 0].set_ylabel('Continuous Walking time')

    axs[1, 1].plot(data_dict['total_time'])
    axs[1, 1].set_title('Number of Interventions')
    axs[1, 1].set_xlabel('Iteration')
    axs[1, 1].set_ylabel('Number of Interventions')

    # Add a title to the entire figure
    fig.suptitle('Terrain Map Extraction Plots')

    # Show the plot
    plt.show()

if __name__ == "__main__":
    home = os.path.expanduser('~')
    path = home + '/Documents/Publications/PhD_Dissertation/Data/terrain-map/'
    files = sorted(os.listdir(path))

    if len(files) == 0:
        print("No log files found at: ", path)
        exit()

    f = open(path + files[-2])
    log_data = f.read()
    plot_log(log_data)


    # for i, filename in enumerate(files):


    #     if i == 0:

    #         print("Opened log file: ", filename)

    #         f = open(path + filename)
    #         log_data = f.read()
    #         plot_log(log_data)



# Timeline
# Feb 27 - GPU-Snap A*
# Feb 21 - GPU-Snap A*
# Feb 08 - MCFP Fast
# Jan 21 - MCFP AlmostAGoodRun, PieceWiseGoodRun
# Jan 17 - MCFP Flat and Rough Terrain
# Jan 16 - MCFP Two Levels 
# Jan 12 - Hybrid Planner
# Jan 11 - Hybrid Planner
# Jan 10 - Hybrid Planner (reduced) A* with MCFP reference
# Jan 08 - CW slow rough terrain A* (reduced A*)
# Nov 13 - CW over uneven terrain (and disaster strikes)
# Oct 27 - Continuous Walking Works with A* Over Height Maps
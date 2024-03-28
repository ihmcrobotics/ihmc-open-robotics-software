
import numpy as np
import re
import os

# TerrainMapStatistics:
#  [
    # TotalTime:0.125, 
    # SimulationTime:0.375, 
    # ExpansionTime:0.250, 
    # PruningTime:10.875, 
    # PropagationTime:11.375, 
# ]

def plot_log(filepath):

    # Initialize lists to store extracted times
    total_times = []
    simulation_times = []
    expansion_times = []
    pruning_times = []
    propagation_times = []
    search_times = []

    nodes = [0,0,0,0,0,0,0,0,0]
    samples = [0,0,0,0,0,0,0,0,0]

    # Regular expression pattern to extract times
    pattern = r'TotalTime: (\d+\.\d+), SimulationTime: (\d+\.\d+), ExpansionTime: (\d+\.\d+), PruningTime: (\d+\.\d+E?-?\d*), PropagationTime: (\d+\.\d+E?-?\d*), SearchTime: (\d+\.\d+E?-?\d*)'

    print("File:", filepath)

    # Read the file
    with open(filepath, 'r') as file:
        for line in file:
            
            line = line.replace("{", "")
            line = line.replace("}", "")

            parts = line.split("NodesPerLayer")

            words = parts[0].split(",")

            total_times.append(float(words[0].replace(" ", "").split(":")[1]))
            simulation_times.append(float(words[1].split(":")[1].strip()))
            expansion_times.append(float(words[2].split(":")[1].strip()))
            pruning_times.append(float(words[3].split(":")[1].strip()))
            propagation_times.append(float(words[4].split(":")[1].strip()))
            search_times.append(float(words[5].split(":")[1].strip()))

            nodes_per_layer = parts[1].replace("(","").replace(")","")[1:]
            nodes_per_layer = nodes_per_layer.split(",")

            for step in nodes_per_layer:
                if ":" in step:
                    counts = step.split(":")
                    first = counts[0]
                    second = counts[1]

                    print(first, second)

                    nodes[int(first)] += int(second)
                    samples[int(first)] += 1


            print(nodes_per_layer)
        
        for i in range(7):
            if samples[i] > 0:
                nodes[i] = nodes[i] / samples[i]


        print(nodes, samples)

    # Create a NumPy matrix from the extracted times
    matrix = np.column_stack((total_times, simulation_times, expansion_times, pruning_times, propagation_times, search_times))

    

    # Create NumPy arrays from the extracted times
    TotalTime = np.array(total_times)
    SimulationTime = np.array(simulation_times)
    ExpansionTime = np.array(expansion_times)
    PruningTime = np.array(pruning_times)
    PropagationTime = np.array(propagation_times)
    SearchTime = np.array(search_times)

    # Store the numpy arrays in a dictionary with descriptive keys
    data_dict = {
        'TotalTime': TotalTime,
        'SimulationTime': SimulationTime,
        'ExpansionTime': ExpansionTime,
        'PruningTime': PruningTime,
        'PropagationTime': PropagationTime,
        'SearchTime': SearchTime
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
    axs[0, 0].plot(data_dict['TotalTime'])
    axs[0, 0].set_title('Total Time')
    axs[0, 0].set_xlabel('Iteration')
    axs[0, 0].set_ylabel('Total Time')

    axs[0, 1].plot(data_dict['SimulationTime'])
    axs[0, 1].set_title('Simulation Time')
    axs[0, 1].set_xlabel('Iteration')
    axs[0, 1].set_ylabel('Simulation Time')

    axs[0, 2].plot(data_dict['ExpansionTime'])
    axs[0, 2].set_title('Expansion Time')
    axs[0, 2].set_xlabel('Iteration')
    axs[0, 2].set_ylabel('Expansion Time')

    axs[1, 0].plot(data_dict['PruningTime'])
    axs[1, 0].set_title('Pruning Time')
    axs[1, 0].set_xlabel('Iteration')
    axs[1, 0].set_ylabel('Pruning Time')

    axs[1, 1].plot(data_dict['PropagationTime'])
    axs[1, 1].set_title('Propagation Time')
    axs[1, 1].set_xlabel('Iteration')
    axs[1, 1].set_ylabel('Propagation Time')

    axs[1, 1].plot(data_dict['SearchTime'])
    axs[1, 1].set_title('Search Time')
    axs[1, 1].set_xlabel('Iteration')
    axs[1, 1].set_ylabel('Search Time')

    # Add a title to the entire figure
    fig.suptitle('Monte-Carlo Planner Log Plots')

    # Show the plot
    plt.show()

if __name__ == "__main__":
    home = os.path.expanduser('~')
    path = home + '/Documents/Publications/PhD_Dissertation/Data/mcfp-logs/'
    local_path = "plotting/data/mcfp_log_real.txt"
    files = sorted(os.listdir(path))

    if len(files) == 0:
        print("No log files found at: ", path)
        exit()

    filepath = local_path
    plot_log(filepath)


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
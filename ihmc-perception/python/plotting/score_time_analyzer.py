import numpy as np
import re
from matplotlib import pyplot as plt


def load_and_plot(data, planner, time_horizon, aggregate_data=True):
    if planner == "MCFP":
        pattern = mcfp_pattern
        file_name = "data/mcfp_console_1.txt"
        block_size = 90
    else:
        pattern = asfp_pattern
        file_name = "data/asfp_console_1.txt"
        block_size = 20

    # Read the text file
    f = open(file_name, "r")

    # Iterate over each line in the file
    for line in f:

        # Match the pattern in the line
        match = re.match(pattern, line)
        if match:

            print(match.group(1), match.group(2), match.group(3))

            # Extract the relevant data from the matched groups
            time = int(match.group(1))
            iteration = int(match.group(2))
            score = float(match.group(3))

            if iteration == 0:

                print("Appending block: ", len(data), len(blocks))
                blocks.append(np.array(data))
                data = []
            
            # Append the extracted data as a tuple to the list
            data.append((time, iteration, score))

    # Append the last block of data to the list
    # blocks.append(data)

    # Close the file
    # f.close()



    print("Number of blocks: ", len(blocks))

    
    total_valid_blocks = 0

    if "ASFP" in planner:
        time_horizon = 1.0
    else:
        time_horizon = 1.0

    print("Planner", planner, "\tTime horizon: ", time_horizon)

    block_array = []
    aggregate_block = np.zeros((100,))
    for index, block in enumerate(blocks):

        if len(block) < 1:
            continue

        data_block = np.array(block)

        if data_block.shape[0] < block_size:
            continue

        total_valid_blocks += 1

        print("Block shape: ", data_block.shape)

        data_block[:, 0] -= data_block[0, 0]
        data_block[:, 0] = data_block[:, 0] / 1e9
        block_array.append(data_block)

        
        # normalize the score
        # data_block[:, 2] /= np.max(data_block[:, 2])

        # remove all rows in the block that have time more than 0.05
        # data_block = data_block[data_block[:, 0] < time_horizon]

        # resample data_block from 0 to time_horizon with 100 points and preserve the iterations
        resampled_score = np.interp(np.linspace(0, time_horizon, 100), data_block[:, 0], data_block[:, 2])
        resampled_time = np.linspace(0, time_horizon, 100)

        print("Resampled block shape: ", data_block.shape)

        if not(aggregate_data):
            plt.plot(resampled_time, resampled_score, label="Plan " + str(index))

        aggregate_block += resampled_score

    aggregate_block /= total_valid_blocks


    if aggregate_data:
        plt.plot(resampled_time, aggregate_block, label=planner)

    # block_array = np.vstack(block_array)

    # print(block_array.shape)

    # total_blocks = len(blocks)

    # print("Total blocks: ", total_blocks)

    # plot_size = 40
    # score_aggregate = np.zeros((plot_size,))

    # for i in range(1):
    #     start = (i)*block_size
    #     end = (i)*block_size + plot_size
    #     plt.plot(block_array[start:end, 1], block_array[start:end, 2], label="Plan " + str(i))

    #     if block_array[start:end, 1].shape != score_aggregate.shape:
    #         print("Shapes not equal: ", block_array[start:end, 1].shape, score_aggregate.shape)
    #         break

    #     print("Shapes: ", block_array[start:end, 1].shape, block_array[start:end, 2].shape, score_aggregate.shape)
    #     score_aggregate += block_array[start:end, 2]

    # print(block_array)

    # score_aggregate /= total_blocks
    # plt.plot(block_array[:plot_size, 0], score_aggregate, label="Aggregate")




if __name__ == "__main__":
    # increase font size of everything
    plt.rc('font', size=14)
    plt.rc('axes', labelsize=14)
    plt.rc('xtick', labelsize=14)
    plt.rc('ytick', labelsize=14)
    plt.rc('legend', fontsize=14)
    plt.rc('figure', titlesize=14)

    # increase plot thickness
    plt.rc('lines', linewidth=2)

    # Initialize empty lists to store the extracted data
    data = []
    blocks = []
    block_size = 90

    # Regular expression pattern to match the desired lines
    mcfp_pattern = r"\d+ \d+:\d+:\d+:\d+ \[WARN\] \(TerrainPlanningDebugger\.java:\d+\): \[MCFP\] Time: (\d+), Iteration: (\d+), Total Score: (\d+\.\d+)"
    asfp_pattern = r"\d+ \d+:\d+:\d+:\d+ \[WARN\] \(AStarFootstepPlanner\.java:\d+\): \[ASFP\] Time: (\d+), Iteration: (\d+), Total Score: (\d+\.\d+)"

    file_name = "data/asfp_console_1.txt"

    plt.figure(figsize=(10, 10))

    plot_time_horizon = 0.8
    mcfp_time_horizon = 1.25
    asfp_time_horizon = 1.5

    load_and_plot(data, "ASFP", asfp_time_horizon, aggregate_data=True)
    load_and_plot(data, "MCFP", mcfp_time_horizon, aggregate_data=True)


    plt.xlabel("Time (s)")
    plt.ylabel("Plan Quality Score")
    # plt.xlim(0, plot_time_horizon)
    # plt.ylim(0, 1.2)
    plt.grid()
    plt.legend()

    plt.show()
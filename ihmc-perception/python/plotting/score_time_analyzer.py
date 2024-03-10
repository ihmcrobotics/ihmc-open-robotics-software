import numpy as np
import re
from matplotlib import pyplot as plt



# Initialize empty lists to store the extracted data
data = []
blocks = []
block_size = 90
planner = "ASFP"

# Regular expression pattern to match the desired lines
mcfp_pattern = r"\d+ \d+:\d+:\d+:\d+ \[WARN\] \(TerrainPlanningDebugger\.java:\d+\): \[MCFP\] Time: (\d+), Iteration: (\d+), Total Score: (\d+\.\d+)"
asfp_pattern = r"\d+ \d+:\d+:\d+:\d+ \[WARN\] \(AStarFootstepPlanner\.java:\d+\): \[ASFP\] Time: (\d+), Iteration: (\d+), Total Score: (\d+\.\d+)"

file_name = "data/asfp_console_1.txt"

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
        iteration = int(match.group(2)) - 1
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

plt.figure(figsize=(10, 10))

block_array = []
for block in blocks:

    if len(block) < 1:
        continue

    data_block = np.array(block)

    if data_block.shape[0] < block_size:
        continue

    block_array.append(data_block)

block_array = np.vstack(block_array)

print(block_array.shape)

total_blocks = len(blocks)

print("Total blocks: ", total_blocks)

plot_size = int(block_size)
score_aggregate = np.zeros((plot_size,))

for i in range(total_blocks):
    start = (i)*block_size
    end = (i)*block_size + plot_size
    plt.plot(block_array[start:end, 1], block_array[start:end, 2], label="Plan " + str(i))

    if block_array[start:end, 1].shape != score_aggregate.shape:
        print("Shapes not equal: ", block_array[start:end, 1].shape, score_aggregate.shape)
        break

    print("Shapes: ", block_array[start:end, 1].shape, block_array[start:end, 2].shape, score_aggregate.shape)
    score_aggregate += block_array[start:end, 2]

score_aggregate /= total_blocks
plt.plot(block_array[:plot_size, 1], score_aggregate, label="Aggregate")

plt.xlabel("Iteration")
plt.ylabel("Total Score")
plt.legend()

plt.show()


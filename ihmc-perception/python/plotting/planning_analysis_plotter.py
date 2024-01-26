import numpy as np
import re
import numpy as np
import matplotlib.pyplot as plt

class MonteCarloPlanningStatistics:
    def __init__(self):
        self.total_time = []
        self.simulation_time = []
        self.expansion_time = []
        self.pruning_time = []
        self.propagation_time = []
        self.search_time = []


    def __str__(self):
        return "TotalTime: {}, SimulationTime: {}, ExpansionTime: {}, PruningTime: {}, PropagationTime: {}, SearchTime: {}".format(
            self.total_time, self.simulation_time, self.expansion_time, self.pruning_time, self.propagation_time, self.search_time
        )
    
class AStarPlanningStatistics:
    def __init__(self):
        self.total_time = []
        self.expansion_time = []
        self.search_time = []


def split_logs():
    # Read the file
    file_path = "data/planning-analysis-console.txt"
    with open(file_path, "r") as file:
        lines = file.readlines()

    # Initialize variables
    sub_logs = []
    current_sub_log = []

    # Iterate through the lines
    for line in lines:

        if "Trial: 0" in line:
            # Add the current sub-log to the list if it is not empty

            if current_sub_log:
                sub_logs.append(current_sub_log)

            # Create a new sub-log
            current_sub_log = []

        # Add the line to the current sub-log
        current_sub_log.append(line)

    # Add the last sub-log to the list if it is not empty
    if current_sub_log:
        sub_logs.append(current_sub_log)

    return sub_logs


# Extract sub logs
sub_logs = split_logs()

# Remove the first log
if sub_logs:
    sub_logs.pop(0)

print("Number of sub-logs:", len(sub_logs))

# Use only the first sub-log instead of the full file
if sub_logs:
    first_sub_log = sub_logs[1]
else:
    first_sub_log = []


for log in sub_logs:
    print("Number of lines in log:", len(log))

# Import the necessary libraries
import matplotlib.pyplot as plt

# Total Time: 83.704 ms, Plan Size: 4, Visited: 486, Layer Counts: {(0:1), (1:125), (2:583), (3:145566), (4:400648), (5:1336410), 

# Initialize lists to store level and node counts
node_counts = [0 for i in range(16)]
total_count = 0

# Iterate through each sub-log
for sub_log in sub_logs:
    # Extract the useful statistics from the sub-log
    for line in sub_log:
        if "Plan Size:" in line:

            total_count += 1

            line = line.split("Layer Counts: ")
            line = line[1].split(", ")
            
            # use regex to extract the level and node count
            for i in range(len(line)):
                match = re.search(r"\((\d+):(\d+)\)", line[i])
                if match:
                    level = int(match.group(1))
                    node_count = int(match.group(2))
                    
                    node_counts[level] += node_count
                    print("Level: {}, Node Count: {}".format(level, node_count))

            # Print the line

# average the node counts
node_counts = [node_count / total_count for node_count in node_counts]
            


# Calculate the standard deviation
std_dev = np.std(node_counts)

# Plot the mean node counts
plt.plot(node_counts)
plt.xlabel('Level')
plt.ylabel('Node Counts')
plt.title('Node Counts as a Function of Level')

# Plot the standard deviation as a band around the mean plot
plt.fill_between(range(len(node_counts)), np.array(node_counts) - std_dev, np.array(node_counts) + std_dev, alpha=0.3)

plt.show()




# Initialize lists to store statistics
total_time_list = []
simulation_time_list = []
expansion_time_list = []
pruning_time_list = []
propagation_time_list = []
search_time_list = []

# Iterate through each sub-log
for sub_log in sub_logs:
    # Create a new statistics object for each sub-log
    mcp_statistics = MonteCarloPlanningStatistics()

    # Extract the useful statistics from the sub-log
    for line in sub_log:
        match = re.search(r"TotalTime: ([\d.]+), SimulationTime: ([\d.]+), ExpansionTime: ([\d.]+), PruningTime: ([\d.E-]+), PropagationTime: ([\d.E-]+), SearchTime: ([\d.E-]+)", line)
        if match:
            mcp_statistics.total_time.append(float(match.group(1)))
            mcp_statistics.simulation_time.append(float(match.group(2)))
            mcp_statistics.expansion_time.append(float(match.group(3)))
            mcp_statistics.pruning_time.append(float(match.group(4)))
            mcp_statistics.propagation_time.append(float(match.group(5)))
            mcp_statistics.search_time.append(float(match.group(6)))

    # Append the statistics from the current sub-log to the lists
    total_time_list.append(mcp_statistics.total_time)
    simulation_time_list.append(mcp_statistics.simulation_time)
    expansion_time_list.append(mcp_statistics.expansion_time)
    pruning_time_list.append(mcp_statistics.pruning_time)
    propagation_time_list.append(mcp_statistics.propagation_time)
    search_time_list.append(mcp_statistics.search_time)

# Convert the lists to numpy arrays
total_time = np.concatenate(total_time_list)
simulation_time = np.concatenate(simulation_time_list)
expansion_time = np.concatenate(expansion_time_list)
pruning_time = np.concatenate(pruning_time_list)
propagation_time = np.concatenate(propagation_time_list)
search_time = np.concatenate(search_time_list)

# Calculate the average of the statistics
average_total_time = np.mean(total_time)
average_simulation_time = np.mean(simulation_time)
average_expansion_time = np.mean(expansion_time)
average_pruning_time = np.mean(pruning_time)
average_propagation_time = np.mean(propagation_time)
average_search_time = np.mean(search_time)


# Plotting the arrays on separate plots
plt.figure(figsize=(18, 12))

plt.subplot(3, 2, 1)
plt.plot(range(len(total_time)), total_time)
plt.fill_between(range(len(total_time)), total_time - np.std(total_time), total_time + np.std(total_time), alpha=0.3, color='red')
plt.title("Total Time")
plt.xticks([])  # Remove x-axis labels

plt.subplot(3, 2, 2)
plt.plot(range(len(simulation_time)), simulation_time)
plt.fill_between(range(len(simulation_time)), simulation_time - np.std(simulation_time), simulation_time + np.std(simulation_time), alpha=0.3, color='red')
plt.title("Simulation Time")
plt.xticks([])  # Remove x-axis labels

plt.subplot(3, 2, 3)
plt.plot(range(len(expansion_time)), expansion_time)
plt.fill_between(range(len(expansion_time)), expansion_time - np.std(expansion_time), expansion_time + np.std(expansion_time), alpha=0.3, color='red')
plt.title("Expansion Time")
plt.xticks([])  # Remove x-axis labels

plt.subplot(3, 2, 4)
plt.plot(range(len(pruning_time)), pruning_time)
plt.fill_between(range(len(pruning_time)), pruning_time - np.std(pruning_time), pruning_time + np.std(pruning_time), alpha=0.3, color='red')
plt.title("Pruning Time")
plt.xticks([])  # Remove x-axis labels

plt.subplot(3, 2, 5)
plt.plot(range(len(propagation_time)), propagation_time)
plt.fill_between(range(len(propagation_time)), propagation_time - np.std(propagation_time), propagation_time + np.std(propagation_time), alpha=0.3, color='red')
plt.title("Propagation Time")
plt.xticks([])  # Remove x-axis labels

plt.subplot(3, 2, 6)
plt.plot(range(len(search_time)), search_time)
plt.fill_between(range(len(search_time)), search_time - np.std(search_time), search_time + np.std(search_time), alpha=0.3, color='red')
plt.title("Search Time")
plt.xticks([])  # Remove x-axis labels

# Display the plots
plt.tight_layout()
plt.show()


import numpy as np
import re
import matplotlib.pyplot as plt

# Read the file
file_path = "data/planning-analysis-console.txt"
with open(file_path, "r") as file:
    lines = file.readlines()


# Extract the useful statistics
total_time = []
simulation_time = []
expansion_time = []
pruning_time = []
propagation_time = []
search_time = []

# ...

for line in lines:
    match = re.search(r"TotalTime: ([\d.]+), SimulationTime: ([\d.]+), ExpansionTime: ([\d.]+), PruningTime: ([\d.E-]+), PropagationTime: ([\d.E-]+), SearchTime: ([\d.E-]+)", line)
    if match:
        total_time.append(float(match.group(1)))
        simulation_time.append(float(match.group(2)))
        expansion_time.append(float(match.group(3)))
        pruning_time.append(float(match.group(4)))
        propagation_time.append(float(match.group(5)))
        search_time.append(float(match.group(6)))

# ...

total_time = np.array(total_time)
simulation_time = np.array(simulation_time)
expansion_time = np.array(expansion_time)
pruning_time = np.array(pruning_time)
propagation_time = np.array(propagation_time)
search_time = np.array(search_time)


# Plotting the arrays on separate plots
plt.figure(figsize=(18, 12))

plt.subplot(3, 2, 1)
plt.plot(total_time)
plt.fill_between(range(len(total_time)), total_time - np.std(total_time), total_time + np.std(total_time), alpha=0.3, color='red')
plt.title("Total Time")

plt.subplot(3, 2, 2)
plt.plot(simulation_time)
plt.fill_between(range(len(simulation_time)), simulation_time - np.std(simulation_time), simulation_time + np.std(simulation_time), alpha=0.3, color='red')
plt.title("Simulation Time")

plt.subplot(3, 2, 3)
plt.plot(expansion_time)
plt.fill_between(range(len(expansion_time)), expansion_time - np.std(expansion_time), expansion_time + np.std(expansion_time), alpha=0.3, color='red')
plt.title("Expansion Time")

plt.subplot(3, 2, 4)
plt.plot(pruning_time)
plt.fill_between(range(len(pruning_time)), pruning_time - np.std(pruning_time), pruning_time + np.std(pruning_time), alpha=0.3, color='red')
plt.title("Pruning Time")

plt.subplot(3, 2, 5)
plt.plot(propagation_time)
plt.fill_between(range(len(propagation_time)), propagation_time - np.std(propagation_time), propagation_time + np.std(propagation_time), alpha=0.3, color='red')
plt.title("Propagation Time")

plt.subplot(3, 2, 6)
plt.plot(search_time)
plt.fill_between(range(len(search_time)), search_time - np.std(search_time), search_time + np.std(search_time), alpha=0.3, color='red')
plt.title("Search Time")

# Display the plots
plt.tight_layout()
plt.show()


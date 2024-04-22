import pandas as pd
import matplotlib.pyplot as plt

import os

# Load data from CSV file
home_path = os.path.expanduser("~")
data = pd.read_csv(home_path + '/Downloads/Logs/20240208_RoughTerrain_MCFP_Run_01.csv')


# Print the name of the columns in separate lines, split the name by "." and get the last element
# Also print serial number of the column as 1. name
for column in data.columns:
    # print(f"{data.columns.get_loc(column)+1}. {column.split('.')[-1]}")

    # print full name
    print(column)

# Create subplots
fig, axes = plt.subplots(nrows=1, ncols=3, figsize=(12, 4))

# Plot the first three signals
for i, signal in enumerate(data.columns[:3]):
    axes[i].plot(data[signal])
    axes[i].set_title(signal)

# Adjust layout and display the plot
plt.tight_layout()
plt.show()
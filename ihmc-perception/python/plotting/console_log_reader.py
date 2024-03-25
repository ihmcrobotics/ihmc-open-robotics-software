
import numpy as np
import re
import os

home = os.path.expanduser('~')
path = home + '/Documents/Publications/PhD_Dissertation/Data/continuous-planning/'
files = sorted(os.listdir(path))

if len(files) == 0:
    print("No log files found at: ", path)
    exit()

# Open the log file and read the contents into a string variable
with open(path + files[-2], 'r') as f:
    log_data = f.read()

print("Opened log file: ", files[-1])

# Split the string into individual lines
lines = log_data.splitlines()

# Create empty numpy arrays for each data field
total_length_completed = np.array([])
total_steps_completed = np.array([])
total_planning_time = np.array([])
last_step_time = np.array([])
total_continuous_walking_time = np.array([])
number_of_interventions = np.array([])
last_planning_time = np.array([])
last_waiting_time = np.array([])
total_waiting_time = np.array([])
total_steps_planned = np.array([])
last_footstep_queue_size = np.array([])
last_step_start_time = np.array([])
last_continuous_walking_time = np.array([])
continuous_walking_speed = np.array([])

# Loop through each line of the log file and extract the relevant data
for line in lines:
    match = re.search(r'total_length_completed:(\d+\.\d+)', line)
    if match:
        total_length_completed = np.append(total_length_completed, float(match.group(1)))
    match = re.search(r'total_steps_completed:(\d+\.\d+)', line)
    if match:
        total_steps_completed = np.append(total_steps_completed, float(match.group(1)))
    match = re.search(r'total_planning_time:(\d+\.\d+)', line)
    if match:
        total_planning_time = np.append(total_planning_time, float(match.group(1)))
    match = re.search(r'last_step_time:(\d+\.\d+)', line)
    if match:
        last_step_time = np.append(last_step_time, float(match.group(1)))
    match = re.search(r'total_continuous_walking_time:(\d+\.\d+)', line)
    if match:
        total_continuous_walking_time = np.append(total_continuous_walking_time, float(match.group(1)))
    match = re.search(r'number_of_interventions:(\d+\.\d+)', line)
    if match:
        number_of_interventions = np.append(number_of_interventions, float(match.group(1)))
    match = re.search(r'last_planning_time:(\d+\.\d+)', line)
    if match:
        last_planning_time = np.append(last_planning_time, float(match.group(1)))
    match = re.search(r'last_waiting_time:(\d+\.\d+)', line)
    if match:
        last_waiting_time = np.append(last_waiting_time, float(match.group(1)))
    match = re.search(r'total_waiting_time:(\d+\.\d+)', line)
    if match:
        total_waiting_time = np.append(total_waiting_time, float(match.group(1)))
    match = re.search(r'total_steps_planned:(\d+\.\d+)', line)
    if match:
        total_steps_planned = np.append(total_steps_planned, float(match.group(1)))
    match = re.search(r'last_footstep_queue_size:(\d+\.\d+)', line)
    if match:
        last_footstep_queue_size = np.append(last_footstep_queue_size, float(match.group(1)))
    match = re.search(r'last_step_start_time:(\d+\.\d+)', line)
    if match:
        last_step_start_time = np.append(last_step_start_time, float(match.group(1)))
    match = re.search(r'last_continuous_walking_time:(\d+\.\d+)', line)
    if match:
        last_continuous_walking_time = np.append(last_continuous_walking_time, float(match.group(1)))

# Store the numpy arrays in a dictionary with descriptive keys
data_dict = {
    'total_length_completed': total_length_completed,
    'total_steps_completed': total_steps_completed,
    'total_planning_time': total_planning_time,
    'last_step_time': last_step_time,
    'total_continuous_walking_time': total_continuous_walking_time,
    'number_of_interventions': number_of_interventions,
    'last_planning_time': last_planning_time,
    'last_waiting_time': last_waiting_time,
    'total_waiting_time': total_waiting_time,
    'total_steps_planned': total_steps_planned,
    'last_footstep_queue_size': last_footstep_queue_size,
    'last_step_start_time': last_step_start_time,
    'last_continuous_walking_time': last_continuous_walking_time
}

# set speed to be total length / total time using numpy
continuous_walking_speed = np.divide(total_length_completed, total_continuous_walking_time)

# Print the data dictionary
# for key in data_dict:
#     print(key, data_dict[key])

import matplotlib.pyplot as plt
plt.rcParams.update({'font.size': 8})

# ... (code to read in data_dict from log file)

# Create a figure with 3 rows and 4 columns of subplots
fig, axs = plt.subplots(3, 4, figsize=(18, 12))

# Adjust the layout to add some space between subplots
plt.subplots_adjust(hspace=0.5)  # You can adjust the value as needed

# Plot each data field in a separate subplot
axs[0, 0].plot(data_dict['total_length_completed'])
axs[0, 0].set_title('Continuous Walking Distance')
axs[0, 3].set_xlabel('Iteration')
axs[0, 0].set_ylabel('Total Displacement (m)')

axs[0, 1].plot(data_dict['total_steps_completed'])
axs[0, 1].set_title('Total Steps Completed')
axs[0, 3].set_xlabel('Iteration')
axs[0, 1].set_ylabel('Number of Steps')

axs[0, 2].plot(data_dict['total_planning_time'])
axs[0, 2].set_title('Total Planning Time')
axs[0, 3].set_xlabel('Iteration')
axs[0, 2].set_ylabel('Planning time')

axs[0, 3].plot(data_dict['total_waiting_time'])
axs[0, 3].set_title('Total Waiting Time')
axs[0, 3].set_xlabel('Iteration')
axs[0, 3].set_ylabel('Total waiting time')

axs[0, 3].plot(data_dict['last_step_time'])
axs[0, 3].set_title('Last Step Time')
axs[0, 3].set_xlabel('Iteration')
axs[0, 3].set_ylabel('Time (s)')

axs[1, 0].plot(data_dict['total_continuous_walking_time'])
axs[1, 0].set_title('Total Continuous Walking Time')
axs[0, 3].set_xlabel('Iteration')
axs[1, 0].set_ylabel('Continuous Walking time')

axs[1, 1].plot(data_dict['number_of_interventions'])
axs[1, 1].set_title('Number of Interventions')
axs[0, 3].set_xlabel('Iteration')
axs[1, 1].set_ylabel('Number of Interventions')

axs[1, 2].plot(data_dict['last_planning_time'])
axs[1, 2].set_title('Last Planning Time')
axs[0, 3].set_xlabel('Iteration')
axs[1, 2].set_ylabel('Planning Time (s)')

axs[1, 3].plot(data_dict['last_waiting_time'])
axs[1, 3].set_title('Last Waiting Time')
axs[1, 3].set_xlabel('Time in seconds')
axs[1, 3].set_ylabel('Last waiting time')

axs[2, 1].plot(data_dict['total_steps_planned'])
axs[2, 1].set_title('Total Steps Planned')
axs[2, 1].set_xlabel('Iteration')
axs[2, 1].set_ylabel('Number of Steps')

axs[2, 2].plot(data_dict['last_footstep_queue_size'])
axs[2, 2].set_title('Last Footstep Queue Size')
axs[2, 2].set_xlabel('Iteration')
axs[2, 2].set_ylabel('Number of Steps')
axs[2, 2].set_ylim(0, 5)

axs[2, 3].plot(continuous_walking_speed)
axs[2, 3].set_title('Continuous Walking Speed')
axs[2, 3].set_xlabel('Speed (m/s)')
axs[2, 3].set_ylabel('Time (seconds)')

# Add a title to the entire figure
fig.suptitle('Continuous Walking Log Data')

# Show the plot
plt.show()



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
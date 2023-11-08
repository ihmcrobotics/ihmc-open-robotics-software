
import numpy as np
import re

# Open the log file and read the contents into a string variable
with open('data/continuous_walking_log.txt', 'r') as f:
    log_data = f.read()

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
last_footstep_queue_length = np.array([])
last_step_start_time = np.array([])
last_continuous_walking_time = np.array([])

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
    match = re.search(r'last_footstep_queue_length:(\d+\.\d+)', line)
    if match:
        last_footstep_queue_length = np.append(last_footstep_queue_length, float(match.group(1)))
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
    'last_footstep_queue_length': last_footstep_queue_length,
    'last_step_start_time': last_step_start_time,
    'last_continuous_walking_time': last_continuous_walking_time
}

# Print the data dictionary
for key in data_dict:
    print(key, data_dict[key])
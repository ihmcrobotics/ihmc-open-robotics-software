import json
import os

from terrain_map_analyzer import *

def load_request_packet(log_file, debug=False):
    
    if debug:
        print("Loading File: {}".format(file_path))

    file = open(log_file, "r")
    data = json.load(file)
    data = data['toolbox_msgs::msg::dds_::FootstepPlanningRequestPacket_']
    return data

def get_height_map(data, debug=False):
    height_map_data = data['height_map_message']
    
    if debug:
        print("ID: {}, Resolution: {}, Grid Size: {}, Center: ({},{}), Ground: {}, Keys: {}, Heights: {}".format(
            height_map_data['sequence_id'],
            height_map_data['xy_resolution'],
            height_map_data['grid_size_xy'],
            height_map_data['grid_center_x'],
            height_map_data['grid_center_y'],
            height_map_data['estimated_ground_height'],
            len(height_map_data['keys']),
            len(height_map_data['heights'])
        ))

    height_map = np.array(height_map_data['heights']).reshape(201, 201)
    return height_map

def plot_terrain_data(height_map, psd):
    # plot height map as heat map with colors non-gray and gray side-by-side and legend
    plt.imshow(height_map, cmap='gray')
    plt.colorbar()
    plt.show()

    plt.imshow(height_map, cmap='jet')
    plt.colorbar()
    plt.show()

    plt.imshow(np.log1p(psd), cmap='gray') 
    plt.colorbar()
    plt.show()

def load_and_analyze(file_path):
    data = load_request_packet(file_path)
    height_map = get_height_map(data)    

    mean, stddev, psd, fft, mean_contact, stddev_contact = get_terrain_stats(height_map)
    roughness = compute_roughness(mean, stddev, psd, fft, mean_contact, stddev_contact)

    print("Mean: {}, Stddev: {}, Mean Contact: {}, Stddev Contact: {}, Roughness: {}".format(mean, stddev, mean_contact, stddev_contact, roughness))
    plot_terrain_data(height_map, psd)

if __name__ == "__main__":

    log_path = "/home/quantum/.ihmc/logs/planning-datasets/"

    log_names = sorted([name for name in os.listdir(log_path) if "FootstepPlannerLog" in name])
    
    for log_name in log_names:
        path = log_path + log_name + "/"
        file_path = path + "RequestPacket.json"
        load_and_analyze(file_path)



# -------------------- NOTES --------------------

# Fields inside log data
# sequence_id
# start_left_foot_pose
# start_right_foot_pose
# goal_left_foot_pose
# goal_right_foot_pose
# requested_initial_stance_side
# snap_goal_steps
# abort_if_goal_step_snapping_fails
# abort_if_body_path_planner_fails
# plan_body_path
# plan_footsteps
# perform_a_star_search
# body_path_waypoints
# goal_distance_proximity
# goal_yaw_proximity
# timeout
# max_iterations
# horizon_length
# planar_regions_list_message
# height_map_message
# assume_flat_ground
# planner_request_id
# status_publish_period
# requested_swing_planner
# reference_plan
# generate_log

# Fields inside height map message:
# sequence_id
# xy_resolution
# grid_size_xy
# grid_center_x
# grid_center_y
# estimated_ground_height
# keys
# heights
# variances
# centroids
# normals




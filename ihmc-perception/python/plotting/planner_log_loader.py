import json
import os

log_path = "/home/quantum/Workspace/Storage/Publications/TRO_2023/Logs/20231207_PlannerLogs/"

log_names = sorted(os.listdir(log_path))

path = log_path + log_names[-1] + "/"
file_path = path + "RequestPacket.json"

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

with open(file_path) as file:
    print("Loading File: {}".format(file_path))
    data = json.load(file)
    data = data['toolbox_msgs::msg::dds_::FootstepPlanningRequestPacket_']
    height_map_data = data['height_map_message']
    
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



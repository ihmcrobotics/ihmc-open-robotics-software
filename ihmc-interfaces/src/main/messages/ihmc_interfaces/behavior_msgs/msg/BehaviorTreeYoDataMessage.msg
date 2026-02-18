# A message used to send behavior tree data from the automomy process
# to the controller process in order to be stored as YoVariables.
# Currently 211 bytes, try to keep this under 250.

uint8 number_of_persistent_detections

uint8 number_of_scene_objects

float32[3] scene_object_x
float32[3] scene_object_y
float32[3] scene_object_z
float32[3] scene_object_yaw
float32[3] scene_object_pitch
float32[3] scene_object_roll

bool automatic_execution

uint16 execution_next_index

bool concurrency_enabled

uint8 number_of_executing_actions

uint8 number_of_failed_actions

int8[5] executing_action_type
int16[5] executing_action_id
float32[5] elapsed_execution_time

float32[2] current_hand_x
float32[2] current_hand_y
float32[2] current_hand_z
float32[2] current_hand_yaw
float32[2] current_hand_pitch
float32[2] current_hand_roll

float32[2] goal_hand_x
float32[2] goal_hand_y
float32[2] goal_hand_z
float32[2] goal_hand_yaw
float32[2] goal_hand_pitch
float32[2] goal_hand_roll

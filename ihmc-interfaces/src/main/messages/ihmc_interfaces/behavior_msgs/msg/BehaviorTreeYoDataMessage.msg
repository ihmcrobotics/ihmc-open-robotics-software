# A message used to send behavior tree data from the automomy process
# to the controller process in order to be stored as YoVariables.
# Try to keep this mesage as small as possible.

uint8 number_of_persistent_detections

uint8 number_of_scene_objects

geometry_msgs/Pose[3] scene_object_pose

bool automatic_execution

uint16 execution_next_index

bool concurrency_enabled

uint8 number_of_executing_actions

uint8 number_of_failed_actions

int8[5] executing_action_type
int16[5] executing_action_id
float32[5] elapsed_execution_time

geometry_msgs/Pose[2] current_hand_pose

geometry_msgs/Pose[2] goal_hand_pose

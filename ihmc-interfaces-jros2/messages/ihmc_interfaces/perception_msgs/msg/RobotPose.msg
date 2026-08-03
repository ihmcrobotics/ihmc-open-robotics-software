# Isaac Sim robot ground-truth root pose + joint angles.
# Published by the Isaac Sim bridge control process (alex_isaac_sim_dds.py) at the
# control rate so RDX can pin the robot mesh to Isaac ground truth (the estimator
# pose in RobotConfigurationData drifts from Isaac truth). Visualization only.

# Sequence number for detecting out-of-order / dropped messages
uint64 sequence_number

# Root (pelvis) world pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation

# Joint angles by name (parallel arrays; applied by name, order-independent)
string[] joint_names
float32[] joint_positions

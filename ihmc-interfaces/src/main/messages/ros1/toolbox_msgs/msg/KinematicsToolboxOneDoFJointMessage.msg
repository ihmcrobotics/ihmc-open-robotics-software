# This message is part of the IHMC whole-body inverse kinematics module.
# This message contains all the information needed to configure the objectives/constraints to apply on
# a given 1-DoF joint in the solver.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# The is the unique hash code of the joint to be solved for.
# It is used on the solver side to retrieve the desired joint to be controlled.
# See Joint.hashCode() for the computation of a joint hash code.
int32 joint_hash_code

# The is the desired joint position.
float64 desired_position

# Weight used to define the priority for reaching the desired position.
float64 weight



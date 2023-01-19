# This message is part of the IHMC multi-contact controller API.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Hash codes of contacting bodies
int32[] contacting_bodies_hash_codes

# List of contact positions for each rigid body, expressed in RigidBody.getParentJoint().getFrameAfterJoint()
geometry_msgs/Point[] contact_point_positions
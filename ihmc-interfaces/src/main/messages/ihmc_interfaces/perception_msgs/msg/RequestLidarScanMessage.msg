# Message used to request a new LIDAR scan, and also enable some filters.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

bool remove_shadows true
bool remove_self_collisions true
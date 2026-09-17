# A message to hold onto the most recent modification timestamp
# on the data this message is associated with.

# The guid of the peer that made the most recent modification
ihmc_common_msgs/GuidMessage latest_modifier_id

# The time the latest modification was made in peer frame
ihmc_common_msgs/InstantMessage latest_modification_time_in_modifier_frame

# Modification number used to order the modifications across all peers
# This allows us to workaround peer clock offset drift.
uint64 latest_modification_number

# Human readable name of the latest modifier
string latest_modifier_name

# A way for a peer to request the full data to be resent
bool full_data_needed

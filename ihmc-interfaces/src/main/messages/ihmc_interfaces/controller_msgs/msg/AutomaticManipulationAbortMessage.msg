# This message is part of the IHMC whole-body controller API.
# Request the controller to enable the automatic manipulation abort feature.
# When enabled, any arm trajectory will get aborted as soon as the balance controller has a large tracking error.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

bool enable
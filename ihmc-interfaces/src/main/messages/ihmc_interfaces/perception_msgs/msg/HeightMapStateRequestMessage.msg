# This message is part of the IHMC height map module.
# It is destined to host user requests for changing the internal state of the height map.

# When true, height map will temporarily stop collecting pointcloud data until another message requesting it to resume is sent.
# In the case height map was already paused, nothing changes.
# In the case both pause and resume are requested, the pause request is ignored.
bool request_pause false
# When true, height map will resume collecting pointcloud data.
# In the case height map was already collecting data, nothing changes.
# In the case both pause and resume are requested, the pause request is ignored.
bool request_resume false
# When true, height map will clear its internal data collected until now.
# After a clear, height map will start the height map from scratch.
# This is useful in the case of noisy data, of if the sensor pose estimation has drifted for instance.
bool request_clear false
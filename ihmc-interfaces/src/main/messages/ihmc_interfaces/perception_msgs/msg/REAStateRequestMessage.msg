# This message is part of the IHMC robot environment awareness module (REA).
# It is destined to host user requests for changing the internal state of REA.

# When true, REA will temporarily stop collecting pointcloud data until another message requesting it to resume is sent.
# In the case REA was already paused, nothing changes.
# In the case both pause and resume are requested, the pause request is ignored.
bool request_pause false
# When true, REA will resume collecting pointcloud data.
# In the case REA was already collecting data, nothing changes.
# In the case both pause and resume are requested, the pause request is ignored.
bool request_resume false
# When true, REA will clear its internal data collected until now.
# After a clear, REA will start building planar regions from scratch.
# This is useful in the case of noisy data, of if the sensor pose estimation has drifted for instance.
bool request_clear false
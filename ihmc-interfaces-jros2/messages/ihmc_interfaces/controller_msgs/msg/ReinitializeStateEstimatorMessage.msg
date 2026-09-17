# This message is part of the IHMC whole-body controller API.
# This message is used to request an initialization fo the state estimator.

bool request_reinitialize
# When true (with request_reinitialize), reinitialize so mid-feet is at world origin.
# Corresponds to the reinitializeStateEstimatorToWorldOrigin YoVariable.
bool reinitialize_to_world_origin

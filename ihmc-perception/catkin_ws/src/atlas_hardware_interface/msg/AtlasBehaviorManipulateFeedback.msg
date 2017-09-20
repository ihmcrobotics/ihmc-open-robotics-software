# Structure for feedback data specific for Manipulate.

uint32 status_flags # Bit-wise array of AtlasBehaviorManipulateFlags values indicating Manipulate behavior status.

AtlasBehaviorPelvisServoParams clamped
AtlasBehaviorPelvisServoParams internal_desired
AtlasBehaviorPelvisServoParams internal_sensed

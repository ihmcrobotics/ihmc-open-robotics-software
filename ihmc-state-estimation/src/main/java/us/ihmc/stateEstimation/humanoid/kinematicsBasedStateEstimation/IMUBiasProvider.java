package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;

public interface IMUBiasProvider
{
   FrameVector3DReadOnly getAngularVelocityBiasInIMUFrame(IMUSensorReadOnly imu);

   FrameVector3DReadOnly getAngularVelocityBiasInWorldFrame(IMUSensorReadOnly imu);

   FrameVector3DReadOnly getLinearAccelerationBiasInIMUFrame(IMUSensorReadOnly imu);

   FrameVector3DReadOnly getLinearAccelerationBiasInWorldFrame(IMUSensorReadOnly imu);
}
package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;

public interface IMUBiasProvider
{

   void getAngularVelocityBiasInIMUFrame(IMUSensorReadOnly imu, Vector3D angularVelocityBiasToPack);

   void getAngularVelocityBiasInIMUFrame(IMUSensorReadOnly imu, FrameVector3D angularVelocityBiasToPack);

   void getAngularVelocityBiasInWorldFrame(IMUSensorReadOnly imu, Vector3D angularVelocityBiasToPack);

   void getAngularVelocityBiasInWorldFrame(IMUSensorReadOnly imu, FrameVector3D angularVelocityBiasToPack);

   void getLinearAccelerationBiasInIMUFrame(IMUSensorReadOnly imu, Vector3D linearAccelerationBiasToPack);

   void getLinearAccelerationBiasInIMUFrame(IMUSensorReadOnly imu, FrameVector3D linearAccelerationBiasToPack);

   void getLinearAccelerationBiasInWorldFrame(IMUSensorReadOnly imu, Vector3D linearAccelerationBiasToPack);

   void getLinearAccelerationBiasInWorldFrame(IMUSensorReadOnly imu, FrameVector3D linearAccelerationBiasToPack);
}
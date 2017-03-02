package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;

public interface IMUBiasProvider
{

   void getAngularVelocityBiasInIMUFrame(IMUSensorReadOnly imu, Vector3D angularVelocityBiasToPack);

   void getAngularVelocityBiasInIMUFrame(IMUSensorReadOnly imu, FrameVector angularVelocityBiasToPack);

   void getAngularVelocityBiasInWorldFrame(IMUSensorReadOnly imu, Vector3D angularVelocityBiasToPack);

   void getAngularVelocityBiasInWorldFrame(IMUSensorReadOnly imu, FrameVector angularVelocityBiasToPack);

   void getLinearAccelerationBiasInIMUFrame(IMUSensorReadOnly imu, Vector3D linearAccelerationBiasToPack);

   void getLinearAccelerationBiasInIMUFrame(IMUSensorReadOnly imu, FrameVector linearAccelerationBiasToPack);

   void getLinearAccelerationBiasInWorldFrame(IMUSensorReadOnly imu, Vector3D linearAccelerationBiasToPack);

   void getLinearAccelerationBiasInWorldFrame(IMUSensorReadOnly imu, FrameVector linearAccelerationBiasToPack);
}
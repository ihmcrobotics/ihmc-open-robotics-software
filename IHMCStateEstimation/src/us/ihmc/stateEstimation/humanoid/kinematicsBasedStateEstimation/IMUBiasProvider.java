package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;

public interface IMUBiasProvider
{

   void getAngularVelocityBiasInIMUFrame(IMUSensorReadOnly imu, Vector3d angularVelocityBiasToPack);

   void getAngularVelocityBiasInIMUFrame(IMUSensorReadOnly imu, FrameVector angularVelocityBiasToPack);

   void getAngularVelocityBiasInWorldFrame(IMUSensorReadOnly imu, Vector3d angularVelocityBiasToPack);

   void getAngularVelocityBiasInWorldFrame(IMUSensorReadOnly imu, FrameVector angularVelocityBiasToPack);

   void getLinearAccelerationBiasInIMUFrame(IMUSensorReadOnly imu, Vector3d linearAccelerationBiasToPack);

   void getLinearAccelerationBiasInIMUFrame(IMUSensorReadOnly imu, FrameVector linearAccelerationBiasToPack);

   void getLinearAccelerationBiasInWorldFrame(IMUSensorReadOnly imu, Vector3d linearAccelerationBiasToPack);

   void getLinearAccelerationBiasInWorldFrame(IMUSensorReadOnly imu, FrameVector linearAccelerationBiasToPack);
}
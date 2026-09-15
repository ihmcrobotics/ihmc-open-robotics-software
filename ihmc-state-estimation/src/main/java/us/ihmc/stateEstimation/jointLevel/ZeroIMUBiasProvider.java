package us.ihmc.stateEstimation.jointLevel;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBiasProvider;

/**
 * Null-object IMUBiasProvider: always zero bias, in the frame that the caller expects.
 * NOT thread-safe: the returned vector aliases one internal field.
 * One instance per estimator pipeline.
 */
public class ZeroIMUBiasProvider implements IMUBiasProvider
{
   private final FrameVector3D zeroVector = new FrameVector3D();

   @Override
   public FrameVector3DReadOnly getAngularVelocityBiasInIMUFrame(IMUSensorReadOnly imu)
   {
      zeroVector.setToZero(imu.getMeasurementFrame());
      return zeroVector;
   }

   @Override
   public FrameVector3DReadOnly getAngularVelocityBiasInWorldFrame(IMUSensorReadOnly imu)
   {
      zeroVector.setToZero(ReferenceFrame.getWorldFrame());
      return zeroVector;
   }

   @Override
   public FrameVector3DReadOnly getLinearAccelerationBiasInIMUFrame(IMUSensorReadOnly imu)
   {
      zeroVector.setToZero(imu.getMeasurementFrame());
      return zeroVector;
   }

   @Override
   public FrameVector3DReadOnly getLinearAccelerationBiasInWorldFrame(IMUSensorReadOnly imu)
   {
      zeroVector.setToZero(ReferenceFrame.getWorldFrame());
      return zeroVector;
   }
}

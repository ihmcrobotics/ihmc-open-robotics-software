package us.ihmc.stateEstimation.jointLevel;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;

import java.util.List;

/**
 * Explicit no-op {@link ProprioceptivePreFilter}: no joint estimates (consumers fall back to raw
 * sensor values for every joint), zero IMU biases, no covariance. Selected via
 * {@code JointLevelEstimatorType.NONE} as a control condition — it reproduces the invariant
 * pipeline's pre-seam behavior exactly. Unlike the other implementations it is stateless, but keep
 * one instance per pipeline anyway for consistency.
 */
public class PassThroughPreFilter implements ProprioceptivePreFilter
{
   private final ZeroIMUBiasProvider zeroBias = new ZeroIMUBiasProvider();

   @Override
   public void initialize()
   {
   }

   @Override
   public void computeJointState()
   {
   }

   @Override
   public void computeImuBiases(List<RigidBodyBasics> trustedFeet)
   {
   }

   @Override
   public boolean containsJoint(OneDoFJointBasics joint)
   {
      return false;
   }

   @Override
   public double getEstimatedJointPosition(OneDoFJointBasics joint)
   {
      return Double.NaN;
   }

   @Override
   public double getEstimatedJointVelocity(OneDoFJointBasics joint)
   {
      return Double.NaN;
   }

   @Override
   public boolean hasCovariance()
   {
      return false;
   }

   @Override
   public void packPositionCovariance(OneDoFJointBasics[] joints, double fallbackVariance, DMatrixRMaj toPack)
   {
      throw new UnsupportedOperationException("Check hasCovariance() before calling.");
   }

   @Override
   public void packVelocityCovariance(OneDoFJointBasics[] joints, double fallbackVariance, DMatrixRMaj toPack)
   {
      throw new UnsupportedOperationException("Check hasCovariance() before calling.");
   }

   @Override
   public FrameVector3DReadOnly getAngularVelocityBiasInIMUFrame(IMUSensorReadOnly imu)
   {
      return zeroBias.getAngularVelocityBiasInIMUFrame(imu);
   }

   @Override
   public FrameVector3DReadOnly getAngularVelocityBiasInWorldFrame(IMUSensorReadOnly imu)
   {
      return zeroBias.getAngularVelocityBiasInWorldFrame(imu);
   }

   @Override
   public FrameVector3DReadOnly getLinearAccelerationBiasInIMUFrame(IMUSensorReadOnly imu)
   {
      return zeroBias.getLinearAccelerationBiasInIMUFrame(imu);
   }

   @Override
   public FrameVector3DReadOnly getLinearAccelerationBiasInWorldFrame(IMUSensorReadOnly imu)
   {
      return zeroBias.getLinearAccelerationBiasInWorldFrame(imu);
   }
}

package us.ihmc.stateEstimation.jointLevel;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBiasProvider;

import java.util.List;

/**
 * Joint-level Kalman filter pre-filter (P-A architecture): one filter over the IMU tree whose pair
 * measurements z_w,ab = J(q^) S_ab qd + b_w,ab + v couple the joints on each IMU-pair path, with
 * per-IMU biases in state and a phase-2 gauge anchor for the absolute base bias.
 *
 * <p><b>Current state: pass-through stub.</b> {@code containsJoint} returns false for every joint
 * (consumers fall back to raw sensor values wholesale), biases are zero, and no covariance is
 * offered. Selecting {@code JOINT_KF} must therefore reproduce raw pass-through behavior exactly —
 * that is the wiring oracle. The filter mathematics land behind this frozen interface.</p>
 *
 * <p><b>Do not share an instance between pipelines</b> (the filter holds its covariance).</p>
 */
public class JointLevelKFPreFilter implements ProprioceptivePreFilter
{
   private final ZeroIMUBiasProvider zeroBias = new ZeroIMUBiasProvider();

   @Override
   public void initialize()
   {
   }

   @Override
   public void computeJointState()
   {
      // Phase 1: joint predict + encoder/pair-gyro measurement updates go here.
   }

   @Override
   public void computeImuBiases(List<RigidBodyBasics> trustedFeet)
   {
      // Phase 2: the absolute-bias gauge anchor (stance-FK / gravity direction) goes here.
   }

   @Override
   public boolean containsJoint(OneDoFJointBasics joint)
   {
      return false; // stub: consumers fall back to the raw sensor map for every joint
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
      return false; // flips to true when the filter publishes Sigma_q / Sigma_qd
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

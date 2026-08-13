package us.ihmc.stateEstimation.jointLevel;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBiasProvider;

import java.util.List;

public interface ProprioceptivePreFilter extends OneDoFJointStateSource, IMUBiasProvider
{
   void initialize();

   // Phase 1: Run at the top of the estimator tick, before joint outputs are consumed
   void computeJointState();

   // Phase 2: Run after the trust decision; trustedFeet may be empty, but *not null*.
   void computeImuBiases(List<RigidBodyBasics> trustedFeet);

   /**
    * Re-seed this pre-filter from the state of the estimator that was already running, for the case where its
    * owning estimator is switched in after standing by cold (see {@code SwitchableMainStateEstimator}). The
    * mean is taken from the handover; the covariance is reset to its startup value, since two estimators do
    * not share a state parameterization.
    *
    * <p>No-op by default: only a filter that carries covariance across ticks can go stale while cold.</p>
    *
    * @param biasSource per-IMU gyro bias from the outgoing estimator; may be null.
    */
   default void seedFromHandover(IMUBiasProvider biasSource)
   {
   }
}

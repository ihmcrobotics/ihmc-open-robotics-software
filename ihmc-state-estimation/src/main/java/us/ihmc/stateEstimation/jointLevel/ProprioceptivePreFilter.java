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
}

package us.ihmc.stateEstimation.jointLevel;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBiasProvider;

import java.util.List;

public interface ProprioceptiveFilter extends OneDoFJointStateSource, IMUBiasProvider
{
   void initialize();
   void computeJointState();
   void computeImuBiases(List<RigidBodyBasics> trustedFeet);
}

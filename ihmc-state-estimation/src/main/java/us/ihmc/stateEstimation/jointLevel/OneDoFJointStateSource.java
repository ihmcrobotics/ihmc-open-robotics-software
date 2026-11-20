package us.ihmc.stateEstimation.jointLevel;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBiasProvider;

import java.util.List;

public interface OneDoFJointStateSource
{
   boolean containsJoint(OneDoFJointBasics joint);
   double getEstimatedJointPosition(OneDoFJointBasics joint); // NaN -> caller falls back to sensor map
   double getEstimatedJointVelocity(OneDoFJointBasics joint);

   boolean hasCovariance();
   /**
    * Packs the m-by-m covariance for the given joints, in the caller's ordering.
    * Joints this source does not own get fallbackVariance on the diagonal and
    * zero cross terms. Only call when hasCovariance() returns true.
    */
   void packPositionCovariance(OneDoFJointBasics[] joints, double fallbackVariance, DMatrixRMaj toPack);
   void packVelocityCovariance(OneDoFJointBasics[] joints, double fallbackVariance, DMatrixRMaj toPack);
}


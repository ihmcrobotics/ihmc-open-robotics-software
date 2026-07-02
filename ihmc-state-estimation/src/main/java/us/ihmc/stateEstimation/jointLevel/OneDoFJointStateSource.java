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
   /** m×m in the caller's joint ordering. Joints this source doesn't own get
    *  fallbackVariance on the diagonal and zero cross terms. Only call when hasCovariance(). */
   void packPositionCovariance(OneDoFJointBasics[] joints, double fallbackVariance, DMatrixRMaj toPack);
   void packVelocityCovariance(OneDoFJointBasics[] joints, double fallbackVariance, DMatrixRMaj toPack);
}


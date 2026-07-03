package us.ihmc.stateEstimation.jointLevel;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBasedJointStateEstimator;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBiasProvider;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBiasStateEstimator;

import java.util.List;

public class AlphaComplementaryFilter implements ProprioceptiveFilter
{
   private final List<IMUBasedJointStateEstimator> jointEstimators;
   private final IMUBiasStateEstimator imuBiasStateEstimator; // may be null (no bias estimation)
   private final IMUBiasProvider biasProviderInternal; // never null: estimator or zero object

   public AlphaComplementaryFilter(List<IMUBasedJointStateEstimator> jointEstimators, IMUBiasStateEstimator imuBiasStateEstimator)
   {
      this.jointEstimators = jointEstimators;
      this.imuBiasStateEstimator = imuBiasStateEstimator;
      this.biasProviderInternal = imuBiasStateEstimator != null ? imuBiasStateEstimator : new ZeroIMUBiasProvider();
   }

   @Override
   public void initialize()
   {
      if (imuBiasStateEstimator != null)
         imuBiasStateEstimator.initialize();
   }

   @Override
   public void computeImuBiases(List<RigidBodyBasics> trustedFeet)
   {
      if (imuBiasStateEstimator != null)
         imuBiasStateEstimator.compute(trustedFeet);
   }

   @Override
   public void computeJointState()
   {
      for (int i = 0; i < jointEstimators.size(); i++)
         jointEstimators.get(i).compute();
   }

   @Override
   public boolean containsJoint(OneDoFJointBasics joint)
   {
      for (int i = 0; i < jointEstimators.size(); i++)
      {
         if (jointEstimators.get(i).containsJoint(joint))
            return true;
      }
      return false;
   }

   @Override
   public double getEstimatedJointPosition(OneDoFJointBasics joint)
   {
      for (int i = 0; i < jointEstimators.size(); i++)
      {
         if (jointEstimators.get(i).containsJoint(joint))
            return jointEstimators.get(i).getEstimatedJointPosition(joint); // may be NaN - which is correct
      }
      return Double.NaN;
   }

   @Override
   public double getEstimatedJointVelocity(OneDoFJointBasics joint)
   {
      for (int i = 0; i < jointEstimators.size(); i++)
      {
         if (jointEstimators.get(i).containsJoint(joint))
            return jointEstimators.get(i).getEstimatedJointVelocity(joint); // may be NaN - which is correct
      }
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
      return biasProviderInternal.getAngularVelocityBiasInIMUFrame(imu);
   }

   @Override
   public FrameVector3DReadOnly getAngularVelocityBiasInWorldFrame(IMUSensorReadOnly imu)
   {
      return biasProviderInternal.getAngularVelocityBiasInWorldFrame();
   }

   @Override
   public FrameVector3DReadOnly getLinearAccelerationBiasInIMUFrame(IMUSensorReadOnly imu)
   {
      return biasProviderInternal.getLinearAccelerationBiasInIMUFrame();
   }

   @Override
   public FrameVector3DReadOnly getLinearAccelerationBiasInWorldFrame(IMUSensorReadOnly imu)
   {
      return biasProviderInternal.getLinearAccelerationBiasInWorldFrame();
   }
}

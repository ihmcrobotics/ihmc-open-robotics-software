package us.ihmc.stateEstimation.jointLevel;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUBasedJointStateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBasedJointStateEstimator;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBiasProvider;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBiasStateEstimator;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * {@link ProprioceptivePreFilter} adapter around the alpha-era machinery: a list of
 * {@link IMUBasedJointStateEstimator}s (complementary encoder/IMU joint fusion) for the joint role
 * and an {@link IMUBiasStateEstimator} (alpha-filtered bias rejection) for the bias role. Contains
 * no new behavior: every method delegates or reproduces what {@code JointStateUpdater} and
 * {@code DRCKinematicsBasedStateEstimator} wired directly before this seam existed.
 *
 * <p><b>Do not share an instance between pipelines.</b> The wrapped estimators hold internal state
 * (backlash windows, integrated IMU position channels); a shared instance ticked by two consumers
 * double-updates. One pre-filter per estimator pipeline.</p>
 */
public class AlphaComplementaryPreFilter implements ProprioceptivePreFilter
{
   private final List<IMUBasedJointStateEstimator> jointEstimators;
   private final IMUBiasStateEstimator imuBiasStateEstimator; // may be null (no bias estimation)
   private final IMUBiasProvider biasProviderInternal; // never null: estimator or zero object

   public AlphaComplementaryPreFilter(List<IMUBasedJointStateEstimator> jointEstimators, IMUBiasStateEstimator imuBiasStateEstimator)
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

   /**
    * Assembles the alpha pre-filter the way the DRC estimator historically did: the IMU-pair lookup
    * moved verbatim from {@code JointStateUpdater.createIMUBasedJointVelocityEstimators}, then the
    * {@link IMUBiasStateEstimator} construction moved from {@code DRCKinematicsBasedStateEstimator}.
    * Runs once at construction time; allocation here is fine (the allocation-free rule governs the
    * per-tick path only).
    */
   public static AlphaComplementaryPreFilter createForKinematicsEstimator(
         SensorOutputMapReadOnly sensorOutputMapReadOnly,
         StateEstimatorParameters stateEstimatorParameters,
         List<? extends IMUSensorReadOnly> imuProcessedOutputs,
         Collection<RigidBodyBasics> feet,
         double gravitationalAcceleration,
         BooleanProvider cancelGravityFromAccelerationMeasurement,
         double estimatorDT,
         YoRegistry parentRegistry)
   {
      if (stateEstimatorParameters == null)
         throw new UnsupportedOperationException("default estimator parameters for this type of estimator not added yet.");

      List<IMUBasedJointStateEstimator> estimators = new ArrayList<>();

      for (IMUBasedJointStateEstimatorParameters parameters : stateEstimatorParameters.getIMUBasedJointStateEstimatorParameters())
      {
         IMUSensorReadOnly parentIMU = null;
         IMUSensorReadOnly childIMU = null;

         String parentIMUName = parameters.getParentIMUName();
         String childIMUName = parameters.getChildIMUName();

         for (int i = 0; i < sensorOutputMapReadOnly.getIMUOutputs().size(); i++)
         {
            IMUSensorReadOnly sensorReadOnly = sensorOutputMapReadOnly.getIMUOutputs().get(i);
            if (sensorReadOnly.getSensorName().equals(parentIMUName))
               parentIMU = sensorReadOnly;

            if (sensorReadOnly.getSensorName().equals(childIMUName))
               childIMU = sensorReadOnly;
         }

         if (parentIMU != null && childIMU != null)
         {
            estimators.add(new IMUBasedJointStateEstimator(stateEstimatorParameters.getEstimatorDT(),
                                                           parentIMU,
                                                           childIMU,
                                                           sensorOutputMapReadOnly,
                                                           parameters,
                                                           parentRegistry));
         }
         else
         {
            LogTools.warn("Could not find the given parent and/or child IMUs: parentIMU = " + parentIMUName + ", childIMU = " + childIMUName);
            if (parentIMU == null)
               LogTools.warn("Parent IMU is null");
            if (childIMU == null)
               LogTools.warn("Child IMU is null");
         }
      }

      IMUBiasStateEstimator biasEstimator = new IMUBiasStateEstimator(imuProcessedOutputs,
                                                                      feet,
                                                                      gravitationalAcceleration,
                                                                      cancelGravityFromAccelerationMeasurement,
                                                                      estimatorDT,
                                                                      stateEstimatorParameters,
                                                                      parentRegistry);
      return new AlphaComplementaryPreFilter(estimators, biasEstimator);

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
      return biasProviderInternal.getAngularVelocityBiasInWorldFrame(imu);
   }

   @Override
   public FrameVector3DReadOnly getLinearAccelerationBiasInIMUFrame(IMUSensorReadOnly imu)
   {
      return biasProviderInternal.getLinearAccelerationBiasInIMUFrame(imu);
   }

   @Override
   public FrameVector3DReadOnly getLinearAccelerationBiasInWorldFrame(IMUSensorReadOnly imu)
   {
      return biasProviderInternal.getLinearAccelerationBiasInWorldFrame(imu);
   }
}

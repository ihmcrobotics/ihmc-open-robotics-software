package us.ihmc.stateEstimation.jointLevel;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.sensorProcessing.imu.IMUSensor;
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
import java.util.Collections;
import java.util.List;
import java.util.Map;

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

   public static AlphaComplementaryPreFilter createForKinematicsEstimator(
         SensorOutputMapReadOnly sensorOutputMapReadOnly,
         StateEstimatorParameters stateEstimatorParameters,
         List<? extends IMUSensorReadOnly> imuProcessedOutputs,
         Map<RigidBodyBasics, ?extends ContactablePlaneBody> feet,
         double gravitationalAcceleration,
         BooleanProvider cancelGravityFromAccelerationMeasurement,
         double estimatorDT,
         YoRegistry parentRegistry)
   {
      // Same thing from `JointStateUpdater`
//      if (stateEstimatorParameters == null)
//         return Collections.emptyList();
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
                                                                      feet.keySet(),
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

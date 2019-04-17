package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoSpatialVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class JointTorqueBasedWrenchSensorDriftEstimator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final List<ForceSensorDefinition> forceSensorDefinitions;
   private final Map<ForceSensorDefinition, WrenchSensorDriftInfo> wrenchSensorDrifInfoMap = new HashMap<>();

   private final RobotMotionStatusHolder robotMotionStatusHolder;

   private final DoubleParameter initializationDuration;
   private final DoubleParameter biasFilterBreakFrequency;
   private final DoubleParameter minJacobianDeterminant;

   public JointTorqueBasedWrenchSensorDriftEstimator(FloatingJointBasics rootJoint, double updateDT, RobotMotionStatusHolder robotMotionStatusHolder,
                                                     Collection<ForceSensorDefinition> forceSensorDefinitions,
                                                     ForceSensorDataHolderReadOnly forceSensorDataHolder, YoVariableRegistry parentRegistry)
   {
      this.robotMotionStatusHolder = robotMotionStatusHolder;
      this.forceSensorDefinitions = new ArrayList<>(forceSensorDefinitions);
      initializationDuration = new DoubleParameter("initializationDurationWrenchSensorDriftEstimator", registry, 10.0);
      biasFilterBreakFrequency = new DoubleParameter("wrenchBiasFilterBreakFrequency", "", registry, 0.5);
      minJacobianDeterminant = new DoubleParameter("wrenchBiasEstimatorMinJacobianDet", registry, 0.05);
      DoubleProvider filterAlphaProvider = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(biasFilterBreakFrequency.getValue(), updateDT);

      for (ForceSensorDefinition forceSensorDefinition : forceSensorDefinitions)
      {
         ForceSensorDataReadOnly forceSensorData = forceSensorDataHolder.get(forceSensorDefinition);
         wrenchSensorDrifInfoMap.put(forceSensorDefinition,
                                     new WrenchSensorDriftInfo(rootJoint,
                                                               updateDT,
                                                               forceSensorDefinition,
                                                               forceSensorData,
                                                               filterAlphaProvider,
                                                               minJacobianDeterminant,
                                                               registry));
      }

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      RobotMotionStatus currentStatus = robotMotionStatusHolder.getCurrentRobotMotionStatus();

      if (currentStatus == RobotMotionStatus.STANDING)
      {
         for (int sensorIndex = 0; sensorIndex < forceSensorDefinitions.size(); sensorIndex++)
         {
            wrenchSensorDrifInfoMap.get(forceSensorDefinitions.get(sensorIndex)).update();
         }
      }
   }

   public void reset()
   {
      for (int sensorIndex = 0; sensorIndex < forceSensorDefinitions.size(); sensorIndex++)
      {
         WrenchSensorDriftInfo wrenchSensorDriftInfo = wrenchSensorDrifInfoMap.get(forceSensorDefinitions.get(sensorIndex));
         wrenchSensorDriftInfo.reset();
         wrenchSensorDriftInfo.setInitializationDuration(initializationDuration.getValue());
      }
   }

   public SpatialVectorReadOnly getSensortDriftBias(ForceSensorDefinition forceSensorDefinition)
   {
      WrenchSensorDriftInfo driftInfo = wrenchSensorDrifInfoMap.get(forceSensorDefinition);

      if (driftInfo != null)
         return driftInfo.getEstimatedDriftBias();
      else
         return null;
   }

   private static class WrenchSensorDriftInfo
   {
      private final ForceSensorDataReadOnly forceSensorData;
      private final EndEffectorWrenchEstimator endEffectorWrenchEstimator;
      private final YoFixedFrameSpatialVector initialOffset;
      private final YoFixedFrameSpatialVector estimatedDriftBias;
      private final AlphaFilteredYoSpatialVector estimatedDriftBiasFiltered;
      private final YoBoolean isInitialized;
      private final Wrench measuredWrench = new Wrench();
      private final SpatialVector spatialVector = new SpatialVector();
      private final DoubleProvider minJacobianDeterminant;

      private double initializationAlpha;
      private double initializationDuration;
      private double currentTimeInInitialization;
      private final double updateDT;

      public WrenchSensorDriftInfo(FloatingJointBasics rootJoint, double updateDT, ForceSensorDefinition forceSensorDefinition,
                                   ForceSensorDataReadOnly forceSensorData, DoubleProvider alphaProvider, DoubleProvider minJacobianDeterminant,
                                   YoVariableRegistry registry)
      {
         this.updateDT = updateDT;
         this.forceSensorData = forceSensorData;
         this.minJacobianDeterminant = minJacobianDeterminant;
         String name = forceSensorDefinition.getSensorName() + "JointTorqueWrench";
         RigidBodyBasics endEffector = forceSensorDefinition.getRigidBody();
         ReferenceFrame wrenchMeasurementFrame = forceSensorDefinition.getSensorFrame();
         endEffectorWrenchEstimator = new EndEffectorWrenchEstimator(name, rootJoint.getSuccessor(), endEffector, wrenchMeasurementFrame, registry);
         initialOffset = new YoFixedFrameSpatialVector(forceSensorDefinition.getSensorName() + "DriftEstimatorInitialOffset", wrenchMeasurementFrame, registry);
         estimatedDriftBias = new YoFixedFrameSpatialVector(forceSensorDefinition.getSensorName() + "WrenchBias", wrenchMeasurementFrame, registry);
         estimatedDriftBiasFiltered = new AlphaFilteredYoSpatialVector(forceSensorDefinition.getSensorName()
               + "WrenchBiasFiltered", "", registry, alphaProvider, alphaProvider, estimatedDriftBias);
         isInitialized = new YoBoolean(forceSensorDefinition.getSensorName() + "DriftEstimatorInitialized", registry);
      }

      public void setInitializationDuration(double initializationDuration)
      {
         this.initializationDuration = initializationDuration;
         initializationAlpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(1.0 / initializationDuration, updateDT);
      }

      public void reset()
      {
         isInitialized.set(false);
         initialOffset.setToZero();
         currentTimeInInitialization = 0.0;
         estimatedDriftBias.setToZero();
         estimatedDriftBiasFiltered.reset();
         estimatedDriftBiasFiltered.update();
      }

      public void update()
      {
         endEffectorWrenchEstimator.calculate();

         if (Math.abs(endEffectorWrenchEstimator.getJacobianDeterminant()) < minJacobianDeterminant.getValue())
         {
            // Singular configuration, the estimator output is not be accurate.
            return;
         }

         forceSensorData.getWrench(measuredWrench);

         if (!isInitialized.getValue())
         {
            currentTimeInInitialization += updateDT;

            spatialVector.setIncludingFrame(endEffectorWrenchEstimator.getExternalWrench());
            spatialVector.sub(measuredWrench);
            initialOffset.getAngularPart().interpolate(spatialVector.getAngularPart(), initialOffset.getAngularPart(), initializationAlpha);
            initialOffset.getLinearPart().interpolate(spatialVector.getLinearPart(), initialOffset.getLinearPart(), initializationAlpha);

            if (currentTimeInInitialization >= initializationDuration)
               isInitialized.set(true);
         }
         else
         {
            estimatedDriftBias.set(measuredWrench);
            estimatedDriftBias.sub(endEffectorWrenchEstimator.getExternalWrench());
            estimatedDriftBias.add(initialOffset);
            estimatedDriftBiasFiltered.update();
         }
      }

      public SpatialVectorReadOnly getEstimatedDriftBias()
      {
         return estimatedDriftBiasFiltered;
      }
   }
}

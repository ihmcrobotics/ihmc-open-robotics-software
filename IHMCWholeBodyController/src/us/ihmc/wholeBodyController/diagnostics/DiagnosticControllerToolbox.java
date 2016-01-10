package us.ihmc.wholeBodyController.diagnostics;

import java.util.Map;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticSensorProcessingConfiguration;
import us.ihmc.sensorProcessing.diagnostic.IMUSensorValidityChecker;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointForceTrackingDelayEstimator;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointSensorValidityChecker;
import us.ihmc.sensorProcessing.diagnostic.PositionVelocity1DConsistencyChecker;
import us.ihmc.sensorProcessing.diagnostic.WrenchSensorValidityChecker;

public class DiagnosticControllerToolbox
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable yoTime;

   private final DiagnosticParameters diagnosticParameters;

   private final Map<OneDoFJoint, OneDoFJointSensorValidityChecker> jointSensorValidityCheckers;
   private final Map<IMUDefinition, IMUSensorValidityChecker> imuSensorValidityCheckers;
   private final Map<ForceSensorDefinition, WrenchSensorValidityChecker> wrenchSensorValidityCheckers;
   private final Map<OneDoFJoint, PositionVelocity1DConsistencyChecker> jointPositionVelocityConsistencyCheckers;
   private final Map<OneDoFJoint, OneDoFJointForceTrackingDelayEstimator> jointForceTrackingDelayEstimators;

   private final double dt;
   private final FullHumanoidRobotModel fullRobotModel;
   private final WalkingControllerParameters walkingControllerParameters;

   public DiagnosticControllerToolbox(FullHumanoidRobotModel fullRobotModel, DiagnosticParameters diagnosticParameters, WalkingControllerParameters walkingControllerParameters, DoubleYoVariable yoTime,
         double dt, DiagnosticSensorProcessingConfiguration diagnosticSensorProcessingConfiguration, YoVariableRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModel;
      this.yoTime = yoTime;
      this.dt = dt;

      this.diagnosticParameters = diagnosticParameters;
      this.walkingControllerParameters = walkingControllerParameters;

      jointSensorValidityCheckers = diagnosticSensorProcessingConfiguration.getJointSensorValidityCheckers();
      imuSensorValidityCheckers = diagnosticSensorProcessingConfiguration.getIMUSensorValidityCheckers();
      wrenchSensorValidityCheckers = diagnosticSensorProcessingConfiguration.getWrenchSensorValidityCheckers();
      jointPositionVelocityConsistencyCheckers = diagnosticSensorProcessingConfiguration.getJointPositionVelocityConsistencyCheckers();
      jointForceTrackingDelayEstimators = diagnosticSensorProcessingConfiguration.getJointForceTrackingDelayEstimators();

      parentRegistry.addChild(registry);
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public DiagnosticParameters getDiagnosticParameters()
   {
      return diagnosticParameters;
   }

   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return walkingControllerParameters;
   }

   public DoubleYoVariable getYoTime()
   {
      return yoTime;
   }

   public double getDT()
   {
      return dt;
   }

   public OneDoFJointSensorValidityChecker getJointSensorValidityChecker(OneDoFJoint joint)
   {
      return jointSensorValidityCheckers.get(joint);
   }

   public IMUSensorValidityChecker getIMUSensorValidityChecker(IMUDefinition imuDefinition)
   {
      return imuSensorValidityCheckers.get(imuDefinition);
   }

   public WrenchSensorValidityChecker getWrenchSensorValidityChecker(ForceSensorDefinition wrenchSensorDefinition)
   {
      return wrenchSensorValidityCheckers.get(wrenchSensorDefinition);
   }

   public PositionVelocity1DConsistencyChecker getJointPositionVelocityConsistencyChecker(OneDoFJoint joint)
   {
      return jointPositionVelocityConsistencyCheckers.get(joint);
   }

   public OneDoFJointForceTrackingDelayEstimator getJointForceTrackingDelayEstimator(OneDoFJoint joint)
   {
      return jointForceTrackingDelayEstimators.get(joint);
   }
}

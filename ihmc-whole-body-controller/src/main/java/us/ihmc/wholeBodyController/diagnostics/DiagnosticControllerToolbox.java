package us.ihmc.wholeBodyController.diagnostics;

import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticSensorProcessingConfiguration;
import us.ihmc.sensorProcessing.diagnostic.IMUSensorValidityChecker;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointForceTrackingDelayEstimator;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointFourierAnalysis;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointSensorValidityChecker;
import us.ihmc.sensorProcessing.diagnostic.OrientationAngularVelocityConsistencyChecker;
import us.ihmc.sensorProcessing.diagnostic.PositionVelocity1DConsistencyChecker;
import us.ihmc.sensorProcessing.diagnostic.WrenchSensorValidityChecker;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DiagnosticControllerToolbox
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble yoTime;

   private final DiagnosticParameters diagnosticParameters;

   private final Map<OneDoFJoint, OneDoFJointSensorValidityChecker> jointSensorValidityCheckers;
   private final Map<String, IMUSensorValidityChecker> imuSensorValidityCheckers;
   private final Map<ForceSensorDefinition, WrenchSensorValidityChecker> wrenchSensorValidityCheckers;

   private final Map<OneDoFJoint, PositionVelocity1DConsistencyChecker> jointPositionVelocityConsistencyCheckers;
   private final Map<String, OrientationAngularVelocityConsistencyChecker> orientationAngularVelocityConsistencyCheckers;

   private final Map<OneDoFJoint, OneDoFJointForceTrackingDelayEstimator> jointForceTrackingDelayEstimators;
   private final Map<OneDoFJoint, OneDoFJointFourierAnalysis> jointFourierAnalysisMap;

   private final double dt;
   private final FullHumanoidRobotModel fullRobotModel;
   private final JointDesiredOutputList lowLevelOutput;
   private final WalkingControllerParameters walkingControllerParameters;
   private final SensorOutputMapReadOnly sensorOutputMap;

   public DiagnosticControllerToolbox(FullHumanoidRobotModel fullRobotModel, JointDesiredOutputList lowLevelOutput, SensorOutputMapReadOnly sensorOutputMap, DiagnosticParameters diagnosticParameters,
         WalkingControllerParameters walkingControllerParameters, YoDouble yoTime, double dt,
         DiagnosticSensorProcessingConfiguration diagnosticSensorProcessingConfiguration, YoVariableRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModel;
      this.lowLevelOutput = lowLevelOutput;
      this.yoTime = yoTime;
      this.dt = dt;

      this.sensorOutputMap = sensorOutputMap;
      this.diagnosticParameters = diagnosticParameters;
      this.walkingControllerParameters = walkingControllerParameters;

      jointSensorValidityCheckers = diagnosticSensorProcessingConfiguration.getJointSensorValidityCheckers();
      imuSensorValidityCheckers = diagnosticSensorProcessingConfiguration.getIMUSensorValidityCheckers();
      wrenchSensorValidityCheckers = diagnosticSensorProcessingConfiguration.getWrenchSensorValidityCheckers();

      jointPositionVelocityConsistencyCheckers = diagnosticSensorProcessingConfiguration.getJointPositionVelocityConsistencyCheckers();
      orientationAngularVelocityConsistencyCheckers = diagnosticSensorProcessingConfiguration.getIMUOrientationAngularVelocityConsistencyCheckers();

      jointForceTrackingDelayEstimators = diagnosticSensorProcessingConfiguration.getJointForceTrackingDelayEstimators();
      jointFourierAnalysisMap = diagnosticSensorProcessingConfiguration.getJointFourierAnalysisMap();

      parentRegistry.addChild(registry);
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public JointDesiredOutputList getLowLevelOutput()
   {
      return lowLevelOutput;
   }

   public DiagnosticParameters getDiagnosticParameters()
   {
      return diagnosticParameters;
   }

   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return walkingControllerParameters;
   }

   public YoDouble getYoTime()
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

   public IMUSensorValidityChecker getIMUSensorValidityChecker(String imuName)
   {
      return imuSensorValidityCheckers.get(imuName);
   }

   public WrenchSensorValidityChecker getWrenchSensorValidityChecker(ForceSensorDefinition wrenchSensorDefinition)
   {
      return wrenchSensorValidityCheckers.get(wrenchSensorDefinition);
   }

   public PositionVelocity1DConsistencyChecker getJointPositionVelocityConsistencyChecker(OneDoFJoint joint)
   {
      return jointPositionVelocityConsistencyCheckers.get(joint);
   }

   public OrientationAngularVelocityConsistencyChecker getIMUOrientationAngularVelocityConsistencyChecker(String imuName)
   {
      return orientationAngularVelocityConsistencyCheckers.get(imuName);
   }

   public OneDoFJointForceTrackingDelayEstimator getJointForceTrackingDelayEstimator(OneDoFJoint joint)
   {
      return jointForceTrackingDelayEstimators.get(joint);
   }

   public OneDoFJointFourierAnalysis getJointFourierAnalysis(OneDoFJoint joint)
   {
      return jointFourierAnalysisMap.get(joint);
   }

   public IMUSensorReadOnly getIMUSensorReadOnly(String imuName)
   {
      List<? extends IMUSensorReadOnly> imuProcessedOutputs = sensorOutputMap.getIMUProcessedOutputs();
      for (int i = 0; i < imuProcessedOutputs.size(); i++)
      {
         if (imuProcessedOutputs.get(i).getSensorName().equals(imuName))
            return imuProcessedOutputs.get(i);
      }
      return null;
   }
}

package us.ihmc.sensorProcessing.diagnostic;

import java.util.List;
import java.util.Map;

import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;

public class DiagnosticSensorProcessingConfiguration implements SensorProcessingConfiguration
{
   private final SensorProcessingConfiguration sensorProcessingConfiguration;

   private Map<OneDoFJoint, OneDoFJointSensorValidityChecker> jointSensorValidityCheckers;
   private Map<String, IMUSensorValidityChecker> imuSensorValidityCheckers;
   private Map<ForceSensorDefinition, WrenchSensorValidityChecker> wrenchSensorValidityCheckers;

   private Map<OneDoFJoint, PositionVelocity1DConsistencyChecker> jointPositionVelocityConsistencyCheckers;
   private Map<String, OrientationAngularVelocityConsistencyChecker> imuOrientationAngularVelocityConsistencyCheckers;

   private Map<OneDoFJoint, OneDoFJointForceTrackingDelayEstimator> jointForceTrackingDelayEstimators;
   private Map<OneDoFJoint, OneDoFJointFourierAnalysis> jointFourierAnalysisMap;
   
   private final JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder;

   private final double dt;
   private final DiagnosticParameters diagnosticParameters;
   private final boolean enableLogging;

   public DiagnosticSensorProcessingConfiguration(DiagnosticParameters diagnosticParameters, double dt, JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder)
   {
      this(diagnosticParameters, null, dt, lowLevelOneDoFJointDesiredDataHolder);
   }

   public DiagnosticSensorProcessingConfiguration(DiagnosticParameters diagnosticParameters, SensorProcessingConfiguration sensorProcessingConfiguration, JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder)
   {
      this(diagnosticParameters, sensorProcessingConfiguration, sensorProcessingConfiguration.getEstimatorDT(), lowLevelOneDoFJointDesiredDataHolder);
   }

   private DiagnosticSensorProcessingConfiguration(DiagnosticParameters diagnosticParameters, SensorProcessingConfiguration sensorProcessingConfiguration, double dt, JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder)
   {
      this.sensorProcessingConfiguration = sensorProcessingConfiguration;
      this.dt = dt;
      this.lowLevelOneDoFJointDesiredDataHolder = lowLevelOneDoFJointDesiredDataHolder;
      this.diagnosticParameters = diagnosticParameters;
      enableLogging = diagnosticParameters.enableLogging();
   }

   @Override
   public void configureSensorProcessing(SensorProcessing sensorProcessing)
   {
      if (sensorProcessingConfiguration != null)
         sensorProcessingConfiguration.configureSensorProcessing(sensorProcessing);

      List<String> jointsToIgnore = diagnosticParameters.getJointsToIgnoreDuringDiagnostic();
      double fftObservationWindow = diagnosticParameters.getFFTObservationWindow();

      jointSensorValidityCheckers = sensorProcessing.addJointSensorValidityCheckers(enableLogging, lowLevelOneDoFJointDesiredDataHolder, jointsToIgnore);
      imuSensorValidityCheckers = sensorProcessing.addIMUSensorValidityCheckers(enableLogging);
      wrenchSensorValidityCheckers = sensorProcessing.addWrenchSensorValidityCheckers(enableLogging);

      jointPositionVelocityConsistencyCheckers = sensorProcessing.addJointPositionVelocityConsistencyCheckers(jointsToIgnore);
      imuOrientationAngularVelocityConsistencyCheckers = sensorProcessing.addIMUOrientationAngularVelocityConsistencyCheckers();

      jointForceTrackingDelayEstimators = sensorProcessing.addJointForceTrackingDelayEstimators(jointsToIgnore, lowLevelOneDoFJointDesiredDataHolder);
      jointFourierAnalysisMap = sensorProcessing.addJointFourierAnalysis(fftObservationWindow, jointsToIgnore, lowLevelOneDoFJointDesiredDataHolder);

      double delayEstimatorFilterBreakFrequency = diagnosticParameters.getDelayEstimatorFilterBreakFrequency();
      double delayEstimatorIntputSignalsSMAWindow = diagnosticParameters.getDelayEstimatorIntputSignalsSMAWindow();
      double maxAbsoluteLead = diagnosticParameters.getDelayEstimatorMaximumLead();
      double maxAbsoluteLag = diagnosticParameters.getDelayEstimatorMaximumLag();
      double observationWindow = diagnosticParameters.getDelayEstimatorObservationWindow();

      for (PositionVelocity1DConsistencyChecker consistencyChecker : jointPositionVelocityConsistencyCheckers.values())
      {
         consistencyChecker.setDelayEstimatorAlphaFilterBreakFrequency(delayEstimatorFilterBreakFrequency);
         consistencyChecker.setInputSignalsSMAWindow(delayEstimatorIntputSignalsSMAWindow);
         consistencyChecker.setDelayEstimationParameters(maxAbsoluteLead, maxAbsoluteLag, observationWindow);
      }

      for (OneDoFJointForceTrackingDelayEstimator forceTracking : jointForceTrackingDelayEstimators.values())
      {
         forceTracking.setAlphaFilterBreakFrequency(delayEstimatorFilterBreakFrequency);
         forceTracking.setEstimationParameters(maxAbsoluteLead, maxAbsoluteLag, observationWindow);
      }
   }

   public Map<OneDoFJoint, OneDoFJointSensorValidityChecker> getJointSensorValidityCheckers()
   {
      return jointSensorValidityCheckers;
   }

   public Map<String, IMUSensorValidityChecker> getIMUSensorValidityCheckers()
   {
      return imuSensorValidityCheckers;
   }

   public Map<ForceSensorDefinition, WrenchSensorValidityChecker> getWrenchSensorValidityCheckers()
   {
      return wrenchSensorValidityCheckers;
   }

   public Map<OneDoFJoint, PositionVelocity1DConsistencyChecker> getJointPositionVelocityConsistencyCheckers()
   {
      return jointPositionVelocityConsistencyCheckers;
   }

   public Map<String, OrientationAngularVelocityConsistencyChecker> getIMUOrientationAngularVelocityConsistencyCheckers()
   {
      return imuOrientationAngularVelocityConsistencyCheckers;
   }

   public Map<OneDoFJoint, OneDoFJointForceTrackingDelayEstimator> getJointForceTrackingDelayEstimators()
   {
      return jointForceTrackingDelayEstimators;
   }

   public Map<OneDoFJoint, OneDoFJointFourierAnalysis> getJointFourierAnalysisMap()
   {
      return jointFourierAnalysisMap;
   }

   @Override
   public SensorNoiseParameters getSensorNoiseParameters()
   {
      return null;
   }

   @Override
   public double getEstimatorDT()
   {
      return dt;
   }
}

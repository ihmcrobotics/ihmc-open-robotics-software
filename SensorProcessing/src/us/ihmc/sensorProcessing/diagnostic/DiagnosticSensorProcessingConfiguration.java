package us.ihmc.sensorProcessing.diagnostic;

import java.util.List;
import java.util.Map;

import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;

public class DiagnosticSensorProcessingConfiguration implements SensorProcessingConfiguration
{
   private final SensorProcessingConfiguration sensorProcessingConfiguration;

   private Map<OneDoFJoint, OneDoFJointSensorValidityChecker> jointSensorValidityCheckers;
   private Map<IMUDefinition, IMUSensorValidityChecker> imuSensorValidityCheckers;
   private Map<ForceSensorDefinition, WrenchSensorValidityChecker> wrenchSensorValidityCheckers;

   private Map<OneDoFJoint, PositionVelocity1DConsistencyChecker> jointPositionVelocityConsistencyCheckers;
   private Map<IMUDefinition, OrientationAngularVelocityConsistencyChecker> imuOrientationAngularVelocityConsistencyCheckers;

   private Map<OneDoFJoint, OneDoFJointForceTrackingDelayEstimator> jointForceTrackingDelayEstimators;
   private Map<OneDoFJoint, OneDoFJointFourierAnalysis> jointFourierAnalysisMap;

   private final double dt;
   private final DiagnosticParameters diagnosticParameters;

   public DiagnosticSensorProcessingConfiguration(DiagnosticParameters diagnosticParameters, double dt)
   {
      this(diagnosticParameters, null, dt);
   }

   public DiagnosticSensorProcessingConfiguration(DiagnosticParameters diagnosticParameters, SensorProcessingConfiguration sensorProcessingConfiguration)
   {
      this(diagnosticParameters, sensorProcessingConfiguration, sensorProcessingConfiguration.getEstimatorDT());
   }

   private DiagnosticSensorProcessingConfiguration(DiagnosticParameters diagnosticParameters, SensorProcessingConfiguration sensorProcessingConfiguration, double dt)
   {
      this.sensorProcessingConfiguration = sensorProcessingConfiguration;
      this.dt = dt;
      this.diagnosticParameters = diagnosticParameters;
   }

   @Override
   public void configureSensorProcessing(SensorProcessing sensorProcessing)
   {
      if (sensorProcessingConfiguration != null)
         sensorProcessingConfiguration.configureSensorProcessing(sensorProcessing);

      List<String> jointsToIgnore = diagnosticParameters.getJointsToIgnoreDuringDiagnostic();
      double fftObservationWindow = diagnosticParameters.getFFTObservationWindow();

      jointSensorValidityCheckers = sensorProcessing.addJointSensorValidityCheckers(true, jointsToIgnore);
      imuSensorValidityCheckers = sensorProcessing.addIMUSensorValidityCheckers(true);
      wrenchSensorValidityCheckers = sensorProcessing.addWrenchSensorValidityCheckers(true);

      jointPositionVelocityConsistencyCheckers = sensorProcessing.addJointPositionVelocityConsistencyCheckers(jointsToIgnore);
      imuOrientationAngularVelocityConsistencyCheckers = sensorProcessing.addIMUOrientationAngularVelocityConsistencyCheckers();

      jointForceTrackingDelayEstimators = sensorProcessing.addJointForceTrackingDelayEstimators(jointsToIgnore);
      jointFourierAnalysisMap = sensorProcessing.addJointFourierAnalysis(fftObservationWindow, jointsToIgnore);

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

   public Map<IMUDefinition, IMUSensorValidityChecker> getIMUSensorValidityCheckers()
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

   public Map<IMUDefinition, OrientationAngularVelocityConsistencyChecker> getIMUOrientationAngularVelocityConsistencyCheckers()
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

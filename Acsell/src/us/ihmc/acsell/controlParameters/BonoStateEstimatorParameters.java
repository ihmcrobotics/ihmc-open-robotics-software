package us.ihmc.acsell.controlParameters;

import us.ihmc.darpaRoboticsChallenge.stateEstimation.DRCSimulatedSensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorFilterParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.PointMeasurementNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;

public class BonoStateEstimatorParameters implements StateEstimatorParameters
{
   private final boolean runningOnRealRobot;

   private final double estimatorDT;

   private final boolean useKinematicsBasedStateEstimator = true;
   private final boolean assumePerfectIMU = true;

   private final double jointVelocitySlopTimeForBacklashCompensation;

   private final double jointPositionFilterFrequencyHz;
   private final double jointVelocityFilterFrequencyHz;
   private final double orientationFilterFrequencyHz;
   private final double angularVelocityFilterFrequencyHz;
   private final double linearAccelerationFilterFrequencyHz;

   // State Estimator Filter Parameters
   private final double pointVelocityXYMeasurementStandardDeviation;
   private final double pointVelocityZMeasurementStandardDeviation;

   private final double pointPositionXYMeasurementStandardDeviation;
   private final double pointPositionZMeasurementStandardDeviation;

   private final SensorFilterParameters sensorFilterParameters;

   private final PointMeasurementNoiseParameters pointMeasurementNoiseParameters;

   //      DRCSimulatedSensorNoiseParameters.createNoiseParametersForEstimatorJerryTuning();
   private final SensorNoiseParameters sensorNoiseParameters = DRCSimulatedSensorNoiseParameters
         .createNoiseParametersForEstimatorJerryTuningSeptember2013();

   public BonoStateEstimatorParameters(boolean runningOnRealRobot, double estimatorDT)
   {
      this.runningOnRealRobot = runningOnRealRobot;

      this.estimatorDT = estimatorDT;

      final double defaultFilterBreakFrequency;

      if (!runningOnRealRobot)
      {
         defaultFilterBreakFrequency = Double.POSITIVE_INFINITY;
      }
      else
      {
         defaultFilterBreakFrequency = 16.0;
      }

      jointPositionFilterFrequencyHz = defaultFilterBreakFrequency;
      jointVelocityFilterFrequencyHz = defaultFilterBreakFrequency;
      orientationFilterFrequencyHz = defaultFilterBreakFrequency;
      angularVelocityFilterFrequencyHz = defaultFilterBreakFrequency;
      linearAccelerationFilterFrequencyHz = defaultFilterBreakFrequency;

      jointVelocitySlopTimeForBacklashCompensation = 0.03;

      pointVelocityXYMeasurementStandardDeviation = 2.0;
      pointVelocityZMeasurementStandardDeviation = 2.0;

      pointPositionXYMeasurementStandardDeviation = 0.1;
      pointPositionZMeasurementStandardDeviation = 0.1;

      boolean useTwoPolesForIMUFiltering = false;
      boolean doFiniteDifferenceForJointVelocities = false;
      
      sensorFilterParameters = new SensorFilterParameters(jointPositionFilterFrequencyHz, jointVelocityFilterFrequencyHz, orientationFilterFrequencyHz,
            angularVelocityFilterFrequencyHz, linearAccelerationFilterFrequencyHz, jointVelocitySlopTimeForBacklashCompensation, estimatorDT, useTwoPolesForIMUFiltering, doFiniteDifferenceForJointVelocities);

      pointMeasurementNoiseParameters = new PointMeasurementNoiseParameters(pointVelocityXYMeasurementStandardDeviation,
            pointVelocityZMeasurementStandardDeviation, pointPositionXYMeasurementStandardDeviation, pointPositionZMeasurementStandardDeviation);
   }

   public SensorFilterParameters getSensorFilterParameters()
   {
      return sensorFilterParameters;
   }

   public boolean getAssumePerfectIMU()
   {
      return assumePerfectIMU;
   }

   public boolean useKinematicsBasedStateEstimator()
   {
      return useKinematicsBasedStateEstimator;
   }

   public PointMeasurementNoiseParameters getPointMeasurementNoiseParameters()
   {
      return pointMeasurementNoiseParameters;
   }

   public SensorNoiseParameters getSensorNoiseParameters()
   {
      return sensorNoiseParameters;
   }

   public double getEstimatorDT()
   {
      return estimatorDT;
   }

   public boolean isRunningOnRealRobot()
   {
      return runningOnRealRobot;
   }

   public double getKinematicsPelvisPositionFilterFreqInHertz()
   {
      return Double.POSITIVE_INFINITY;
   }

   public double getKinematicsPelvisLinearVelocityFilterFreqInHertz()
   {
      return 16.0;
   }

   public double getCoPFilterFreqInHertz()
   {
      return 4.0;
   }

   public boolean useAccelerometerForEstimation()
   {
      return false;
   }

   public boolean useHackishAccelerationIntegration()
   {
      return false;
   }

   public boolean estimateGravity()
   {
      return false;
   }

   public double getGravityFilterFreqInHertz()
   {
      return 5.3052e-4;
   }

   public double getPelvisPositionFusingFrequency()
   {
      return 11.7893; // alpha = 0.8 with dt = 0.003
   }

   public double getPelvisLinearVelocityFusingFrequency()
   {
      return 0.4261; // alpha = 0.992 with dt = 0.003
   }

   public double getDelayTimeForTrustingFoot()
   {
      return 0.02;
   }

   public double getForceInPercentOfWeightThresholdToTrustFoot()
   {
      return 0.3;
   }

   public boolean estimateIMUDrift()
   {
      return false;
   }

   public boolean compensateIMUDrift()
   {
      return false;
   }

   public double getIMUDriftFilterFreqInHertz()
   {
      return 0.5332;
   }

   public double getFootVelocityUsedForImuDriftFilterFreqInHertz()
   {
      return 0.5332;
   }

   public double getFootVelocityThresholdToEnableIMUDriftCompensation()
   {
      return 0.03;
   }

   public boolean trustCoPAsNonSlippingContactPoint()
   {
      return true;
   }

   public boolean useTwistForPelvisLinearStateEstimation()
   {
      return false;
   }

   @Override
   public double getPelvisLinearVelocityAlphaNewTwist()
   {
      // TODO Tune
      return 1.0;
   }
}

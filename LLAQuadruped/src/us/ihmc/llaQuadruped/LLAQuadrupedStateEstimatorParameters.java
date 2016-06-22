package us.ihmc.llaQuadruped;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;

public class LLAQuadrupedStateEstimatorParameters implements StateEstimatorParameters
{
   @Override
   public void configureSensorProcessing(SensorProcessing sensorProcessing)
   {
   }

   @Override
   public SensorNoiseParameters getSensorNoiseParameters()
   {
      return null;
   }

   @Override
   public boolean isRunningOnRealRobot()
   {
      return false;
   }

   @Override
   public double getEstimatorDT()
   {
      return 0;
   }

   @Override
   public boolean trustCoPAsNonSlippingContactPoint()
   {
      return false;
   }

   @Override
   public boolean useControllerDesiredCenterOfPressure()
   {
      return false;
   }

   @Override
   public boolean useIMUsForSpineJointVelocityEstimation()
   {
      return false;
   }

   @Override
   public double getAlphaIMUsForSpineJointVelocityEstimation()
   {
      return 0;
   }

   @Override
   public ImmutablePair<String, String> getIMUsForSpineJointVelocityEstimation()
   {
      return null;
   }

   @Override
   public boolean requestWristForceSensorCalibrationAtStart()
   {
      return false;
   }

   @Override
   public SideDependentList<String> getWristForceSensorNames()
   {
      return null;
   }

   @Override
   public boolean requestFootForceSensorCalibrationAtStart()
   {
      return false;
   }

   @Override
   public SideDependentList<String> getFootForceSensorNames()
   {
      return null;
   }

   @Override
   public double getKinematicsPelvisPositionFilterFreqInHertz()
   {
      return 0;
   }

   @Override
   public double getKinematicsPelvisLinearVelocityFilterFreqInHertz()
   {
      return 0;
   }

   @Override
   public double getCoPFilterFreqInHertz()
   {
      return 0;
   }

   @Override
   public boolean useAccelerometerForEstimation()
   {
      return false;
   }

   @Override
   public boolean estimateAccelerationBias()
   {
      return false;
   }

   @Override
   public boolean cancelGravityFromAccelerationMeasurement()
   {
      return false;
   }

   @Override
   public double getAccelerationBiasFilterFreqInHertz()
   {
      return 0;
   }

   @Override
   public double getPelvisPositionFusingFrequency()
   {
      return 0;
   }

   @Override
   public double getPelvisLinearVelocityFusingFrequency()
   {
      return 0;
   }

   @Override
   public double getPelvisVelocityBacklashSlopTime()
   {
      return 0;
   }

   @Override
   public double getDelayTimeForTrustingFoot()
   {
      return 0;
   }

   @Override
   public double getForceInPercentOfWeightThresholdToTrustFoot()
   {
      return 0;
   }

   @Override
   public boolean estimateIMUDrift()
   {
      return false;
   }

   @Override
   public boolean compensateIMUDrift()
   {
      return false;
   }

   @Override
   public double getIMUDriftFilterFreqInHertz()
   {
      return 0;
   }

   @Override
   public double getFootVelocityUsedForImuDriftFilterFreqInHertz()
   {
      return 0;
   }

   @Override
   public double getFootVelocityThresholdToEnableIMUDriftCompensation()
   {
      return 0;
   }

   @Override
   public boolean useTwistForPelvisLinearStateEstimation()
   {
      return false;
   }

   @Override
   public double getPelvisLinearVelocityAlphaNewTwist()
   {
      return 0;
   }

   @Override
   public boolean createFusedIMUSensor()
   {
      return false;
   }

   @Override
   public double getContactThresholdForce()
   {
      return 0;
   }

   @Override
   public double getFootSwitchCoPThresholdFraction()
   {
      return 0;
   }

   @Override
   public double getContactThresholdHeight()
   {
      return 0;
   }

   @Override
   public FootSwitchType getFootSwitchType()
   {
      return null;
   }
}

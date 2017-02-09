package us.ihmc.llaQuadrupedController.parameters;

import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;

public class LLAQuadrupedStateEstimatorParameters extends StateEstimatorParameters
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
   public double getCoPFilterFreqInHertz()
   {
      return 0;
   }

   @Override
   public boolean enableIMUBiasCompensation()
   {
      return false;
   }

   @Override
   public boolean enableIMUYawDriftCompensation()
   {
      return false;
   }

   @Override
   public double getIMUBiasFilterFreqInHertz()
   {
      return 6.0e-3;
   }

   @Override
   public double getIMUYawDriftFilterFreqInHertz()
   {
      return 1.0e-3;
   }

   @Override
   public double getIMUBiasVelocityThreshold()
   {
      return 0.015;
   }

   @Override
   public boolean useAccelerometerForEstimation()
   {
      return false;
   }

   @Override
   public boolean cancelGravityFromAccelerationMeasurement()
   {
      return false;
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
   public double getPelvisLinearVelocityAlphaNewTwist()
   {
      return 0;
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

   @Override
   public boolean getPelvisLinearStateUpdaterTrustImuWhenNoFeetAreInContact()
   {
      return false;
   }
   
   @Override
   public double getCenterOfMassVelocityFusingFrequency()
   {
      return 0.4261;
   }

   @Override
   public boolean useGroundReactionForcesToComputeCenterOfMassVelocity()
   {
      return false;
   }
}

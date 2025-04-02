package us.ihmc.alexander.parameters.controller;

import us.ihmc.alexander.AlexanderJointMap;
import us.ihmc.alexander.AlexanderSensorInformation;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitchFactory;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.stateEstimation.IMUBasedJointStateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;

import java.util.ArrayList;
import java.util.List;

public class AlexanderStateEstimatorParameters extends StateEstimatorParameters
{
   private final double estimatorDT;
   private final RobotTarget target;
   private final AlexanderJointMap jointMap;

   private final double kinematicsPelvisPositionFilterFreqInHertz;

   private final SideDependentList<String> footForceSensorNames;

   private final List<IMUBasedJointStateEstimatorParameters> imuBasedJointStateEstimatorParameters = new ArrayList<>();

   private final AlexanderSensorInformation sensorInformation;

   public AlexanderStateEstimatorParameters(double estimatorDT, RobotTarget target, AlexanderSensorInformation sensorInformation, AlexanderJointMap jointMap)
   {
      this.target = target;
      this.estimatorDT = estimatorDT;
      this.sensorInformation = sensorInformation;
      this.jointMap = jointMap;

      this.footForceSensorNames = sensorInformation.getFeetForceSensorNames();

      kinematicsPelvisPositionFilterFreqInHertz = Double.POSITIVE_INFINITY;

      String pelvisIMU = sensorInformation.getPrimaryBodyImu();

      if (pelvisIMU != null && target == RobotTarget.REAL_ROBOT)
      {
         double breakFrequencyForPositionEstimation = 2.0;
         double breakFrequencyForVelocityEstimation = 0.15;

         String torsoIMU = sensorInformation.getTorsoIMUName();

         if (torsoIMU != null)
         {
            imuBasedJointStateEstimatorParameters.add(new IMUBasedJointStateEstimatorParameters("Spine",
                                                                                                true,
                                                                                                pelvisIMU,
                                                                                                torsoIMU,
                                                                                                breakFrequencyForVelocityEstimation,
                                                                                                breakFrequencyForPositionEstimation));
         }
      }
   }

   @Override
   public void configureSensorProcessing(SensorProcessing sensorProcessing)
   {

   }

   @Override
   public double getEstimatorDT()
   {
      return estimatorDT;
   }

   @Override
   public boolean usePelvisLinearStateNewFusingFilter()
   {
      return true;
   }

   @Override
   public double getKinematicsPelvisPositionFilterFreqInHertz()
   {
      return kinematicsPelvisPositionFilterFreqInHertz;
   }

   @Override
   public double getCoPFilterFreqInHertz()
   {
      return Double.POSITIVE_INFINITY;
   }

   @Override
   public boolean enableIMUBiasCompensation()
   {
      return true;
   }

   @Override
   public double getIMUBiasFilterFreqInHertz()
   {
      return 0.02;
   }

   @Override
   public double getIMUBiasVelocityThreshold()
   {
      // TODO Tune me
      return 0.015;
   }

   @Override
   public boolean correctTrustedFeetPositions()
   {
      return true;
   }

   @Override
   public boolean useAccelerometerForEstimation()
   {
      return true;
   }

   @Override
   public boolean cancelGravityFromAccelerationMeasurement()
   {
      return true;
   }

   @Override
   public double getPelvisPositionFusingFrequency()
   {
      // TODO Tune me
      return 11.7893;
   }

   @Override
   public double getPelvisLinearVelocityFusingFrequency()
   {
      // TODO Tune me
      return 2.0146195328088035;
   }

   @Override
   public double getDelayTimeForTrustingFoot()
   {
      // TODO Tune me
      return 0.02;
   }

   @Override
   public double getForceInPercentOfWeightThresholdToTrustFoot()
   {
      // TODO Tune me
      return 0.3;
   }

   @Override
   public boolean trustCoPAsNonSlippingContactPoint()
   {
      return true;
   }

   @Override
   public double getPelvisLinearVelocityAlphaNewTwist()
   {
      return 0.15;
   }

   @Override
   public boolean requestFootForceSensorCalibrationAtStart()
   {
      return false;
   }

   @Override
   public SideDependentList<String> getFootForceSensorNames()
   {
      return footForceSensorNames;
   }

   @Override
   public boolean getPelvisLinearStateUpdaterTrustImuWhenNoFeetAreInContact()
   {
      return true;
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

   @Override
   public FootSwitchFactory getFootSwitchFactory()
   {
      WrenchBasedFootSwitchFactory factory = new WrenchBasedFootSwitchFactory();
      factory.setDefaultContactThresholdForce(80.0);
      factory.setDefaultCoPThresholdFraction(0.02);
      factory.setDefaultSecondContactThresholdForceIgnoringCoP(Double.POSITIVE_INFINITY);
      return factory;
   }

   @Override
   public List<IMUBasedJointStateEstimatorParameters> getIMUBasedJointStateEstimatorParameters()
   {
      return imuBasedJointStateEstimatorParameters;
   }
}

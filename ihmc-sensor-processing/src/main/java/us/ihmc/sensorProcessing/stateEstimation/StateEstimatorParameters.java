package us.ihmc.sensorProcessing.stateEstimation;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.commons.UnitConversions;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;

public class StateEstimatorParameters
{
   public static final double ROBOT_CONFIGURATION_DATA_PUBLISH_DT = UnitConversions.hertzToSeconds(120.0);
   private static final double DEFAULT_ESTIMATE_DT = 0.001;

   public void configureSensorProcessing(SensorProcessing sensorProcessing)
   {
   }

   public double getEstimatorDT()
   {
      return DEFAULT_ESTIMATE_DT;
   }

   public boolean trustCoPAsNonSlippingContactPoint()
   {
      return true;
   }

   public boolean useControllerDesiredCenterOfPressure()
   {
      return false;
   }

   public List<IMUBasedJointStateEstimatorParameters> getIMUBasedJointStateEstimatorParameters()
   {
      return new ArrayList<>();
   }

   public boolean requestWristForceSensorCalibrationAtStart()
   {
      return false;
   }

   public SideDependentList<String> getWristForceSensorNames()
   {
      return null;
   }

   public boolean requestFootForceSensorCalibrationAtStart()
   {
      return false;
   }

   public boolean requestFrozenModeAtStart()
   {
      return false;
   }

   public SideDependentList<String> getFootForceSensorNames()
   {
      SideDependentList<String> footSwitch = new SideDependentList<>();
      footSwitch.put(RobotSide.LEFT, "leftFootFTSensor");
      footSwitch.put(RobotSide.RIGHT, "rightFootFTSensor");
      return footSwitch;
   }

   // Parameters related to the kinematics based state estimator
   public double getKinematicsPelvisPositionFilterFreqInHertz()
   {
      return Double.POSITIVE_INFINITY;
   }

   public double getCoPFilterFreqInHertz()
   {
      return Double.POSITIVE_INFINITY;
   }

   /**
    * Used to enable/disabled the IMUYawDriftEstimator
    */
   public boolean enableIMUYawDriftCompensation()
   {
      return false;
   }

   /**
    * When the IMU yaw drift cannot be estimated, the estimated rate of the yaw drift can be used to
    * partially compensate for the drift. Works well when the yaw drift is linear. When false, the yaw
    * drift will only be compensated whn the robot is standing.
    */
   public boolean integrateEstimatedIMUYawDriftRate()
   {
      return false;
   }

   /**
    * The estimation of the yaw drift uses the position of the feet that are loaded and static. This is
    * the delay from the moment a foot becomes loaded and the moment it is used in the estimation.
    */
   public double getIMUYawDriftEstimatorDelayBeforeTrustingFoot()
   {
      return 0.5;
   }

   /**
    * The estimation of the yaw drift uses the position of the feet that are loaded and static. This
    * the maximum value for the linear velocity magnitude for each foot. The estimation will start when
    * all feet are loaded and static.
    */
   public double getIMUYawDriftFootLinearVelocityThreshold()
   {
      return 0.04;
   }

   /**
    * Filter break frequency for the estimation of the yaw drift.
    */
   public double getIMUYawDriftFilterFreqInHertz()
   {
      return 0.8;
   }

   /**
    * Filter break frequency for the estimation of the yaw rate drift.
    */
   public double getIMUYawDriftRateFilterFreqInHertz()
   {
      return 1.5e-3;
   }

   public boolean enableIMUBiasCompensation()
   {
      return true;
   }

   public double getIMUBiasFilterFreqInHertz()
   {
      return 0.04;
   }

   public double getIMUBiasVelocityThreshold()
   {
      return 0.035;
   }

   public boolean useAccelerometerForEstimation()
   {
      return true;
   }

   public boolean cancelGravityFromAccelerationMeasurement()
   {
      return true;
   }

   public double getPelvisPositionFusingFrequency()
   {
      return 11.7893;
   }

   /** The smaller the value, the more it trusts the IMU **/
   public double getPelvisLinearVelocityFusingFrequency()
   {
      return 2.0146195328088035;
   }

   /**
    * The new fusing filter continuously estimates the bias from the accelerometer when integrating
    * into pelvis linear velocity and then position.
    */
   public boolean usePelvisLinearStateNewFusingFilter()
   {
      return true;
   }

   /**
    * Parameter for the new pelvis linear state fusing filter.
    *
    * @return proportional gain to correct the integrated linear velocity using the information from
    *       the kinematics. A lower value means less correction, thus trusting more the IMU.
    */
   public double getPelvisPositionNewFusingFilterKp()
   {
      return 0.05;
   }

   /**
    * Parameter for the new pelvis linear state fusing filter.
    *
    * @return integral gain used to estimate the linear velocity bias. A lower value means a slower
    *       update of the bias.
    */
   public double getPelvisPositionNewFusingFilterKi()
   {
      return 1.0e-4;
   }

   /**
    * Parameter for the new pelvis linear state fusing filter.
    *
    * @return proportional gain to correct the integrated linear acceleration using the information
    *       from the kinematics. A lower value means less correction, thus trusting more the IMU.
    */
   public double getPelvisLinearVelocityNewFusingFilterKp()
   {
      return 0.025;
   }

   /**
    * Parameter for the new pelvis linear state fusing filter.
    *
    * @return integral gain used to estimate the linear acceleration bias. A lower value means a slower
    *       update of the bias.
    */
   public double getPelvisLinearVelocityNewFusingFilterKi()
   {
      return 1.0e-4;
   }

   public MomentumEstimatorMode getMomentumEstimatorMode()
   {
      return MomentumEstimatorMode.SIMPLE;
   }

   /**
    * Parameter for whether the CoM position adjustment in the {@link MomentumEstimatorMode#DISTRIBUTED_IMUS} module is used
    */
   public boolean enableCoMPositionAdjustment()
   {
      return false;
   }

   /**
    * Parameter for whether the CoM velocity adjustment in the {@link MomentumEstimatorMode#DISTRIBUTED_IMUS} module is used
    */
   public boolean enableCoMVelocityAdjustment()
   {
      return false;
   }

   public String[] getIMUsToUseInMomentumEstimator()
   {
      return null;
   }

   /** The smaller the value, the more it trusts the IMU **/
   public double getCenterOfMassVelocityFusingFrequency()
   {
      return 0.4261;
   }

   public double getDelayTimeForTrustingFoot()
   {
      return 0.02;
   }

   public double getForceInPercentOfWeightThresholdToTrustFoot()
   {
      return 0.3;
   }

   public double getForceInPercentOfWeightThresholdToNotTrustFoot()
   {
      return getForceInPercentOfWeightThresholdToTrustFoot();
   }

   public double getAngularVelocityToNotTrustFoot()
   {
      return Double.POSITIVE_INFINITY;
   }

   public double getPelvisLinearVelocityAlphaNewTwist()
   {
      return 0.2;
   }

   public boolean createFootWrenchSensorDriftEstimator()
   {
      return false;
   }

   public FootSwitchFactory getFootSwitchFactory()
   {
      return null;
   }

   public SideDependentList<FootSwitchFactory> getFootSwitchFactories()
   {
      FootSwitchFactory footSwitchFactory = getFootSwitchFactory();
      return new SideDependentList<>(footSwitchFactory, footSwitchFactory);
   }

   public boolean getPelvisLinearStateUpdaterTrustImuWhenNoFeetAreInContact()
   {
      return true;
   }

   public boolean useGroundReactionForcesToComputeCenterOfMassVelocity()
   {
      return false;
   }

   public boolean correctTrustedFeetPositions()
   {
      return true;
   }

   public enum MomentumEstimatorMode
   {
      /** Default mode: the state estimator does not instantiate a momentum estimator, the controller will have to compute it using kinematics data. */
      NONE,
      /** Old implementation from Georg and Jerry: estimates the CoM acceleration from F/T sensors and uses that to refines the momentum estimate. */
      SIMPLE,
      /**
       * Effective when the robot has many IMUs and the kinematics is not trusted (unsensed backlash or elasticity). Exploit as much as possible measurements
       * from IMUs to refine every rigid-body state.
       */
      DISTRIBUTED_IMUS,
      /** Similar to the {@link #SIMPLE} estimator, version from the paper: "Humanoid Momentum Estimation Using Sensed Contact Wrenches". */
      WRENCH_BASED
   }
}

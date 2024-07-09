package us.ihmc.sensorProcessing.stateEstimation;

import java.util.Collections;
import java.util.List;

import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.tools.UnitConversions;

public abstract class StateEstimatorParameters implements SensorProcessingConfiguration
{
   public static final double ROBOT_CONFIGURATION_DATA_PUBLISH_DT = UnitConversions.hertzToSeconds(120.0);

   @Override
   public abstract double getEstimatorDT();

   public abstract boolean trustCoPAsNonSlippingContactPoint();

   public boolean useControllerDesiredCenterOfPressure()
   {
      return false;
   }

   public List<IMUBasedJointStateEstimatorParameters> getIMUBasedJointStateEstimatorParameters()
   {
      return Collections.emptyList();
   }

   public boolean requestWristForceSensorCalibrationAtStart()
   {
      return false;
   }

   public SideDependentList<String> getWristForceSensorNames()
   {
      return null;
   }

   public abstract boolean requestFootForceSensorCalibrationAtStart();

   public boolean requestFrozenModeAtStart()
   {
      return false;
   }

   public abstract SideDependentList<String> getFootForceSensorNames();

   // Parameters related to the kinematics based state estimator
   public abstract double getKinematicsPelvisPositionFilterFreqInHertz();

   public abstract double getCoPFilterFreqInHertz();

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

   public abstract boolean enableIMUBiasCompensation();

   public abstract double getIMUBiasFilterFreqInHertz();

   public abstract double getIMUBiasVelocityThreshold();

   public abstract boolean useAccelerometerForEstimation();

   public abstract boolean cancelGravityFromAccelerationMeasurement();

   public abstract double getPelvisPositionFusingFrequency();

   /** The smaller the value, the more it trusts the IMU **/
   public abstract double getPelvisLinearVelocityFusingFrequency();

   /**
    * The new fusing filter continuously estimates the bias from the accelerometer when integrating
    * into pelvis linear velocity and then position.
    */
   public boolean usePelvisLinearStateNewFusingFilter()
   {
      return false;
   }

   /**
    * Parameter for the new pelvis linear state fusing filter.
    * 
    * @return proportional gain to correct the integrated linear velocity using the information from
    *         the kinematics. A lower value means less correction, thus trusting more the IMU.
    */
   public double getPelvisPositionNewFusingFilterKp()
   {
      return 0.05;
   }

   /**
    * Parameter for the new pelvis linear state fusing filter.
    * 
    * @return integral gain used to estimate the linear velocity bias. A lower value means a slower
    *         update of the bias.
    */
   public double getPelvisPositionNewFusingFilterKi()
   {
      return 1.0e-4;
   }

   /**
    * Parameter for the new pelvis linear state fusing filter.
    * 
    * @return proportional gain to correct the integrated linear acceleration using the information
    *         from the kinematics. A lower value means less correction, thus trusting more the IMU.
    */
   public double getPelvisLinearVelocityNewFusingFilterKp()
   {
      return 0.025;
   }

   /**
    * Parameter for the new pelvis linear state fusing filter.
    * 
    * @return integral gain used to estimate the linear acceleration bias. A lower value means a slower
    *         update of the bias.
    */
   public double getPelvisLinearVelocityNewFusingFilterKi()
   {
      return 1.0e-4;
   }

   public MomentumEstimatorMode getMomentumEstimatorMode()
   {
      return MomentumEstimatorMode.NONE;
   }

   /** The smaller the value, the more it trusts the IMU **/
   public abstract double getCenterOfMassVelocityFusingFrequency();

   public abstract double getDelayTimeForTrustingFoot();

   public abstract double getForceInPercentOfWeightThresholdToTrustFoot();

   public double getForceInPercentOfWeightThresholdToNotTrustFoot()
   {
      return getForceInPercentOfWeightThresholdToTrustFoot();
   }

   public abstract double getPelvisLinearVelocityAlphaNewTwist();

   public boolean createFootWrenchSensorDriftEstimator()
   {
      return false;
   }

   public abstract FootSwitchFactory getFootSwitchFactory();

   public SideDependentList<FootSwitchFactory> getFootSwitchFactories()
   {
      FootSwitchFactory footSwitchFactory = getFootSwitchFactory();
      return new SideDependentList<>(footSwitchFactory, footSwitchFactory);
   }

   public abstract boolean getPelvisLinearStateUpdaterTrustImuWhenNoFeetAreInContact();

   public abstract boolean useGroundReactionForcesToComputeCenterOfMassVelocity();

   public boolean correctTrustedFeetPositions()
   {
      return false;
   }

   public enum MomentumEstimatorMode
   {
      /** Default mode: the state estimator does not instantiate a momentum estimator, the controller will have to compute it using kinematics data. */
      NONE,
      /** Old implementation from Georg and Jerry: estimates the CoM acceleration from F/T sensors and uses that to refines the momentum estimate. */
      SIMPLE,
      /** Effective when the robot has many IMUs and the kinematics is not trusted (unsensed backlash or elasticity). Exploit as much as possible measurements from IMUs to refine every rigid-body state. */
      DISTRIBUTED_IMUS,
      /** Similar to the {@link #SIMPLE} estimator, version from the paper: "Humanoid Momentum Estimation Using Sensed Contact Wrenches". */
      WRENCH_BASED
   }
}

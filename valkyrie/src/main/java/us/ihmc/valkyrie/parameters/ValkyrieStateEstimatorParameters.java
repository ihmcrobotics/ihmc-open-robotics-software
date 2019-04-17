package us.ihmc.valkyrie.parameters;

import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitchFactory;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.valkyrie.fingers.ValkyrieHandJointName;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ValkyrieStateEstimatorParameters extends StateEstimatorParameters
{
   private static final boolean DEBUG_VELOCITY_WITH_FD = false;

   private final RobotTarget target;
   private final double estimatorDT;

   private final double kinematicsPelvisPositionFilterFreqInHertz;

   private final double lowerBodyJointVelocityBacklashSlopTime;
   private final double armJointVelocityBacklashSlopTime;

   private final double armJointPositionFilterFrequencyHz;
   private final double lowerBodyJointPositionFilterFrequencyHz;
   private final double jointOutputEncoderVelocityFilterFrequencyHz;
   private final double lowerBodyJointVelocityFilterFrequencyHz;
   private final double fingerPositionFilterFrequencyHz;
   private final double orientationFilterFrequencyHz;
   private final double angularVelocityFilterFrequencyHz;
   private final double linearAccelerationFilterFrequencyHz;

   private final ValkyrieSensorInformation sensorInformation;
   private final ValkyrieJointMap jointMap;

   private SensorNoiseParameters sensorNoiseParameters = null;

   private final boolean doElasticityCompensation;
   private final double jointElasticityFilterFrequencyHz;
   private final double maximumDeflection;
   private final double defaultJointStiffness;
   private final HashMap<String, Double> jointSpecificStiffness = new HashMap<String, Double>();

   private final SideDependentList<String> footForceSensorNames;
   private final SideDependentList<String> wristForceSensorNames;

   /*
    * Challenge with Valkyrie is that we need minimum delay in all signal, which
    * is usual but it does matter way more for Valkyrie as there is almost no
    * natural damping. In the same spirit, all the signals need to be
    * consistent, for instance joint velocity should be in phase with joint
    * position.
    */
   public ValkyrieStateEstimatorParameters(RobotTarget target, double estimatorDT, ValkyrieSensorInformation sensorInformation,
                                           ValkyrieJointMap jointMap)
   {
      this.target = target;
      this.estimatorDT = estimatorDT;

      this.sensorInformation = sensorInformation;
      this.jointMap = jointMap;
      this.footForceSensorNames = sensorInformation.getFeetForceSensorNames();
      this.wristForceSensorNames = sensorInformation.getWristForceSensorNames();

      boolean runningOnRealRobot = target == RobotTarget.REAL_ROBOT;

      // Filtering done onboard at 20Hz
      // We might want to try to set them at around 30Hz to see if that gets rid of the shakies in single support when walking.
      armJointPositionFilterFrequencyHz = Double.POSITIVE_INFINITY;
      jointOutputEncoderVelocityFilterFrequencyHz = runningOnRealRobot ? 20.0 : Double.POSITIVE_INFINITY;
      lowerBodyJointPositionFilterFrequencyHz = Double.POSITIVE_INFINITY;
      lowerBodyJointVelocityFilterFrequencyHz = runningOnRealRobot ? 25.0 : Double.POSITIVE_INFINITY;
      fingerPositionFilterFrequencyHz = runningOnRealRobot ? 2.5 : Double.POSITIVE_INFINITY;

      // Somehow it's less shaky when these are low especially when pitching the chest forward. I still don't quite get it. Sylvain
      // Update (2018-09-12 on Unit B): Tried 50Hz for IMU filters, it looks like 25Hz reduces shakies in single support while using 50Hz for joint filters.
      // Unit A seems to be shakier in general, 25Hz works better.
      orientationFilterFrequencyHz = runningOnRealRobot ? 25.0 : Double.POSITIVE_INFINITY;
      angularVelocityFilterFrequencyHz = runningOnRealRobot ? 25.0 : Double.POSITIVE_INFINITY;
      linearAccelerationFilterFrequencyHz = runningOnRealRobot ? 25.0 : Double.POSITIVE_INFINITY;

      lowerBodyJointVelocityBacklashSlopTime = 0.0;
      armJointVelocityBacklashSlopTime = 0.0;

      doElasticityCompensation = runningOnRealRobot;
      jointElasticityFilterFrequencyHz = 20.0;
      maximumDeflection = 0.10;
      defaultJointStiffness = 10000.0;
      for (RobotSide robotSide : RobotSide.values)
         jointSpecificStiffness.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL), 9500.0);

      kinematicsPelvisPositionFilterFreqInHertz = Double.POSITIVE_INFINITY;
   }

   @Override
   public void configureSensorProcessing(SensorProcessing sensorProcessing)
   {
      String[] namesOfJointsUsingOutputEncoder = jointMap.getNamesOfJointsUsingOutputEncoder();
      String[] armJointNames = createArrayWithArmJointNames();

      YoVariableRegistry registry = sensorProcessing.getYoVariableRegistry();

      if (DEBUG_VELOCITY_WITH_FD)
      {
         YoDouble dummyAlpha = new YoDouble("dummyAlpha", registry);
         sensorProcessing.computeJointVelocityFromFiniteDifference(dummyAlpha, true);
      }

      DoubleProvider orientationAlphaFilter = sensorProcessing.createAlphaFilter("orientationBreakFrequency", orientationFilterFrequencyHz);
      DoubleProvider angularVelocityAlphaFilter = sensorProcessing.createAlphaFilter("angularVelocityBreakFrequency", angularVelocityFilterFrequencyHz);
      DoubleProvider linearAccelerationAlphaFilter = sensorProcessing.createAlphaFilter("linearAccelerationBreakFrequency", linearAccelerationFilterFrequencyHz);

      // Lower body: For the joints using the output encoders: Compute velocity from the joint position using finite difference.
      DoubleProvider jointOutputEncoderVelocityAlphaFilter = sensorProcessing.createAlphaFilter("jointOutputEncoderVelocityBreakFrequency",
                                                                                                jointOutputEncoderVelocityFilterFrequencyHz);
      sensorProcessing.computeJointVelocityFromFiniteDifferenceOnlyForSpecifiedJoints(jointOutputEncoderVelocityAlphaFilter, false,
                                                                                      namesOfJointsUsingOutputEncoder);

      // Lower body: Then apply for all velocity: 1- alpha filter 2- backlash compensator 3- elasticity compensation.
      DoubleProvider lowerBodyJointVelocityAlphaFilter = sensorProcessing.createAlphaFilter("lowerBodyJointVelocityBreakFrequency",
                                                                                      lowerBodyJointVelocityFilterFrequencyHz);
      DoubleProvider lowerBodyJointVelocitySlopTime = new DoubleParameter("lowerBodyJointVelocityBacklashSlopTime", registry, lowerBodyJointVelocityBacklashSlopTime);
      sensorProcessing.addSensorAlphaFilterWithSensorsToIgnore(lowerBodyJointVelocityAlphaFilter, false, JOINT_VELOCITY, armJointNames);
      sensorProcessing.addJointVelocityBacklashFilterWithJointsToIgnore(lowerBodyJointVelocitySlopTime, false, armJointNames);

      // Lower body: Apply an alpha filter on the position to be in phase with the velocity
      DoubleProvider lowerBodyJointPositionAlphaFilter = sensorProcessing.createAlphaFilter("lowerBodyJointPositionBreakFrequency",
                                                                                            lowerBodyJointPositionFilterFrequencyHz);
      sensorProcessing.addSensorAlphaFilterWithSensorsToIgnore(lowerBodyJointPositionAlphaFilter, false, JOINT_POSITION, armJointNames);

      if (doElasticityCompensation)
      {
         DoubleProvider elasticityAlphaFilter = sensorProcessing.createAlphaFilter("jointDeflectionDotBreakFrequency", jointElasticityFilterFrequencyHz);
         DoubleProvider maxDeflection = sensorProcessing.createMaxDeflection("jointAngleMaxDeflection", maximumDeflection);
         Map<OneDoFJointBasics, DoubleProvider> jointPositionStiffness = sensorProcessing.createStiffness("stiffness", defaultJointStiffness, jointSpecificStiffness);

         Map<String, Integer> filteredTauForElasticity = sensorProcessing.addSensorAlphaFilter(elasticityAlphaFilter, true, JOINT_TAU);
         sensorProcessing.addJointPositionElasticyCompensatorWithJointsToIgnore(jointPositionStiffness, maxDeflection, filteredTauForElasticity, false,
                                                                                armJointNames);
         sensorProcessing.addJointVelocityElasticyCompensatorWithJointsToIgnore(jointPositionStiffness, maxDeflection, filteredTauForElasticity, false,
                                                                                armJointNames);
      }

      // Arm joints: Apply for all velocity: 1- backlash compensator.
      DoubleProvider armJointVelocitySlopTime = new DoubleParameter("armJointVelocityBacklashSlopTime", registry, armJointVelocityBacklashSlopTime);
      sensorProcessing.addJointVelocityBacklashFilterOnlyForSpecifiedJoints(armJointVelocitySlopTime, false, armJointNames);

      // Arm joints: Apply an alpha filter on the position to be in phase with the velocity
      DoubleProvider armJointPositionAlphaFilter = sensorProcessing.createAlphaFilter("armJointPositionBreakFrequency", armJointPositionFilterFrequencyHz);
      sensorProcessing.addSensorAlphaFilterOnlyForSpecifiedSensors(armJointPositionAlphaFilter, false, JOINT_POSITION, armJointNames);

      // Filter the finger joint position a lot as they're super noisy.
      DoubleProvider fingerPositionAlphaFilter = sensorProcessing.createAlphaFilter("fingerPositionBreakFrequency", fingerPositionFilterFrequencyHz);
      sensorProcessing.addSensorAlphaFilterOnlyForSpecifiedSensors(fingerPositionAlphaFilter, false, JOINT_POSITION, createArrayWithFingerJointNames());

      //imu
      sensorProcessing.addSensorAlphaFilter(orientationAlphaFilter, false, IMU_ORIENTATION);
      sensorProcessing.addSensorAlphaFilter(angularVelocityAlphaFilter, false, IMU_ANGULAR_VELOCITY);
      sensorProcessing.addSensorAlphaFilter(linearAccelerationAlphaFilter, false, IMU_LINEAR_ACCELERATION);
   }

   private String[] createArrayWithArmJointNames()
   {
      List<String> nameList = new ArrayList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         for (ArmJointName armJointName : jointMap.getArmJointNames())
         {
            nameList.add(jointMap.getArmJointName(robotSide, armJointName));
         }
      }

      return nameList.toArray(new String[0]);
   }

   private String[] createArrayWithFingerJointNames()
   {
      List<String> nameList = new ArrayList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         for (ValkyrieHandJointName jointName : ValkyrieHandJointName.values)
         {
            nameList.add(jointName.getJointName(robotSide));
         }
      }

      return nameList.toArray(new String[0]);
   }

   @Override
   public SensorNoiseParameters getSensorNoiseParameters()
   {
      return sensorNoiseParameters;
   }

   @Override
   public double getEstimatorDT()
   {
      return estimatorDT;
   }

   @Override
   public boolean isRunningOnRealRobot()
   {
      return target == RobotTarget.REAL_ROBOT;
   }

   @Override
   public double getKinematicsPelvisPositionFilterFreqInHertz()
   {
      return kinematicsPelvisPositionFilterFreqInHertz;
   }

   @Override
   public double getCoPFilterFreqInHertz()
   {
      return 4.0;
   }

   @Override
   public boolean enableIMUBiasCompensation()
   {
      return true;
   }

   @Override
   public boolean enableIMUYawDriftCompensation()
   {
      return true;
   }

   @Override
   public double getIMUBiasFilterFreqInHertz()
   {
      return 6.0e-3;
   }

   @Override
   public double getIMUYawDriftFilterFreqInHertz()
   {
      return 0.4; // alpha = 0.995 with dt = 2ms
   }

   @Override
   public double getIMUBiasVelocityThreshold()
   {
      return 0.015;
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
      return 11.7893; // alpha = 0.8 with dt = 0.003
   }

   @Override
   public double getPelvisLinearVelocityFusingFrequency()
   {
      //      return 0.4261; // alpha = 0.992 with dt = 0.003
      // For some reason, trusting more the accelerometer causes the state estimator to understimate the
      // pelvis linear velocity when doing reasonably quick transfer during walking.
      // This contributes in the capture point overshooting to the outside of the feet.
      return 2.0146195328088035; // alpha = 0.975 with dt = 0.002
   }

   @Override
   public double getIMUJointVelocityEstimationBacklashSlopTime()
   {
      return 0.0;
   }

   @Override
   public double getDelayTimeForTrustingFoot()
   {
      return 0.02;
   }

   @Override
   public double getForceInPercentOfWeightThresholdToTrustFoot()
   {
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
   public boolean useIMUsForSpineJointVelocityEstimation()
   {
      return target == RobotTarget.REAL_ROBOT;
   }

   @Override
   public double getBreakFrequencyForSpineJointVelocityEstimation()
   {
      return AlphaFilteredYoVariable.computeBreakFrequencyGivenAlpha(0.95, 0.002);
   }

   /**
    * IMUs to use to compute the spine joint velocities.
    * @return {@code Pair<String, String>} the first element is the name of one pelvis IMU, the second is the name of one IMU of the trunk. 
    */
   @Override
   public ImmutablePair<String, String> getIMUsForSpineJointVelocityEstimation()
   {
      return new ImmutablePair<String, String>(sensorInformation.getRearPelvisIMUSensor(), sensorInformation.getLeftTrunkIMUSensor());
   }

   @Override
   public FootSwitchFactory getFootSwitchFactory()
   {
      WrenchBasedFootSwitchFactory footSwitchFactory = new WrenchBasedFootSwitchFactory();
      footSwitchFactory.setDefaultContactThresholdForce(120.0);
      footSwitchFactory.setDefaultCoPThresholdFraction(0.02);
      footSwitchFactory.setDefaultSecondContactThresholdForceIgnoringCoP(Double.POSITIVE_INFINITY);
      return footSwitchFactory;
   }

   @Override
   public boolean requestWristForceSensorCalibrationAtStart()
   {
      return false;
   }

   @Override
   public SideDependentList<String> getWristForceSensorNames()
   {
      return wristForceSensorNames;
   }

   @Override
   public boolean requestFootForceSensorCalibrationAtStart()
   {
      return target == RobotTarget.REAL_ROBOT;
   }

   @Override
   public SideDependentList<String> getFootForceSensorNames()
   {
      return footForceSensorNames;
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

   @Override
   public boolean createFootWrenchSensorDriftEstimator()
   {
      return target == RobotTarget.REAL_ROBOT;
   }
}

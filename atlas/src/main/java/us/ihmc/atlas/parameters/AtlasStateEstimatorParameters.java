package us.ihmc.atlas.parameters;

import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.FORCE_SENSOR;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.IMU_ANGULAR_VELOCITY;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.IMU_LINEAR_ACCELERATION;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.IMU_ORIENTATION;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.JOINT_TAU;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.TORQUE_SENSOR;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class AtlasStateEstimatorParameters extends StateEstimatorParameters
{
   private final boolean runningOnRealRobot;

   private final double estimatorDT;

   private final double jointVelocitySlopTimeForBacklashCompensation;
   private static final double backXBacklashSlopTime = 0.03;
   private static final double backXAlphaFilterBreakFrequency = 16.0;

   private final double defaultFilterBreakFrequency;
   private final double defaultFilterBreakFrequencyArm;

   // private final SensorNoiseParameters sensorNoiseParameters = DRCSimulatedSensorNoiseParameters.createNoiseParametersForEstimatorJerryTuning();
   // private SensorNoiseParameters sensorNoiseParameters = DRCSimulatedSensorNoiseParameters.createNoiseParametersForEstimatorJerryTuningSeptember2013();
   private SensorNoiseParameters sensorNoiseParameters = null;

   private final boolean doElasticityCompensation;
   private final double jointElasticityFilterFrequencyHz;
   private final double defaultJointStiffness;
   private final HashMap<String, Double> jointSpecificStiffness = new HashMap<>();

   private final SideDependentList<String> footForceSensorNames;
   private final SideDependentList<String> wristForceSensorNames;

   private final DRCRobotJointMap jointMap;

   private final ImmutablePair<String, String> imusForSpineJointEstimation;

   private final boolean applyJointPositionPolynomialApproximation;

   public AtlasStateEstimatorParameters(DRCRobotJointMap jointMap, AtlasSensorInformation sensorInformation, boolean runningOnRealRobot, double estimatorDT)
   {
      this.jointMap = jointMap;
      this.runningOnRealRobot = runningOnRealRobot;
      this.estimatorDT = estimatorDT;

      imusForSpineJointEstimation = new ImmutablePair<String, String>(sensorInformation.getPrimaryBodyImu(), sensorInformation.getChestImu());

      wristForceSensorNames = sensorInformation.getWristForceSensorNames();
      footForceSensorNames = sensorInformation.getFeetForceSensorNames();

      defaultFilterBreakFrequency = runningOnRealRobot ? 16.0 : Double.POSITIVE_INFINITY;
      defaultFilterBreakFrequencyArm = runningOnRealRobot ? 40.0 : Double.POSITIVE_INFINITY;

      jointVelocitySlopTimeForBacklashCompensation = 0.03;

      applyJointPositionPolynomialApproximation = runningOnRealRobot;
      doElasticityCompensation = runningOnRealRobot;
      jointElasticityFilterFrequencyHz = 20.0;
      defaultJointStiffness = 20000.0;
      for (RobotSide robotSide : RobotSide.values)
      {
         jointSpecificStiffness.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL), 12000.0); //8000.0);
//         jointSpecificStiffness.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW), 7000.0);
      }
//      jointSpecificStiffness.put(jointMap.getSpineJointName(SpineJointName.SPINE_YAW), 8000.0);
//      jointSpecificStiffness.put(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH), 8000.0);
//      jointSpecificStiffness.put(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL), 8000.0);

   }

   @Override
   public void configureSensorProcessing(SensorProcessing sensorProcessing)
   {
      YoVariableRegistry registry = sensorProcessing.getYoVariableRegistry();

      if (applyJointPositionPolynomialApproximation)
      {
         YoPolynomial backZPolynomial = new YoPolynomial("q_poly_back_bkz", 2, registry);
         // This was obtained using LIDAR by observing the variation in the back z position error. It does not correct for a possible absolute position error.
         backZPolynomial.setDirectly(new double[]{0.00305, 1.04087});
         sensorProcessing.addJointPositionPolynomialProcessorOnlyForSpecifiedJoints(backZPolynomial, false, jointMap.getSpineJointName(SpineJointName.SPINE_YAW));
      }

      String[] armJointNames = createArmJointNames();

      String[] backXName = new String[] {jointMap.getSpineJointName(SpineJointName.SPINE_ROLL)};
      String[] armAndBackJoints = new String[armJointNames.length + backXName.length];
      System.arraycopy(armJointNames, 0, armAndBackJoints, 0, armJointNames.length);
      System.arraycopy(backXName, 0, armAndBackJoints, armJointNames.length, backXName.length);
      YoDouble backXFilter = sensorProcessing.createAlphaFilter("backXAlphaFilter", backXAlphaFilterBreakFrequency);
      YoDouble backXSlopTime = new YoDouble("backXSlopTime", registry);
      backXSlopTime.set(backXBacklashSlopTime);

      YoDouble jointVelocityAlphaFilter = sensorProcessing.createAlphaFilter("jointVelocityAlphaFilter", defaultFilterBreakFrequency);
      YoDouble wristForceAlphaFilter = sensorProcessing.createAlphaFilter("wristForceAlphaFilter", defaultFilterBreakFrequency);
      YoDouble jointVelocitySlopTime = new YoDouble("jointBacklashSlopTime", registry);
      jointVelocitySlopTime.set(jointVelocitySlopTimeForBacklashCompensation);

      YoDouble armJointVelocityAlphaFilter1 = sensorProcessing.createAlphaFilter("armJointVelocityAlphaFilter1", defaultFilterBreakFrequencyArm);
//      YoDouble armJointVelocityAlphaFilter2 = sensorProcessing.createAlphaFilter("armJointVelocityAlphaFilter2", defaultFilterBreakFrequencyArm);
      YoDouble armJointVelocitySlopTime = new YoDouble("armJointBacklashSlopTime", registry);
      armJointVelocitySlopTime.set(jointVelocitySlopTimeForBacklashCompensation);

      YoDouble orientationAlphaFilter = sensorProcessing.createAlphaFilter("orientationAlphaFilter", defaultFilterBreakFrequency);
      YoDouble angularVelocityAlphaFilter = sensorProcessing.createAlphaFilter("angularVelocityAlphaFilter", defaultFilterBreakFrequency);
      YoDouble linearAccelerationAlphaFilter = sensorProcessing.createAlphaFilter("linearAccelerationAlphaFilter", defaultFilterBreakFrequency);

      // 1- Compute joint velocities with finite difference and a low-pass filter
      sensorProcessing.computeJointVelocityFromFiniteDifferenceWithJointsToIgnore(jointVelocityAlphaFilter, false, armAndBackJoints);
      sensorProcessing.computeJointVelocityFromFiniteDifferenceOnlyForSpecifiedJoints(backXFilter, false, backXName);
      sensorProcessing.computeJointVelocityFromFiniteDifferenceOnlyForSpecifiedJoints(armJointVelocityAlphaFilter1, false, armJointNames);

      // 2- Add elasticity compensation
      if (doElasticityCompensation)
      {
         YoDouble elasticityAlphaFilter = sensorProcessing.createAlphaFilter("jointDeflectionDotAlphaFilter", jointElasticityFilterFrequencyHz);
         YoDouble maxDeflection = sensorProcessing.createMaxDeflection("jointAngleMaxDeflection", 0.1);
         Map<OneDoFJoint, YoDouble> jointPositionStiffness = sensorProcessing.createStiffnessWithJointsToIgnore("stiffness", defaultJointStiffness, jointSpecificStiffness, armJointNames);

         Map<String, Integer> filteredTauForElasticity = sensorProcessing.addSensorAlphaFilterWithSensorsToIgnore(elasticityAlphaFilter, true, JOINT_TAU, armJointNames);
         sensorProcessing.addJointPositionElasticyCompensatorWithJointsToIgnore(jointPositionStiffness, maxDeflection, filteredTauForElasticity, false, armJointNames);
         sensorProcessing.addJointVelocityElasticyCompensatorWithJointsToIgnore(jointPositionStiffness, maxDeflection, filteredTauForElasticity, false, armJointNames);
      }

      // 3- Add backlash compensator
      sensorProcessing.addJointVelocityBacklashFilterWithJointsToIgnore(jointVelocitySlopTime, false, armAndBackJoints);
      sensorProcessing.addJointVelocityBacklashFilterOnlyForSpecifiedJoints(backXSlopTime, false, backXName);
      sensorProcessing.addJointVelocityBacklashFilterOnlyForSpecifiedJoints(armJointVelocitySlopTime, false, armJointNames);

      sensorProcessing.computeJointAccelerationFromFiniteDifference(jointVelocityAlphaFilter, false);

      sensorProcessing.addSensorAlphaFilter(orientationAlphaFilter, false, IMU_ORIENTATION);
      sensorProcessing.addSensorAlphaFilter(angularVelocityAlphaFilter, false, IMU_ANGULAR_VELOCITY);
      sensorProcessing.addSensorAlphaFilter(linearAccelerationAlphaFilter, false, IMU_LINEAR_ACCELERATION);

      if (wristForceSensorNames != null)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            sensorProcessing.addSensorAlphaFilterOnlyForSpecifiedSensors(wristForceAlphaFilter, false, FORCE_SENSOR, wristForceSensorNames.get(robotSide));
            sensorProcessing.addSensorAlphaFilterOnlyForSpecifiedSensors(wristForceAlphaFilter, false, TORQUE_SENSOR, wristForceSensorNames.get(robotSide));
         }
      }
   }

   private String[] createArmJointNames()
   {
      ArrayList<String> armJointNameList = new ArrayList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         for (ArmJointName jointName : jointMap.getArmJointNames())
         {
            armJointNameList.add(jointMap.getArmJointName(robotSide, jointName));
         }
      }

      String[] armJointNames = new String[armJointNameList.size()];
      armJointNameList.toArray(armJointNames);
      return armJointNames;
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
      return runningOnRealRobot;
   }

   @Override
   public double getKinematicsPelvisPositionFilterFreqInHertz()
   {
      return Double.POSITIVE_INFINITY;
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
      return 0.04;
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
      return 0.4261; // alpha = 0.992 with dt = 0.003
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
   public double getContactThresholdForce()
   {
      return 120.0;
   }

   @Override
   public double getContactThresholdHeight()
   {
      return 0.05;
   }

   @Override
   public FootSwitchType getFootSwitchType()
   {
      return FootSwitchType.WrenchBased;
   }

   @Override
   public double getFootSwitchCoPThresholdFraction()
   {
      return 0.02;
   }

   @Override
   public SideDependentList<String> getWristForceSensorNames()
   {
      return wristForceSensorNames;
   }

   @Override
   public boolean requestWristForceSensorCalibrationAtStart()
   {
      return runningOnRealRobot;
   }

   @Override
   public SideDependentList<String> getFootForceSensorNames()
   {
      return footForceSensorNames;
   }

   @Override
   public boolean requestFootForceSensorCalibrationAtStart()
   {
      return false;
   }

   @Override
   public boolean requestFrozenModeAtStart()
   {
      return false; //runningOnRealRobot;
   }

   @Override
   public boolean getPelvisLinearStateUpdaterTrustImuWhenNoFeetAreInContact()
   {
      return false;
   }

   @Override
   public double getCenterOfMassVelocityFusingFrequency()
   {
      return 5.0;
   }

   @Override
   public boolean useGroundReactionForcesToComputeCenterOfMassVelocity()
   {
      return false;
   }

   @Override
   public boolean correctTrustedFeetPositions()
   {
      return true;
   }

   @Override
   public boolean useIMUsForSpineJointVelocityEstimation()
   {
      return true;
   }
   
   @Override
   public double getAlphaIMUsForSpineJointVelocityEstimation()
   {
      // 04/24/2017 get rid of pelvis shaking
      return 0.85;
   }

   @Override
   public double getAlphaIMUsForSpineJointPositionEstimation()
   {
      // 04/24/2017 get rid of pelvis shaking
      return runningOnRealRobot ? 0.995 : 0.0;
   }

   @Override
   public ImmutablePair<String, String> getIMUsForSpineJointVelocityEstimation()
   {
      return imusForSpineJointEstimation;
   }
}

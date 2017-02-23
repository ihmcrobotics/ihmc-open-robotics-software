package us.ihmc.atlas.parameters;

import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.FORCE_SENSOR;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.IMU_ANGULAR_VELOCITY;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.IMU_LINEAR_ACCELERATION;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.IMU_ORIENTATION;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.JOINT_VELOCITY;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.TORQUE_SENSOR;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
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
   private final double defaultJointStiffness;
   private final HashMap<String, Double> jointSpecificStiffness = new HashMap<>();

   private final SideDependentList<String> footForceSensorNames;
   private final SideDependentList<String> wristForceSensorNames;

   private final DRCRobotJointMap jointMap;

   private final ImmutablePair<String, String> imusForSpineJointEstimation;

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

      doElasticityCompensation = runningOnRealRobot;
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

      String[] armJointNames = createArmJointNames();

      String[] backXName = new String[] {jointMap.getSpineJointName(SpineJointName.SPINE_ROLL)};
      String[] armAndBackJoints = new String[armJointNames.length + backXName.length];
      System.arraycopy(armJointNames, 0, armAndBackJoints, 0, armJointNames.length);
      System.arraycopy(backXName, 0, armAndBackJoints, armJointNames.length, backXName.length);
      DoubleYoVariable backXFilter = sensorProcessing.createAlphaFilter("backXAlphaFilter", backXAlphaFilterBreakFrequency);
      DoubleYoVariable backXSlopTime = new DoubleYoVariable("backXSlopTime", registry);
      backXSlopTime.set(backXBacklashSlopTime);

      DoubleYoVariable jointVelocityAlphaFilter = sensorProcessing.createAlphaFilter("jointVelocityAlphaFilter", defaultFilterBreakFrequency);
      DoubleYoVariable wristForceAlphaFilter = sensorProcessing.createAlphaFilter("wristForceAlphaFilter", defaultFilterBreakFrequency);
      DoubleYoVariable jointVelocitySlopTime = new DoubleYoVariable("jointBacklashSlopTime", registry);
      jointVelocitySlopTime.set(jointVelocitySlopTimeForBacklashCompensation);

      DoubleYoVariable armJointVelocityAlphaFilter1 = sensorProcessing.createAlphaFilter("armJointVelocityAlphaFilter1", defaultFilterBreakFrequencyArm);
//      DoubleYoVariable armJointVelocityAlphaFilter2 = sensorProcessing.createAlphaFilter("armJointVelocityAlphaFilter2", defaultFilterBreakFrequencyArm);
      DoubleYoVariable armJointVelocitySlopTime = new DoubleYoVariable("armJointBacklashSlopTime", registry);
      armJointVelocitySlopTime.set(jointVelocitySlopTimeForBacklashCompensation);

      DoubleYoVariable orientationAlphaFilter = sensorProcessing.createAlphaFilter("orientationAlphaFilter", defaultFilterBreakFrequency);
      DoubleYoVariable angularVelocityAlphaFilter = sensorProcessing.createAlphaFilter("angularVelocityAlphaFilter", defaultFilterBreakFrequency);
      DoubleYoVariable linearAccelerationAlphaFilter = sensorProcessing.createAlphaFilter("linearAccelerationAlphaFilter", defaultFilterBreakFrequency);

      if (doElasticityCompensation)
      {
         DoubleYoVariable maxDeflection = sensorProcessing.createMaxDeflection("jointAngleMaxDeflection", 0.1);
         Map<OneDoFJoint, DoubleYoVariable> jointPositionStiffness = sensorProcessing.createStiffnessWithJointsToIgnore("stiffness", defaultJointStiffness, jointSpecificStiffness, armJointNames);
         sensorProcessing.addJointPositionElasticyCompensatorWithJointsToIgnore(jointPositionStiffness, maxDeflection, false, armJointNames);
      }

      sensorProcessing.computeJointVelocityWithBacklashCompensatorWithJointsToIgnore(jointVelocityAlphaFilter, jointVelocitySlopTime, false, armAndBackJoints);
      sensorProcessing.computeJointVelocityWithBacklashCompensatorOnlyForSpecifiedJoints(backXFilter, backXSlopTime, false, backXName);
      sensorProcessing.addSensorAlphaFilterWithSensorsToIgnore(jointVelocityAlphaFilter, false, JOINT_VELOCITY, armJointNames);

      sensorProcessing.computeJointVelocityWithBacklashCompensatorOnlyForSpecifiedJoints(armJointVelocityAlphaFilter1, armJointVelocitySlopTime, false, armJointNames);
//      sensorProcessing.addJointVelocityAlphaFilterOnlyForSpecifiedJoints(armJointVelocityAlphaFilter2, false, armJointNames);

      sensorProcessing.computeJointAccelerationFromFiniteDifference(jointVelocityAlphaFilter, false);

      sensorProcessing.addSensorAlphaFilter(orientationAlphaFilter, false, IMU_ORIENTATION);
      sensorProcessing.addSensorAlphaFilter(angularVelocityAlphaFilter, false, IMU_ANGULAR_VELOCITY);
      sensorProcessing.addSensorAlphaFilter(linearAccelerationAlphaFilter, false, IMU_LINEAR_ACCELERATION);

      for (RobotSide robotSide : RobotSide.values)
      {
         sensorProcessing.addSensorAlphaFilterOnlyForSpecifiedSensors(wristForceAlphaFilter, false, FORCE_SENSOR, wristForceSensorNames.get(robotSide));
         sensorProcessing.addSensorAlphaFilterOnlyForSpecifiedSensors(wristForceAlphaFilter, false, TORQUE_SENSOR, wristForceSensorNames.get(robotSide));
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
      return 0.95;
   }
   
   @Override
   public ImmutablePair<String, String> getIMUsForSpineJointVelocityEstimation()
   {
      return imusForSpineJointEstimation;
   }
}

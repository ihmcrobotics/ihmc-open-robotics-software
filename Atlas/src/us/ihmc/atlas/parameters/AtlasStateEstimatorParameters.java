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

import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class AtlasStateEstimatorParameters implements StateEstimatorParameters
{
   private final boolean runningOnRealRobot;

   private final double estimatorDT;

   private final double jointVelocitySlopTimeForBacklashCompensation;

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

   public AtlasStateEstimatorParameters(DRCRobotJointMap jointMap, AtlasSensorInformation sensorInformation, boolean runningOnRealRobot, double estimatorDT)
   {
      this.jointMap = jointMap;
      this.runningOnRealRobot = runningOnRealRobot;

      this.estimatorDT = estimatorDT;

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
         Map<OneDoFJoint, DoubleYoVariable> jointPositionStiffness = sensorProcessing.createStiffnessWithJointsToIgnore("stiffness", defaultJointStiffness, jointSpecificStiffness, armJointNames);
         sensorProcessing.addJointPositionElasticyCompensatorWithJointsToIgnore(jointPositionStiffness, false, armJointNames);
      }

      sensorProcessing.computeJointVelocityWithBacklashCompensatorWithJointsToIgnore(jointVelocityAlphaFilter, jointVelocitySlopTime, false, armJointNames);
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
   public double getKinematicsPelvisLinearVelocityFilterFreqInHertz()
   {
      return 16.0;
   }

   @Override
   public double getCoPFilterFreqInHertz()
   {
      return 4.0;
   }

   @Override
   public boolean useAccelerometerForEstimation()
   {
      return true;
   }

   @Override
   public boolean estimateAccelerationBias()
   {
      return false;
   }

   @Override
   public boolean cancelGravityFromAccelerationMeasurement()
   {
      return true;
   }

   @Override
   public double getAccelerationBiasFilterFreqInHertz()
   {
      return 5.3052e-4;
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
   public double getPelvisVelocityBacklashSlopTime()
   {
      return jointVelocitySlopTimeForBacklashCompensation;
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
   public boolean estimateIMUDrift()
   {
      return true;
   }

   @Override
   public boolean compensateIMUDrift()
   {
      return true;
   }

   @Override
   public double getIMUDriftFilterFreqInHertz()
   {
      return 0.5332;
   }

   @Override
   public double getFootVelocityUsedForImuDriftFilterFreqInHertz()
   {
      return 0.5332;
   }

   @Override
   public double getFootVelocityThresholdToEnableIMUDriftCompensation()
   {
      return 0.03;
   }

   @Override
   public boolean trustCoPAsNonSlippingContactPoint()
   {
      return true;
   }

   @Override
   public boolean useControllerDesiredCenterOfPressure()
   {
      return false;
   }

   @Override
   public boolean useTwistForPelvisLinearStateEstimation()
   {
      return true;
   }

   @Override
   public double getPelvisLinearVelocityAlphaNewTwist()
   {
      return 0.15;
   }

   @Override
   public boolean createFusedIMUSensor()
   {
      return false;
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
   public boolean useIMUsForSpineJointVelocityEstimation()
   {
      // TODO For Valkyrie. Probably have to make more generic.
      return false;
   }

   @Override
   public double getAlphaIMUsForSpineJointVelocityEstimation()
   {
      // TODO For Valkyrie. Probably have to make more generic.
      return 0;
   }

   @Override
   public ImmutablePair<String, String> getIMUsForSpineJointVelocityEstimation()
   {
      // TODO For Valkyrie. Probably have to make more generic.
      return null;
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
}

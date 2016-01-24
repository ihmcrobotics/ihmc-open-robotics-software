package us.ihmc.valkyrie.parameters;

import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.IMU_ANGULAR_VELOCITY;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.IMU_LINEAR_ACCELERATION;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.IMU_ORIENTATION;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.JOINT_TAU;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.JOINT_VELOCITY;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
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

public class ValkyrieStateEstimatorParameters implements StateEstimatorParameters
{
   private final boolean runningOnRealRobot;

   private final double estimatorDT;

   private final double kinematicsPelvisLinearVelocityFilterFreqInHertz;
   private final double kinematicsPelvisPositionFilterFreqInHertz;

   private final double lowerBodyJointVelocityBacklashSlopTime;
   private final double armJointVelocityBacklashSlopTime;

   private final double lowerBodyJointVelocityFilterFrequencyHz;
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

   public ValkyrieStateEstimatorParameters(boolean runningOnRealRobot, double estimatorDT, ValkyrieSensorInformation sensorInformation, ValkyrieJointMap jointMap)
   {
      this.runningOnRealRobot = runningOnRealRobot;

      this.estimatorDT = estimatorDT;
      
      this.sensorInformation = sensorInformation;
      this.jointMap = jointMap;
      this.footForceSensorNames = sensorInformation.getFeetForceSensorNames();
      this.wristForceSensorNames = sensorInformation.getWristForceSensorNames();

      lowerBodyJointVelocityFilterFrequencyHz = runningOnRealRobot ? 20.0 : Double.POSITIVE_INFINITY;

      orientationFilterFrequencyHz        = 20.0; //50.0;
      angularVelocityFilterFrequencyHz    = 20.0; //50.0;
      linearAccelerationFilterFrequencyHz = 20.0; //runningOnRealRobot ? Double.POSITIVE_INFINITY : Double.POSITIVE_INFINITY;

      lowerBodyJointVelocityBacklashSlopTime = 0.03;
      armJointVelocityBacklashSlopTime = 0.06;

      doElasticityCompensation = runningOnRealRobot;
      jointElasticityFilterFrequencyHz = 20.0;
      maximumDeflection = 0.25;
      defaultJointStiffness = 10000;
      for (RobotSide robotSide : RobotSide.values)
         jointSpecificStiffness.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL), 6500.0);

      kinematicsPelvisPositionFilterFreqInHertz = Double.POSITIVE_INFINITY;
      kinematicsPelvisLinearVelocityFilterFreqInHertz = 50.0;
   }

   @Override
   public void configureSensorProcessing(SensorProcessing sensorProcessing)
   {
      String[] namesOfJointsUsingOutputEncoder = jointMap.getNamesOfJointsUsingOutputEncoder();
      String[] armJointNames = createArrayWithArmJointNames();
      
      YoVariableRegistry registry = sensorProcessing.getYoVariableRegistry();


      DoubleYoVariable orientationAlphaFilter = sensorProcessing.createAlphaFilter("orientationAlphaFilter", orientationFilterFrequencyHz);
      DoubleYoVariable angularVelocityAlphaFilter = sensorProcessing.createAlphaFilter("angularVelocityAlphaFilter", angularVelocityFilterFrequencyHz);
      DoubleYoVariable linearAccelerationAlphaFilter = sensorProcessing.createAlphaFilter("linearAccelerationAlphaFilter", linearAccelerationFilterFrequencyHz);

      // Lower body: For the joints using the output encoders: Compute velocity from the joint position using finite difference.
      DoubleYoVariable jointOutputEncoderVelocityAlphaFilter = sensorProcessing.createAlphaFilter("jointOutputEncoderVelocityAlphaFilter", lowerBodyJointVelocityFilterFrequencyHz);
      sensorProcessing.computeJointVelocityFromFiniteDifferenceOnlyForSpecifiedJoints(jointOutputEncoderVelocityAlphaFilter, false, namesOfJointsUsingOutputEncoder);

      // Lower body: Then apply for all: 1- alpha filter 2- backlash compensator 3- elasticity compensation.
      DoubleYoVariable lowerBodyJointVelocityAlphaFilter = sensorProcessing.createAlphaFilter("lowerBodyJointVelocityAlphaFilter", lowerBodyJointVelocityFilterFrequencyHz);
      DoubleYoVariable lowerBodyJointVelocitySlopTime = new DoubleYoVariable("lowerBodyJointVelocityBacklashSlopTime", registry);
      lowerBodyJointVelocitySlopTime.set(lowerBodyJointVelocityBacklashSlopTime);
      sensorProcessing.addSensorAlphaFilterWithSensorsToIgnore(lowerBodyJointVelocityAlphaFilter, false, JOINT_VELOCITY, armJointNames);
      sensorProcessing.addJointVelocityBacklashFilterWithJointsToIgnore(lowerBodyJointVelocitySlopTime, false, armJointNames);
      
      if (doElasticityCompensation)
      {
         DoubleYoVariable elasticityAlphaFilter = sensorProcessing.createAlphaFilter("jointDeflectionDotAlphaFilter", jointElasticityFilterFrequencyHz);
         DoubleYoVariable maxDeflection = sensorProcessing.createMaxDeflection("jointAngleMaxDeflection", maximumDeflection);
         Map<OneDoFJoint, DoubleYoVariable> jointPositionStiffness = sensorProcessing.createStiffness("stiffness", defaultJointStiffness, jointSpecificStiffness);

         Map<String, Integer> filteredTauForElasticity = sensorProcessing.addSensorAlphaFilter(elasticityAlphaFilter, true, JOINT_TAU);
         sensorProcessing.addJointPositionElasticyCompensatorWithJointsToIgnore(jointPositionStiffness, maxDeflection, filteredTauForElasticity, false, armJointNames);
         sensorProcessing.addJointVelocityElasticyCompensatorWithJointsToIgnore(jointPositionStiffness, maxDeflection, filteredTauForElasticity, false, armJointNames);
      }

      // Arm joints: Apply for all: 1- backlash compensator.
      DoubleYoVariable armJointVelocitySlopTime = new DoubleYoVariable("armJointVelocityBacklashSlopTime", registry);
      armJointVelocitySlopTime.set(armJointVelocityBacklashSlopTime);
      sensorProcessing.addJointVelocityBacklashFilterOnlyForSpecifiedJoints(armJointVelocitySlopTime, false, armJointNames);
      
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
      return kinematicsPelvisPositionFilterFreqInHertz;
   }

   @Override
   public double getKinematicsPelvisLinearVelocityFilterFreqInHertz()
   {
      return kinematicsPelvisLinearVelocityFilterFreqInHertz;
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
      return lowerBodyJointVelocityBacklashSlopTime;
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
   public double getFootSwitchCoPThresholdFraction()
   {
      return 0.02;
   }

   @Override
   public boolean useIMUsForSpineJointVelocityEstimation()
   {
      return runningOnRealRobot;
   }

   @Override
   public double getAlphaIMUsForSpineJointVelocityEstimation()
   {
      return 0.8; // 35 Hz
   }

   /**
    * IMUs to use to compute the spine joint velocities.
    * @return {@code Pair<String, String>} the first element is the name of one pelvis IMU, the second is the name of one IMU of the trunk. 
    */
   @Override
   public ImmutablePair<String, String> getIMUsForSpineJointVelocityEstimation()
   {
      return new ImmutablePair<String, String>(sensorInformation.getMiddlePelvisIMUSensor(), sensorInformation.getLeftTrunkIMUSensor());
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
//      return runningOnRealRobot ? FootSwitchType.WrenchAndContactSensorFused : FootSwitchType.WrenchBased;
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
      return runningOnRealRobot;
   }

   @Override
   public SideDependentList<String> getFootForceSensorNames()
   {
      return footForceSensorNames;
   }
}

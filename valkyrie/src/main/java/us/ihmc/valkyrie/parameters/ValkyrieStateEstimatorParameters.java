package us.ihmc.valkyrie.parameters;

import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.IMU_ANGULAR_VELOCITY;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.IMU_LINEAR_ACCELERATION;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.IMU_ORIENTATION;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.JOINT_POSITION;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.JOINT_TAU;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.JOINT_VELOCITY;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.IndexFingerPitch1;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.IndexFingerPitch2;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.IndexFingerPitch3;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.MiddleFingerPitch1;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.MiddleFingerPitch2;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.MiddleFingerPitch3;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.PinkyPitch1;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.PinkyPitch2;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.PinkyPitch3;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.ThumbPitch1;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.ThumbPitch2;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.ThumbPitch3;
import static us.ihmc.valkyrie.fingers.ValkyrieHandJointName.ThumbRoll;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.yaml.snakeyaml.Yaml;

import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.valkyrie.fingers.ValkyrieHandJointName;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ValkyrieStateEstimatorParameters extends StateEstimatorParameters
{
   private static final boolean DEBUG_VELOCITY_WITH_FD = false;
   private static final String FINGER_TRANSMISSION_FILE = System.getProperty("user.home") + File.separator + "valkyrie/ValkyrieFingerJointTransmissionCoeffs.yaml";

   private final boolean runningOnRealRobot;

   private final double estimatorDT;

   private final double kinematicsPelvisPositionFilterFreqInHertz;

   private final double lowerBodyJointVelocityBacklashSlopTime;
   private final double armJointVelocityBacklashSlopTime;

   private final double armJointPositionFilterFrequencyHz;
   private final double lowerBodyJointPositionFilterFrequencyHz;
   private final double jointOutputEncoderVelocityFilterFrequencyHz;
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

   /*
    * Challenge with Valkyrie is that we need minimum delay in all signal, which
    * is usual but it does matter way more for Valkyrie as there is almost no
    * natural damping. In the same spirit, all the signals need to be
    * consistent, for instance joint velocity should be in phase with joint
    * position.
    */
   public ValkyrieStateEstimatorParameters(boolean runningOnRealRobot, double estimatorDT, ValkyrieSensorInformation sensorInformation,
                                           ValkyrieJointMap jointMap)
   {
      this.runningOnRealRobot = runningOnRealRobot;

      this.estimatorDT = estimatorDT;

      this.sensorInformation = sensorInformation;
      this.jointMap = jointMap;
      this.footForceSensorNames = sensorInformation.getFeetForceSensorNames();
      this.wristForceSensorNames = sensorInformation.getWristForceSensorNames();

      // Filtering done onboard at 20Hz
      // We might want to try to set them at around 30Hz to see if that gets rid of the shakies in single support when walking.
      armJointPositionFilterFrequencyHz = Double.POSITIVE_INFINITY;
      jointOutputEncoderVelocityFilterFrequencyHz = runningOnRealRobot ? 20.0 : Double.POSITIVE_INFINITY;
      lowerBodyJointPositionFilterFrequencyHz = Double.POSITIVE_INFINITY;
      lowerBodyJointVelocityFilterFrequencyHz = runningOnRealRobot ? 25.0 : Double.POSITIVE_INFINITY;

      // Somehow it's less shaky when these are low especially when pitching the chest forward.
      // I still don't quite get it. Sylvain
      orientationFilterFrequencyHz = runningOnRealRobot ? 25.0 : Double.POSITIVE_INFINITY;
      angularVelocityFilterFrequencyHz = runningOnRealRobot ? 25.0 : Double.POSITIVE_INFINITY;
      linearAccelerationFilterFrequencyHz = runningOnRealRobot ? 25.0 : Double.POSITIVE_INFINITY;

      lowerBodyJointVelocityBacklashSlopTime = 0.03;
      armJointVelocityBacklashSlopTime = 0.03;

      doElasticityCompensation = runningOnRealRobot;
      jointElasticityFilterFrequencyHz = 20.0;
      maximumDeflection = 0.10;
      defaultJointStiffness = 10000.0;
      for (RobotSide robotSide : RobotSide.values)
         jointSpecificStiffness.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL), 8000.0);

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

      YoDouble orientationAlphaFilter = sensorProcessing.createAlphaFilter("orientationAlphaFilter", orientationFilterFrequencyHz);
      YoDouble angularVelocityAlphaFilter = sensorProcessing.createAlphaFilter("angularVelocityAlphaFilter", angularVelocityFilterFrequencyHz);
      YoDouble linearAccelerationAlphaFilter = sensorProcessing.createAlphaFilter("linearAccelerationAlphaFilter", linearAccelerationFilterFrequencyHz);

      // Lower body: For the joints using the output encoders: Compute velocity from the joint position using finite difference.
      YoDouble jointOutputEncoderVelocityAlphaFilter = sensorProcessing.createAlphaFilter("jointOutputEncoderVelocityAlphaFilter",
                                                                                          jointOutputEncoderVelocityFilterFrequencyHz);
      sensorProcessing.computeJointVelocityFromFiniteDifferenceOnlyForSpecifiedJoints(jointOutputEncoderVelocityAlphaFilter, false,
                                                                                      namesOfJointsUsingOutputEncoder);

      // Lower body: Then apply for all velocity: 1- alpha filter 2- backlash compensator 3- elasticity compensation.
      YoDouble lowerBodyJointVelocityAlphaFilter = sensorProcessing.createAlphaFilter("lowerBodyJointVelocityAlphaFilter",
                                                                                      lowerBodyJointVelocityFilterFrequencyHz);
      YoDouble lowerBodyJointVelocitySlopTime = new YoDouble("lowerBodyJointVelocityBacklashSlopTime", registry);
      lowerBodyJointVelocitySlopTime.set(lowerBodyJointVelocityBacklashSlopTime);
      sensorProcessing.addSensorAlphaFilterWithSensorsToIgnore(lowerBodyJointVelocityAlphaFilter, false, JOINT_VELOCITY, armJointNames);
      sensorProcessing.addJointVelocityBacklashFilterWithJointsToIgnore(lowerBodyJointVelocitySlopTime, false, armJointNames);

      // Lower body: Apply an alpha filter on the position to be in phase with the velocity
      YoDouble lowerBodyJointPositionAlphaFilter = sensorProcessing.createAlphaFilter("lowerBodyJointPositionAlphaFilter",
                                                                                      lowerBodyJointPositionFilterFrequencyHz);
      sensorProcessing.addSensorAlphaFilterWithSensorsToIgnore(lowerBodyJointPositionAlphaFilter, false, JOINT_POSITION, armJointNames);

      if (doElasticityCompensation)
      {
         YoDouble elasticityAlphaFilter = sensorProcessing.createAlphaFilter("jointDeflectionDotAlphaFilter", jointElasticityFilterFrequencyHz);
         YoDouble maxDeflection = sensorProcessing.createMaxDeflection("jointAngleMaxDeflection", maximumDeflection);
         Map<OneDoFJoint, YoDouble> jointPositionStiffness = sensorProcessing.createStiffness("stiffness", defaultJointStiffness, jointSpecificStiffness);

         Map<String, Integer> filteredTauForElasticity = sensorProcessing.addSensorAlphaFilter(elasticityAlphaFilter, true, JOINT_TAU);
         sensorProcessing.addJointPositionElasticyCompensatorWithJointsToIgnore(jointPositionStiffness, maxDeflection, filteredTauForElasticity, false,
                                                                                armJointNames);
         sensorProcessing.addJointVelocityElasticyCompensatorWithJointsToIgnore(jointPositionStiffness, maxDeflection, filteredTauForElasticity, false,
                                                                                armJointNames);
      }

      // Arm joints: Apply for all velocity: 1- backlash compensator.
      YoDouble armJointVelocitySlopTime = new YoDouble("armJointVelocityBacklashSlopTime", registry);
      armJointVelocitySlopTime.set(armJointVelocityBacklashSlopTime);
      sensorProcessing.addJointVelocityBacklashFilterOnlyForSpecifiedJoints(armJointVelocitySlopTime, false, armJointNames);

      // Arm joints: Apply an alpha filter on the position to be in phase with the velocity
      YoDouble armJointPositionAlphaFilter = sensorProcessing.createAlphaFilter("armJointPositionAlphaFilter", armJointPositionFilterFrequencyHz);
      sensorProcessing.addSensorAlphaFilterOnlyForSpecifiedSensors(armJointPositionAlphaFilter, false, JOINT_POSITION, armJointNames);

      //imu
      sensorProcessing.addSensorAlphaFilter(orientationAlphaFilter, false, IMU_ORIENTATION);
      sensorProcessing.addSensorAlphaFilter(angularVelocityAlphaFilter, false, IMU_ANGULAR_VELOCITY);
      sensorProcessing.addSensorAlphaFilter(linearAccelerationAlphaFilter, false, IMU_LINEAR_ACCELERATION);

      if (runningOnRealRobot)
         configureFingerProcessing(sensorProcessing);
   }

   private void configureFingerProcessing(SensorProcessing sensorProcessing)
   {
      YoVariableRegistry registry = sensorProcessing.getYoVariableRegistry();
      
      SideDependentList<EnumMap<ValkyrieHandJointName, YoDouble>> sideDependentScales = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);
      SideDependentList<EnumMap<ValkyrieHandJointName, YoDouble>> sideDependentBiases = SideDependentList.createListOfEnumMaps(ValkyrieHandJointName.class);
      
      for (RobotSide robotSide : RobotSide.values)
      {
         EnumMap<ValkyrieHandJointName, YoDouble> scales = sideDependentScales.get(robotSide);
         EnumMap<ValkyrieHandJointName, YoDouble> biases = sideDependentBiases.get(robotSide);
         
         for (ValkyrieHandJointName fingerJoint : ValkyrieHandJointName.values)
         {
            YoDouble scale = new YoDouble("scale" + fingerJoint.getPascalCaseJointName(robotSide), registry);
            YoDouble bias = new YoDouble("bias" + fingerJoint.getPascalCaseJointName(robotSide), registry);
            scales.put(fingerJoint, scale);
            biases.put(fingerJoint, bias);
         }
      }

      Yaml yaml = new Yaml();
      File coeffFile = new File(FINGER_TRANSMISSION_FILE);

      boolean areCoeffsLoaded = false;

      if (coeffFile.exists())
      {
         try
         {
            FileInputStream fileInputStream = new FileInputStream(coeffFile);
            @SuppressWarnings("unchecked")
            Map<String, Map<String, Double>> coeffs = (Map<String, Map<String, Double>>) yaml.load(fileInputStream);

            for (RobotSide robotSide : RobotSide.values)
            {
               EnumMap<ValkyrieHandJointName, YoDouble> scales = sideDependentScales.get(robotSide);
               EnumMap<ValkyrieHandJointName, YoDouble> biases = sideDependentBiases.get(robotSide);

               for (ValkyrieHandJointName jointEnum : ValkyrieHandJointName.values)
               {
                  Map<String, Double> jointCoeffs = coeffs.get(jointEnum.getJointName(robotSide));

                  scales.get(jointEnum).set(jointCoeffs.getOrDefault("scale", 0.0));
                  biases.get(jointEnum).set(jointCoeffs.getOrDefault("bias", 0.0));
               }
            }
            areCoeffsLoaded = true;
         }
         catch (FileNotFoundException | NullPointerException e)
         {
            e.printStackTrace();
            System.err.println("Setting to default coeffs.");
         }
      }
      else
      {
         PrintTools.info(this, "Did not find: \"" + FINGER_TRANSMISSION_FILE + "\", setting coeffs to default.");
      }

      if (!areCoeffsLoaded)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            EnumMap<ValkyrieHandJointName, YoDouble> scales = sideDependentScales.get(robotSide);
            EnumMap<ValkyrieHandJointName, YoDouble> biases = sideDependentBiases.get(robotSide);
            
            boolean isLeftSide = robotSide == RobotSide.LEFT;
            
            scales.get(ThumbRoll         ).set(isLeftSide ? -1.00 : 1.00);
            scales.get(ThumbPitch1       ).set(isLeftSide ? -1.00 : 1.00);
            scales.get(IndexFingerPitch1 ).set(isLeftSide ? -0.75 : 0.60);
            scales.get(MiddleFingerPitch1).set(isLeftSide ? -0.75 : 0.75);
            scales.get(PinkyPitch1       ).set(isLeftSide ? -0.70 : 0.60);
            
            scales.get(ThumbPitch2       ).set(isLeftSide ?-0.50 : 0.40);
            scales.get(IndexFingerPitch2 ).set(isLeftSide ? 0.62 : 0.70);
            scales.get(MiddleFingerPitch2).set(isLeftSide ? 0.42 : 0.60);
            scales.get(PinkyPitch2       ).set(isLeftSide ? 0.50 : 0.65);
            
            scales.get(ThumbPitch3       ).set(isLeftSide ? 0.30 : 0.30);
            scales.get(IndexFingerPitch3 ).set(isLeftSide ? 0.30 : 0.40);
            scales.get(MiddleFingerPitch3).set(isLeftSide ? 0.30 : 0.30);
            scales.get(PinkyPitch3       ).set(isLeftSide ? 0.30 : 0.30);
            
            biases.get(ThumbRoll         ).set(1.57); // TODO at the same I added these, the thumb roll did not work.
            biases.get(ThumbPitch1       ).set(isLeftSide ?-0.20 :-1.10);
            biases.get(IndexFingerPitch1 ).set(isLeftSide ? 0.82 :-0.60);
            biases.get(MiddleFingerPitch1).set(isLeftSide ? 1.30 : 0.00);
            biases.get(PinkyPitch1       ).set(isLeftSide ? 1.40 :-0.50);
            
            biases.get(ThumbPitch2       ).set(isLeftSide ? 0.00 : 0.40);
            biases.get(IndexFingerPitch2 ).set(isLeftSide ?-0.60 : 0.40);
            biases.get(MiddleFingerPitch2).set(isLeftSide ?-0.80 :-0.20);
            biases.get(PinkyPitch2       ).set(isLeftSide ?-0.63 : 0.30);
            
            biases.get(ThumbPitch3       ).set(isLeftSide ? 0.00 : 0.00);
            biases.get(IndexFingerPitch3 ).set(isLeftSide ?-0.40 : 0.20);
            biases.get(MiddleFingerPitch3).set(isLeftSide ?-0.40 : 0.30);
            biases.get(PinkyPitch3       ).set(isLeftSide ?-0.40 : 0.10);
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         { // Doing the thumb separately
            ValkyrieHandJointName[] thumbBaseJoints = {ThumbRoll, ThumbPitch1, ThumbPitch2};
            for (ValkyrieHandJointName joint : thumbBaseJoints)
            {
               YoDouble scale = sideDependentScales.get(robotSide).get(joint);
               YoDouble bias = sideDependentBiases.get(robotSide).get(joint);
               String jointName = joint.getCamelCaseJointName(robotSide);
               sensorProcessing.addJointPositionAffineTransformOnlyForSpecifiedJoints(scale, bias, false, jointName);
            }
            {
               ValkyrieHandJointName slaveJoint = ThumbPitch3;
               YoDouble scale = sideDependentScales.get(robotSide).get(slaveJoint);
               YoDouble bias = sideDependentBiases.get(robotSide).get(slaveJoint);
               String masterJointName = ThumbPitch2.getCamelCaseJointName(robotSide);
               String slaveJointName = slaveJoint.getCamelCaseJointName(robotSide);
               sensorProcessing.computeJointPositionUsingCoupling(masterJointName, slaveJointName, scale, bias, false);
            }
         }

         ValkyrieHandJointName[] masterJoints = {IndexFingerPitch1, MiddleFingerPitch1, PinkyPitch1};
         ValkyrieHandJointName[] slaveJoints2 = {IndexFingerPitch2, MiddleFingerPitch2, PinkyPitch2};
         ValkyrieHandJointName[] slaveJoints3 = {IndexFingerPitch3, MiddleFingerPitch3, PinkyPitch3};

         for (ValkyrieHandJointName joint : masterJoints)
         {
            YoDouble scale = sideDependentScales.get(robotSide).get(joint);
            YoDouble bias = sideDependentBiases.get(robotSide).get(joint);
            String jointName = joint.getCamelCaseJointName(robotSide);
            sensorProcessing.addJointPositionAffineTransformOnlyForSpecifiedJoints(scale, bias, false, jointName);
         }

         for (int i = 0; i < 3; i++)
         {
            ValkyrieHandJointName masterJoint = masterJoints[i];

            {
               ValkyrieHandJointName slaveJoint = slaveJoints2[i];
               YoDouble scale = sideDependentScales.get(robotSide).get(slaveJoint);
               YoDouble bias = sideDependentBiases.get(robotSide).get(slaveJoint);
               String masterJointName = masterJoint.getCamelCaseJointName(robotSide);
               String slaveJointName = slaveJoint.getCamelCaseJointName(robotSide);
               sensorProcessing.computeJointPositionUsingCoupling(masterJointName, slaveJointName, scale, bias, false);
            }

            {
               ValkyrieHandJointName slaveJoint = slaveJoints3[i];
               YoDouble scale = sideDependentScales.get(robotSide).get(slaveJoint);
               YoDouble bias = sideDependentBiases.get(robotSide).get(slaveJoint);
               String masterJointName = masterJoint.getCamelCaseJointName(robotSide);
               String slaveJointName = slaveJoint.getCamelCaseJointName(robotSide);
               sensorProcessing.computeJointPositionUsingCoupling(masterJointName, slaveJointName, scale, bias, false);
            }
         }
      }
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
      return 0.95; // 35 Hz
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

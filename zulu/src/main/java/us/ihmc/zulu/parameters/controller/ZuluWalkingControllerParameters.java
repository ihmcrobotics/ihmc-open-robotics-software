package us.ihmc.zulu.parameters.controller;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.zulu.ZuluJointMap;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.parameters.model.ZuluPhysicalProperties;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentParameters;
import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.configurations.NaturalPostureParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeSlippingDetectorParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OneDoFJointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitchFactory;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.*;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ZuluWalkingControllerParameters extends WalkingControllerParameters
{
   private final ZuluVersion version;
   private final RobotTarget target;
   protected final ZuluJointMap jointMap;
   private final ZuluPhysicalProperties physicalProperties;
   private final ToeOffParameters toeOffParameters;
   private SwingTrajectoryParameters swingTrajectoryParameters;
   private final ZuluSteppingParameters steppingParameters;
   private final ZuluICPControllerParameters icpControllerParameters;
   private final ZuluStepAdjustmentParameters stepAdjustmentParameters;
   private JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters = new ZuluJointPrivilegedConfigurationParameters();
   private final OneDoFJointPrivilegedConfigurationParameters kneePrivilegedConfigurationParameters;
   private final JointLimitParameters kneeJointLimitParameters;

   protected TObjectDoubleHashMap<String> jointHomeConfiguration = null;
   private Map<String, Pose3D> bodyHomeConfiguration = null;

   private final ZuluMomentumOptimizationSettings momentumOptimizationSettings;

   private final double minimumHeightAboveGround;
   private double nominalHeightAboveGround;
   private final double maximumHeightAboveGround;

   /**
    * When {@code true}, the chest and hand will be reset to a default pose right when starting to walk, cancelling any ongoing trajectory and not allowing to
    * hold a pose in world frame for instance.
    */
   private boolean doPrepareManipulationForLocomotion = true;
   /**
    * When {@code true}, the pelvis will be reset to a default pose right when starting to walk, cancelling any ongoing trajectory and not allowing to hold a
    * pose in world frame for instance.
    */
   private boolean doPreparePelvisForLocomotion = true;
   /**
    * When {@code false}, messages for the upper-body will be ignored while the robot is walking.
    */
   private boolean allowUpperBodyMotionDuringLocomotion = true;

   /* Setting to true makes the arms stiffer but more responsive for streaming */
   public static final boolean RESPONSIVE_STREAMING_MODE = false;

   public ZuluWalkingControllerParameters(ZuluVersion version, RobotTarget target, ZuluJointMap jointMap, ZuluPhysicalProperties physicalProperties)
   {
      this(version, target, jointMap, physicalProperties, new ZuluContactPointParameters(jointMap, physicalProperties, true));
   }

   public ZuluWalkingControllerParameters(ZuluVersion version,
                                          RobotTarget target,
                                          ZuluJointMap jointMap,
                                          ZuluPhysicalProperties physicalProperties,
                                          RobotContactPointParameters<RobotSide> contactPointParameters)
   {
      this.version = version;
      this.target = target;
      this.jointMap = jointMap;
      this.physicalProperties = physicalProperties;

      toeOffParameters = new ZuluToeOffParameters(physicalProperties);
      momentumOptimizationSettings = new ZuluMomentumOptimizationSettings(target, jointMap, 2 + contactPointParameters.getAdditionalContactNames().size());
      swingTrajectoryParameters = new ZuluSwingTrajectoryParameters();
      steppingParameters = new ZuluSteppingParameters(physicalProperties);
      icpControllerParameters = new ZuluICPControllerParameters();
      stepAdjustmentParameters = new ZuluStepAdjustmentParameters();

      kneePrivilegedConfigurationParameters = new OneDoFJointPrivilegedConfigurationParameters();
      kneePrivilegedConfigurationParameters.setConfigurationGain(target == RobotTarget.REAL_ROBOT ? 60.0 : 120.0);
      kneePrivilegedConfigurationParameters.setVelocityGain(6.0);
      kneePrivilegedConfigurationParameters.setWeight(target == RobotTarget.REAL_ROBOT ? 15.0 : 5.0);
      kneePrivilegedConfigurationParameters.setMaxAcceleration(Double.POSITIVE_INFINITY);
      kneePrivilegedConfigurationParameters.setPrivilegedConfigurationOption(PrivilegedConfigurationCommand.PrivilegedConfigurationOption.AT_MID_RANGE);

      kneeJointLimitParameters = new JointLimitParameters();
      kneeJointLimitParameters.setMaxAbsJointVelocity(4.0);
      kneeJointLimitParameters.setJointLimitDistanceForMaxVelocity(Math.toRadians(30.0));
      kneeJointLimitParameters.setJointLimitFilterBreakFrequency(15.0);
      kneeJointLimitParameters.setVelocityControlGain(600.0);
      kneeJointLimitParameters.setVelocityDeadbandSize(0.3);

      // TODO Needs tune up
      minimumHeightAboveGround = 0.6 * jointMap.getModelScale();
      nominalHeightAboveGround = 0.88 * jointMap.getModelScale();
      maximumHeightAboveGround = 0.91 * jointMap.getModelScale();
   }

   @Override
   public SmoothFootUnloadMethod enforceSmoothFootUnloading()
   {
      return SmoothFootUnloadMethod.RHO_WEIGHT;
   }

   @Override
   public boolean minimizeAngularMomentumRateZDuringSwing()
   {
      return true;
   }

   @Override
   public double getMinimumTransferTime()
   {
      return 0.05;
   }

   @Override
   public double getOmega0()
   {
      return 3.0;
   }

   @Override
   public boolean controlToeDuringSwing()
   {
      return true;
   }

   @Override
   public boolean enableToeOffSlippingDetection()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public ToeSlippingDetectorParameters getToeSlippingDetectorParameters()
   {
      return new ToeSlippingDetectorParameters();
   }

   @Override
   public boolean allowDisturbanceRecoveryBySpeedingUpSwing()
   {
      return true;
   }

   @Override
   public boolean allowAutomaticManipulationAbort()
   {
      return true;
   }

   @Override
   public double getICPErrorThresholdToSpeedUpSwing()
   {
      return 0.05 * jointMap.getModelScale();
   }

   @Override
   public double getMinimumSwingTimeForDisturbanceRecovery()
   {
      return target == RobotTarget.REAL_ROBOT ? 0.45 : 0.35;
   }

   @Override
   public boolean resubmitStepsInSwingEveryTick()
   {
      return true;
   }

   @Override
   public boolean resubmitStepsInTransferEveryTick()
   {
      return true;
   }

   @Override
   public double minimumHeightAboveAnkle()
   {
      return minimumHeightAboveGround;
   }

   @Override
   public double nominalHeightAboveAnkle()
   {
      return nominalHeightAboveGround;
   }

   @Override
   public double maximumHeightAboveAnkle()
   {
      return maximumHeightAboveGround;
   }

   @Override
   public double getMaxLegLengthReductionSteppingDown()
   {
      return 0.05;
   }

   @Override
   public double getMaximumLegLengthForSingularityAvoidance()
   {
      return 0.78;
   }

   @Override
   public PDGains getCoMHeightControlGains()
   {
      // TODO Needs tune up
      PDGains gains = new PDGains();
      double kp = 50.0; // 40.0;
      double zeta = 0.8;
      double maxAcceleration = 0.5 * 9.81;
      double maxJerk = maxAcceleration / 0.05;

      gains.setKp(kp);
      gains.setZeta(zeta);
      gains.setMaximumFeedback(maxAcceleration);
      gains.setMaximumFeedbackRate(maxJerk);

      return gains;
   }

   @Override
   public List<GroupParameter<PIDGainsReadOnly>> getJointSpaceControlGains()
   {
      List<String> spineNames = jointMap.getSpineJointNamesAsStrings();
      List<String> neckNames = jointMap.getNeckJointNamesAsStrings();
      List<String> legNames = jointMap.getLegJointNamesAsStrings();

      List<String> cycloidArmJointNames = new ArrayList<>();

      for (RobotSide side : RobotSide.values)
      {
         cycloidArmJointNames.add(jointMap.getArmJointName(side, ArmJointName.SHOULDER_PITCH));
         cycloidArmJointNames.add(jointMap.getArmJointName(side, ArmJointName.SHOULDER_ROLL));
         cycloidArmJointNames.add(jointMap.getArmJointName(side, ArmJointName.SHOULDER_YAW));
         cycloidArmJointNames.add(jointMap.getArmJointName(side, ArmJointName.ELBOW_PITCH));

         if (version.hasCycloidForearm())
         {
            cycloidArmJointNames.add(jointMap.getArmJointName(side, ArmJointName.ELBOW_YAW));
            cycloidArmJointNames.add(jointMap.getArmJointName(side, ArmJointName.WRIST_ROLL));
            cycloidArmJointNames.add(jointMap.getArmJointName(side, ArmJointName.WRIST_YAW));
         }
      }

      PIDGains spineGains = createSpineControlGains();
      PIDGains neckGains = createNeckControlGains();
      PIDGains cycloidUpperArmGains = createCycloidArmGains();
      PIDGains legGains = createLegControlGains();

      List<GroupParameter<PIDGainsReadOnly>> jointspaceGains = new ArrayList<>();
      jointspaceGains.add(new GroupParameter<>("_SpineJointGains", spineGains, spineNames));
      jointspaceGains.add(new GroupParameter<>("_NeckJointGains", neckGains, neckNames));
      jointspaceGains.add(new GroupParameter<>("_LegJointGains", legGains, legNames));
      jointspaceGains.add(new GroupParameter<>("_CycloidArmJointGains", cycloidUpperArmGains, cycloidArmJointNames));

      return jointspaceGains;
   }

   private PIDGains createSpineControlGains()
   {
      // TODO Needs tune up
      PIDGains spineGains = new PIDGains();

      double kp = 50.0;
      double zeta = 0.8;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = Double.POSITIVE_INFINITY;
      double maxJerk = Double.POSITIVE_INFINITY;

      spineGains.setKp(kp);
      spineGains.setZeta(zeta);
      spineGains.setKi(ki);
      spineGains.setMaxIntegralError(maxIntegralError);
      spineGains.setMaximumFeedback(maxAccel);
      spineGains.setMaximumFeedbackRate(maxJerk);

      return spineGains;
   }

   private PIDGains createNeckControlGains()
   {
      // TODO Needs tune up
      PIDGains gains = new PIDGains();

      double kp = 40.0;
      double zeta = 0.8;
      double maxAccel = 36.0;
      double maxJerk = 540.0;

      gains.setKp(kp);
      gains.setZeta(zeta);
      gains.setMaximumFeedback(maxAccel);
      gains.setMaximumFeedbackRate(maxJerk);

      return gains;
   }

   private PIDGains createCycloidArmGains()
   {
      PIDGains armGains = new PIDGains();

      double kp = 120.0;
      double zeta = 0.7;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = Double.POSITIVE_INFINITY;
      double maxJerk = Double.POSITIVE_INFINITY;

      armGains.setKp(kp);
      armGains.setZeta(zeta);
      armGains.setKi(ki);
      armGains.setMaxIntegralError(maxIntegralError);
      armGains.setMaximumFeedback(maxAccel);
      armGains.setMaximumFeedbackRate(maxJerk);

      return armGains;
   }

   private PIDGains createLegControlGains()
   {
      PIDGains armGains = new PIDGains();

      double kp = 120.0;
      double zeta = 0.7;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = Double.POSITIVE_INFINITY;
      double maxJerk = Double.POSITIVE_INFINITY;

      armGains.setKp(kp);
      armGains.setZeta(zeta);
      armGains.setKi(ki);
      armGains.setMaxIntegralError(maxIntegralError);
      armGains.setMaximumFeedback(maxAccel);
      armGains.setMaximumFeedbackRate(maxJerk);

      return armGains;
   }

   @Override
   public List<GroupParameter<PID3DConfiguration>> getTaskspaceOrientationControlGains()
   {
      List<GroupParameter<PID3DConfiguration>> taskspaceAngularGains = new ArrayList<>();

      PID3DGains chestAngularGains = createChestOrientationControlGains();
      PID3DConfiguration chestAngularGainConfiguration = new PID3DConfiguration(GainCoupling.XY, false, chestAngularGains);
      taskspaceAngularGains.add(new GroupParameter<>("Chest", chestAngularGainConfiguration, jointMap.getChestName()));

      PID3DGains headAngularGains = createHeadOrientationControlGains();
      PID3DConfiguration headAngularGainConfiguration = new PID3DConfiguration(GainCoupling.XYZ, false, headAngularGains);
      taskspaceAngularGains.add(new GroupParameter<>("Head", headAngularGainConfiguration, jointMap.getHeadName()));

      boolean isSymmetric = jointMap.hasCycloidForearm(RobotSide.LEFT) == jointMap.hasCycloidForearm(RobotSide.RIGHT);
      if (isSymmetric)
      {
         boolean is4DoFArms = !jointMap.hasCycloidForearm(RobotSide.LEFT);
         PID3DGains handAngularGains = createHandOrientationControlGains(is4DoFArms);
         GainCoupling handGainCoupling = is4DoFArms ? GainCoupling.XY : GainCoupling.XYZ;
         PID3DConfiguration handAngularGainConfiguration = new PID3DConfiguration(handGainCoupling, false, handAngularGains);
         taskspaceAngularGains.add(new GroupParameter<>("Hand", handAngularGainConfiguration, jointMap.getHandNames()));
      }
      else
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            boolean is4DoFArms = !jointMap.hasCycloidForearm(robotSide);
            PID3DGains handAngularGains = createHandOrientationControlGains(is4DoFArms);
            GainCoupling handGainCoupling = is4DoFArms ? GainCoupling.XY : GainCoupling.XYZ;
            PID3DConfiguration handAngularGainConfiguration = new PID3DConfiguration(handGainCoupling, false, handAngularGains);
            taskspaceAngularGains.add(new GroupParameter<>(robotSide.getPascalCaseName() + "_Hand",
                                                           handAngularGainConfiguration,
                                                           jointMap.getHandName(robotSide)));
         }
      }

      PID3DGains pelvisAngularGains = createPelvisOrientationControlGains();
      PID3DConfiguration pelvisAngularGainConfiguration = new PID3DConfiguration(GainCoupling.XY, false, pelvisAngularGains);
      taskspaceAngularGains.add(new GroupParameter<>("Pelvis", pelvisAngularGainConfiguration, jointMap.getPelvisName()));

      return taskspaceAngularGains;
   }

   protected PID3DGains createPelvisOrientationControlGains()
   {
      // TODO Needs tune up
      double kpXY = 80.0;
      double kpZ = 80.0;
      double zeta = 0.8;
      double maxAccel = Double.POSITIVE_INFINITY;
      double maxJerk = Double.POSITIVE_INFINITY;

      DefaultPID3DGains gains = new DefaultPID3DGains();
      gains.setProportionalGains(kpXY, kpXY, kpZ);
      gains.setDampingRatios(zeta);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return gains;
   }

   private PID3DGains createHeadOrientationControlGains()
   {
      // TODO Needs tune up
      double kpX = 5.0;
      double kpYZ = 20.0;
      double zeta = 0.8;
      double maxAccel = 36.0;
      double maxJerk = 540.0;

      DefaultPID3DGains gains = new DefaultPID3DGains();
      gains.setProportionalGains(kpX, kpYZ, kpYZ);
      gains.setDampingRatios(zeta);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return gains;
   }

   private PID3DGains createChestOrientationControlGains()
   {
      // TODO Needs tune up
      double kpXY = 100.0;
      double kpZ = 100.0;
      double zetaXY = 0.8;
      double zetaZ = 0.8;
      double maxAccel = Double.POSITIVE_INFINITY;
      double maxJerk = Double.POSITIVE_INFINITY;

      DefaultPID3DGains gains = new DefaultPID3DGains();
      gains.setProportionalGains(kpXY, kpXY, kpZ);
      gains.setDampingRatios(zetaXY, zetaXY, zetaZ);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return gains;
   }

   private PID3DGains createHandOrientationControlGains(boolean is4DoFArm)
   {
      DefaultPID3DGains gains = new DefaultPID3DGains();
      double ki = 0.0;
      double maxIntegralError = 0.0;

      if (is4DoFArm)
      { // For 4-dof arm, only control Z orientation of hand and place at higher gain
         double kp = 85.0;
         double zeta = 0.65;
         gains.setProportionalGains(0.0, 0.0, kp);
         gains.setDampingRatios(0.0, 0.0, zeta);
         gains.setIntegralGains(0.0, 0.0, ki, maxIntegralError);

         double maximumOrientationError = Math.toRadians(22.0);
         double maximumOrientationRateError = Math.toRadians(150.0);
         gains.setMaxProportionalError(maximumOrientationError);
         gains.setMaxDerivativeError(maximumOrientationRateError);
      }
      else
      {
         double kp = 80.0;
         double zeta = 0.9;
         double maxAccel = 8.0;
         double maxJerk = 100.0;

         gains.setProportionalGains(kp);
         gains.setDampingRatios(zeta);
         gains.setIntegralGains(ki, maxIntegralError);
         gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);
      }

      return gains;
   }

   @Override
   public List<GroupParameter<PID3DConfiguration>> getTaskspacePositionControlGains()
   {
      List<GroupParameter<PID3DConfiguration>> taskspaceLinearGains = new ArrayList<>();

      PID3DGains handLinearGains = createHandPositionControlGains();
      PID3DConfiguration handLinearGainConfiguration = new PID3DConfiguration(GainCoupling.XYZ, false, handLinearGains);
      taskspaceLinearGains.add(new GroupParameter<>("Hand", handLinearGainConfiguration, jointMap.getHandNames()));

      return taskspaceLinearGains;
   }

   private PID3DGains createHandPositionControlGains()
   {
      double kp = 80.0;
      double zeta = 1.2;
      double maximumPositionError = 0.12;
      double maximumVelocityError = 0.5;

      DefaultPID3DGains gains = new DefaultPID3DGains();
      gains.setProportionalGains(kp);
      gains.setDampingRatios(zeta);
      gains.setMaxProportionalError(maximumPositionError);
      gains.setMaxDerivativeError(maximumVelocityError);

      return gains;
   }

   /** {@inheritDoc} */
   @Override
   public Map<String, RigidBodyControlMode> getDefaultControlModesForRigidBodies()
   {
      Map<String, RigidBodyControlMode> defaultControlModes = new HashMap<>();
      defaultControlModes.put(jointMap.getChestName(), RigidBodyControlMode.JOINTSPACE);
      return defaultControlModes;
   }

   @Override
   public TObjectDoubleHashMap<String> getOrCreateJointHomeConfiguration()
   {
      if (jointHomeConfiguration != null)
         return jointHomeConfiguration;

      jointHomeConfiguration = new TObjectDoubleHashMap<String>();

      for (SpineJointName name : jointMap.getSpineJointNames())
         jointHomeConfiguration.put(jointMap.getSpineJointName(name), 0.0);

      for (NeckJointName name : jointMap.getNeckJointNames())
         jointHomeConfiguration.put(jointMap.getNeckJointName(name), 0.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         jointHomeConfiguration.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW), 0.0);
         jointHomeConfiguration.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL), 0.0);
         jointHomeConfiguration.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), -0.175);
         jointHomeConfiguration.put(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), 0.433);
         jointHomeConfiguration.put(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH), -0.211);
         jointHomeConfiguration.put(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL), 0.0);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH), 0.018);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfRightSide(0.146));
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), robotSide.negateIfRightSide(-0.202));
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), -0.312);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_YAW), 0.0);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_ROLL), 0.0);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_YAW), 0.0);
      }

      return jointHomeConfiguration;
   }

   @Override
   public Map<String, Pose3D> getOrCreateBodyHomeConfiguration()
   {
      if (bodyHomeConfiguration != null)
         return bodyHomeConfiguration;

      bodyHomeConfiguration = new HashMap<String, Pose3D>();

      Pose3D homeChestPoseInPelvisZUpFrame = new Pose3D();
      bodyHomeConfiguration.put(jointMap.getChestName(), homeChestPoseInPelvisZUpFrame);

      for (RobotSide robotSide : RobotSide.values)
      { // Hand taskspace home pose can be computed from jointspace home pose with ZuluTaskspaceHomePoseCalculator

         Pose3D handPoseInChestBodyFrame = new Pose3D();

         if (version.getJointMap().hasCycloidForearm(robotSide))
         { // Cycloid forearm home pose
            handPoseInChestBodyFrame.getPosition().set(0.111, robotSide.negateIfRightSide(0.338), -0.641);
            handPoseInChestBodyFrame.getOrientation()
                                    .set(robotSide.negateIfRightSide(0.056), -0.140, robotSide.negateIfRightSide(-0.105), 0.983);
         }
         else
         { // 4-dof cycloid home pose
            handPoseInChestBodyFrame.getPosition().set(0.125, robotSide.negateIfRightSide(0.342), -0.695);
            handPoseInChestBodyFrame.getOrientation()
                                    .set(robotSide.negateIfRightSide(0.056), -0.140, robotSide.negateIfRightSide(-0.105), 0.983);
         }

         bodyHomeConfiguration.put(jointMap.getHandName(robotSide), handPoseInChestBodyFrame);
      }

      return bodyHomeConfiguration;
   }

   @Override
   public PIDSE3Configuration getSwingFootControlGains()
   {
      // TODO Needs tune up
      double kpXY = 150.0;
      double kpZ = 200.0;
      double zetaXY = 0.7;
      double zetaZ = 0.7;

      double kpXYOrientation = 200.0;
      double kpZOrientation = 200.0;
      double zetaXYOrientation = 0.7;
      double zetaZOrientation = 0.7;

      double maxPositionAcceleration = Double.POSITIVE_INFINITY;
      double maxPositionJerk = Double.POSITIVE_INFINITY;
      double maxOrientationAcceleration = Double.POSITIVE_INFINITY;
      double maxOrientationJerk = Double.POSITIVE_INFINITY;

      DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains();
      gains.setPositionProportionalGains(kpXY, kpXY, kpZ);
      gains.setPositionDampingRatios(zetaXY, zetaXY, zetaZ);
      gains.setPositionMaxFeedbackAndFeedbackRate(maxPositionAcceleration, maxPositionJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatios(zetaXYOrientation, zetaXYOrientation, zetaZOrientation);
      gains.setOrientationMaxFeedbackAndFeedbackRate(maxOrientationAcceleration, maxOrientationJerk);

      return new PIDSE3Configuration(GainCoupling.XY, false, gains);
   }

   @Override
   public PIDSE3Configuration getHoldPositionFootControlGains()
   {
      // TODO Needs tune up
      double kpXY = 100.0;
      double kpZ = 0.0;
      double zetaXYZ = 1.0;
      double kpXYOrientation = 100.0;
      double kpZOrientation = 200.0;
      double zetaOrientation = 1.0;
      double maxLinearAcceleration = Double.POSITIVE_INFINITY;
      double maxLinearJerk = Double.POSITIVE_INFINITY;
      double maxAngularAcceleration = Double.POSITIVE_INFINITY;
      double maxAngularJerk = Double.POSITIVE_INFINITY;

      DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains();
      gains.setPositionProportionalGains(kpXY, kpXY, kpZ);
      gains.setPositionDampingRatios(zetaXYZ);
      gains.setPositionMaxFeedbackAndFeedbackRate(maxLinearAcceleration, maxLinearJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatios(zetaOrientation);
      gains.setOrientationMaxFeedbackAndFeedbackRate(maxAngularAcceleration, maxAngularJerk);

      return new PIDSE3Configuration(GainCoupling.XY, false, gains);
   }

   @Override
   public PIDSE3Configuration getToeOffFootControlGains()
   {
      double kpXY = 40.0;
      double kpZ = 0.0;
      double zetaXYZ = 0.4;
      double kpXYOrientation = 200.0;
      double kpZOrientation = 200.0;
      double zetaOrientation = 0.4;
      double maxLinearAcceleration = Double.POSITIVE_INFINITY;
      double maxLinearJerk = Double.POSITIVE_INFINITY;
      double maxAngularAcceleration = Double.POSITIVE_INFINITY;
      double maxAngularJerk = Double.POSITIVE_INFINITY;

      DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains();
      gains.setPositionProportionalGains(kpXY, kpXY, kpZ);
      gains.setPositionDampingRatios(zetaXYZ);
      gains.setPositionMaxFeedbackAndFeedbackRate(maxLinearAcceleration, maxLinearJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatios(zetaOrientation);
      gains.setOrientationMaxFeedbackAndFeedbackRate(maxAngularAcceleration, maxAngularJerk);

      return new PIDSE3Configuration(GainCoupling.XY, false, gains);
   }

   public double getSwingMaxHeightForPushRecoveryTrajectory()
   {
      // TODO Needs tune up
      return 0.15;
   }

   @Override
   public double getDefaultTransferTime()
   {
      return 0.25;
   }

   @Override
   public double getDefaultSwingTime()
   {
      return 0.60;
   }

   @Override
   public double getDefaultInitialTransferTime()
   {
      return 1.0;
   }

   @Override
   public String[] getJointsToIgnoreInController()
   {
      return new String[0];
   }

   @Override
   public String[] getJointsWithRestrictiveLimits()
   {
      String leftKnyName = jointMap.getLegJointName(RobotSide.LEFT, LegJointName.KNEE_PITCH);
      String rightKnyName = jointMap.getLegJointName(RobotSide.RIGHT, LegJointName.KNEE_PITCH);
      String[] joints = {leftKnyName, rightKnyName};
      return joints;
   }

   @Override
   public JointLimitParameters getJointLimitParametersForJointsWithRestrictiveLimits(String jointName)
   {
      if (jointMap.getLegJointName(jointName) != null)
      {
         if (jointMap.getLegJointName(jointName).getRight() == LegJointName.KNEE_PITCH)
            return kneeJointLimitParameters;
      }

      return null;
   }

   @Override
   public MomentumOptimizationSettings getMomentumOptimizationSettings()
   {
      return momentumOptimizationSettings;
   }

   @Override
   public double getMaxICPErrorBeforeSingleSupportForwardX()
   {
      // TODO Needs tune up
      return 0.035;
   }

   @Override
   public double getMaxICPErrorBeforeSingleSupportInnerY()
   {
      // TODO Needs tune up
      return 0.015;
   }

   @Override
   public boolean finishSingleSupportWhenICPPlannerIsDone()
   {
      return false;
   }

   @Override
   public double getHighCoPDampingDurationToPreventFootShakies()
   {
      return -1.0;
   }

   @Override
   public double getCoPErrorThresholdForHighCoPDamping()
   {
      return Double.POSITIVE_INFINITY;
   }


   @Override
   public ToeOffParameters getToeOffParameters()
   {
      return toeOffParameters;
   }

   @Override
   public SwingTrajectoryParameters getSwingTrajectoryParameters()
   {
      return swingTrajectoryParameters;
   }

   @Override
   public SteppingParameters getSteppingParameters()
   {
      return steppingParameters;
   }

   @Override
   public double getMinSwingTrajectoryClearanceFromStanceFoot()
   {
      return 0.15 * jointMap.getModelScale();
   }

   @Override
   public FootSwitchFactory getFootSwitchFactory()
   {
      WrenchBasedFootSwitchFactory factory = new WrenchBasedFootSwitchFactory();
      factory.setDefaultContactThresholdForce(50.0);
      factory.setDefaultCoPThresholdDistance(4.0e-3);
      factory.setDefaultSecondContactThresholdForceIgnoringCoP(75.0);
      return factory;
   }

   @Override
   public ICPControllerParameters getICPControllerParameters()
   {
      return icpControllerParameters;
   }

   @Override
   public StepAdjustmentParameters getStepAdjustmentParameters()
   {
      return stepAdjustmentParameters;
   }

   @Override
   public boolean allowUpperBodyMotionDuringLocomotion()
   {
      return allowUpperBodyMotionDuringLocomotion;
   }

   public void setAllowUpperBodyMotionDuringLocomotion(boolean allowUpperBodyMotionDuringLocomotion)
   {
      this.allowUpperBodyMotionDuringLocomotion = allowUpperBodyMotionDuringLocomotion;
   }

   @Override
   public boolean doPrepareManipulationForLocomotion()
   {
      return doPrepareManipulationForLocomotion;
   }

   public void setDoPrepareManipulationForLocomotion(boolean doPrepareManipulationForLocomotion)
   {
      this.doPrepareManipulationForLocomotion = doPrepareManipulationForLocomotion;
   }

   @Override
   public boolean doPreparePelvisForLocomotion()
   {
      return doPreparePelvisForLocomotion;
   }

   public void setDoPreparePelvisForLocomotion(boolean doPreparePelvisForLocomotion)
   {
      this.doPreparePelvisForLocomotion = doPreparePelvisForLocomotion;
   }

   public void setSwingTrajectoryParameters(SwingTrajectoryParameters swingTrajectoryParameters)
   {
      this.swingTrajectoryParameters = swingTrajectoryParameters;
   }

   public void setJointPrivilegedConfigurationParameters(JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters)
   {
      this.jointPrivilegedConfigurationParameters = jointPrivilegedConfigurationParameters;
   }

   /** {@inheritDoc} */
   @Override
   public JointPrivilegedConfigurationParameters getJointPrivilegedConfigurationParameters()
   {
      return jointPrivilegedConfigurationParameters;
   }

   /** {@inheritDoc} */
   @Override
   public OneDoFJointPrivilegedConfigurationParameters getKneePrivilegedConfigurationParameters()
   {
      return kneePrivilegedConfigurationParameters;
   }

   @Override
   public NaturalPostureParameters getNaturalPostureParameters()
   {
      // 240308 - disabled NP while integrating cycloid forearms

      return null;
   }

   public double getFractionOfSwingToSwitchToLoaded()
   {
      return Double.NaN;
   }

   @Override
   public boolean enableFunctionGeneratorMode(String rigidBodyName)
   {
      if (rigidBodyName.contains("ELBOW"))
      { // Right hand
         return false;
      }
      if (rigidBodyName.contains("GRIPPER"))
      { // Left hand
         return false;
      }

      return false;
   }

   @Override
   public List<String> getJointsToCheckTorqueFeasibilityInMultiContact()
   {
      return Arrays.asList("RIGHT_SHOULDER_Y", "RIGHT_SHOULDER_X", "RIGHT_SHOULDER_Z", "RIGHT_ELBOW_Y",
                           "LEFT_SHOULDER_Y", "LEFT_SHOULDER_X", "LEFT_SHOULDER_Z", "LEFT_ELBOW_Y",
                           "SPINE_Z", "SPINE_X", "SPINE_Y");
   }
}
package us.ihmc.valkyrie.parameters;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentParameters;
import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OneDoFJointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitchFactory;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PID3DConfiguration;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDSE3Configuration;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.FootSwitchFactory;

public class ValkyrieWalkingControllerParameters extends WalkingControllerParameters
{
   private final RobotTarget target;

   protected final ValkyrieJointMap jointMap;

   private TObjectDoubleHashMap<String> jointHomeConfiguration = null;
   private Map<String, Pose3D> bodyHomeConfiguration = null;

   private final ToeOffParameters toeOffParameters;
   private final SwingTrajectoryParameters swingTrajectoryParameters;
   private final ValkyrieSteppingParameters steppingParameters;
   private final ICPControllerParameters icpControllerParameters;
   private final StepAdjustmentParameters stepAdjustmentParameters;

   private final ValkyriePhysicalProperties physicalProperties;

   private final OneDoFJointPrivilegedConfigurationParameters kneePrivilegedConfigurationParameters;

   // USE THESE FOR Real Robot and sims when controlling pelvis height instead of CoM.
   private final double minimumHeightAboveGround;
   private final double nominalHeightAboveGround;
   private final double maximumHeightAboveGround;

   public ValkyrieWalkingControllerParameters(ValkyrieJointMap jointMap, ValkyriePhysicalProperties physicalProperties)
   {
      this(jointMap, physicalProperties, RobotTarget.SCS);
   }

   public ValkyrieWalkingControllerParameters(ValkyrieJointMap jointMap, ValkyriePhysicalProperties physicalProperties, RobotTarget target)
   {
      this.jointMap = jointMap;
      this.physicalProperties = physicalProperties;
      this.target = target;

      toeOffParameters = new ValkyrieToeOffParameters(physicalProperties, target);
      swingTrajectoryParameters = new ValkyrieSwingTrajectoryParameters(physicalProperties, target);
      steppingParameters = new ValkyrieSteppingParameters(physicalProperties, target);
      icpControllerParameters = new ValkyrieICPControllerParameters(target);
      stepAdjustmentParameters = new ValkyrieStepAdjustmentParameters(target);

      minimumHeightAboveGround = jointMap.getModelScale() * (0.595 + 0.23 + 0.08);
      nominalHeightAboveGround = jointMap.getModelScale() * (0.675 + 0.23 - 0.01 + 0.08);
      maximumHeightAboveGround = jointMap.getModelScale() * (0.735 + 0.23 + 0.08);


      kneePrivilegedConfigurationParameters = new OneDoFJointPrivilegedConfigurationParameters();
      kneePrivilegedConfigurationParameters.setConfigurationGain(target == RobotTarget.REAL_ROBOT ? 40.0 : 150.0);
      kneePrivilegedConfigurationParameters.setVelocityGain(6.0);
      kneePrivilegedConfigurationParameters.setWeight(5.0);
      kneePrivilegedConfigurationParameters.setMaxAcceleration(Double.POSITIVE_INFINITY);
      kneePrivilegedConfigurationParameters.setPrivilegedConfigurationOption(PrivilegedConfigurationCommand.PrivilegedConfigurationOption.AT_MID_RANGE);

   }

   @Override
   public double getOmega0()
   {
      // TODO probably need to be tuned.
      return (target == RobotTarget.REAL_ROBOT ? 3.0 : 3.3) / Math.sqrt(jointMap.getModelScale()); // 3.3 seems more appropriate.
   }

   @Override
   public double getMaxAllowedDistanceCMPSupport()
   {
      return target == RobotTarget.REAL_ROBOT ? 0.0 : 0.04;
   }

   @Override
   public boolean allowDisturbanceRecoveryBySpeedingUpSwing()
   {
      return true;
   }

   @Override
   public boolean allowAutomaticManipulationAbort()
   {
      return false;
   }

   @Override
   public double getICPErrorThresholdToSpeedUpSwing()
   {
      return 0.05 * jointMap.getModelScale();
   }

   @Override
   public boolean allowUpperBodyMotionDuringLocomotion()
   {
      if (target == RobotTarget.SCS)
         return false;
      else
         return true;
   }

   @Override
   public boolean doPrepareManipulationForLocomotion()
   {
      if (target == RobotTarget.SCS)
         return true;
      else
         return false;
   }

   @Override
   public double getMinimumSwingTimeForDisturbanceRecovery()
   {
      return target != RobotTarget.SCS ? 0.70 : 0.30;
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
   public double defaultOffsetHeightAboveAnkle()
   {
      return 0.0;
   }

   @Override
   public double getMaximumLegLengthForSingularityAvoidance()
   {
      return physicalProperties.getThighLength() + physicalProperties.getShinLength();
   }

   @Override
   public boolean controlToeDuringSwing()
   {
      return true;
   }

   @Override
   public boolean ignoreSwingInitialAngularVelocityZ()
   {
      return true;
   }

   @Override
   public double getMaxSwingInitialAngularVelocityMagnitude()
   {
      return 1.5;
   }

   @Override
   public PDGains getCoMHeightControlGains()
   {
      PDGains gains = new PDGains();

      boolean runningOnRealRobot = target == RobotTarget.REAL_ROBOT;

      double kp = runningOnRealRobot ? 40.0 : 50.0;
      double zeta = runningOnRealRobot ? 0.4 : 1.0;
      double maxAcceleration = 0.5 * 9.81;
      double maxJerk = maxAcceleration / 0.05;

      gains.setKp(kp);
      gains.setZeta(zeta);
      gains.setMaximumFeedback(maxAcceleration);
      gains.setMaximumFeedbackRate(maxJerk);

      return gains;
   }

   /** {@inheritDoc} */
   @Override
   public List<GroupParameter<PIDGainsReadOnly>> getJointSpaceControlGains()
   {
      PIDGains spineGains = createSpineControlGains();
      PIDGains neckGains = createNeckControlGains();
      PIDGains armGains = createArmControlGains();

      List<GroupParameter<PIDGainsReadOnly>> jointspaceGains = new ArrayList<>();
      jointspaceGains.add(new GroupParameter<>("SpineJoints", spineGains, jointMap.getSpineJointNamesAsStrings()));
      jointspaceGains.add(new GroupParameter<>("NeckJoints", neckGains, jointMap.getNeckJointNamesAsStrings()));
      jointspaceGains.add(new GroupParameter<>("ArmJoints", armGains, jointMap.getArmJointNamesAsStrings()));

      return jointspaceGains;
   }

   private PIDGains createSpineControlGains()
   {
      PIDGains spineGains = new PIDGains();
      boolean runningOnRealRobot = target == RobotTarget.REAL_ROBOT;

      double kp = 50.0;
      double zeta = runningOnRealRobot ? 0.3 : 0.8;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = runningOnRealRobot ? 20.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;

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
      PIDGains gains = new PIDGains();
      boolean realRobot = target == RobotTarget.REAL_ROBOT;

      double kp = 40.0;
      double zeta = realRobot ? 0.4 : 0.8;
      double maxAccel = 18.0;
      double maxJerk = 270.0;

      gains.setKp(kp);
      gains.setZeta(zeta);
      gains.setMaximumFeedback(maxAccel);
      gains.setMaximumFeedbackRate(maxJerk);

      return gains;
   }

   private PIDGains createArmControlGains()
   {
      PIDGains armGains = new PIDGains();
      boolean runningOnRealRobot = target == RobotTarget.REAL_ROBOT;

      double kp = runningOnRealRobot ? 200.0 : 120.0; // 200.0
      double zeta = runningOnRealRobot ? 1.0 : 0.7;
      double ki = runningOnRealRobot ? 0.0 : 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 2000.0 : Double.POSITIVE_INFINITY;

      armGains.setKp(kp);
      armGains.setZeta(zeta);
      armGains.setKi(ki);
      armGains.setMaxIntegralError(maxIntegralError);
      armGains.setMaximumFeedback(maxAccel);
      armGains.setMaximumFeedbackRate(maxJerk);

      return armGains;
   }

   /** {@inheritDoc} */
   @Override
   public List<GroupParameter<PID3DConfiguration>> getTaskspaceOrientationControlGains()
   {
      List<GroupParameter<PID3DConfiguration>> taskspaceAngularGains = new ArrayList<>();

      PID3DConfiguration chestAngularGains = createChestOrientationControlGains();
      taskspaceAngularGains.add(new GroupParameter<>("Chest", chestAngularGains, jointMap.getChestName()));

      PID3DConfiguration headAngularGains = createHeadOrientationControlGains();
      taskspaceAngularGains.add(new GroupParameter<>("Head", headAngularGains, jointMap.getHeadName()));

      PID3DConfiguration handAngularGains = createHandOrientationControlGains();
      taskspaceAngularGains.add(new GroupParameter<>("Hand", handAngularGains, jointMap.getHandNames()));

      PID3DConfiguration pelvisAngularGains = createPelvisOrientationControlGains();
      taskspaceAngularGains.add(new GroupParameter<>("Pelvis", pelvisAngularGains, jointMap.getPelvisName()));

      return taskspaceAngularGains;
   }

   private PID3DConfiguration createPelvisOrientationControlGains()
   {
      boolean runningOnRealRobot = target == RobotTarget.REAL_ROBOT;

      double kpXY = runningOnRealRobot ? 100.0 : 100.0; // Was 100.0 before tuneup of sep 2018
      double kpZ = runningOnRealRobot ? 90.0 : 100.0; // Was 80.0 before tuneup of sep 2018
      double zetaXY = runningOnRealRobot ? 0.8 : 0.8; // Was 0.9 before tuneup of sep 2018
      double zetaZ = runningOnRealRobot ? 0.8 : 0.8;
      double maxAccel = runningOnRealRobot ? 100.0 : 18.0; // Was 18.0 before tuneup of sep 2018
      double maxJerk = runningOnRealRobot ? 1500.0 : 270.0; // Was 270.0 before tuneup of sep 2018

      DefaultPID3DGains gains = new DefaultPID3DGains();
      gains.setProportionalGains(kpXY, kpXY, kpZ);
      gains.setDampingRatios(zetaXY, zetaXY, zetaZ);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return new PID3DConfiguration(GainCoupling.XY, false, gains);
   }

   private PID3DConfiguration createHeadOrientationControlGains()
   {
      boolean runningOnRealRobot = target == RobotTarget.REAL_ROBOT;

      double kpX = 5.0;
      double kpYZ = 20.0;//40.0;
      double zeta = runningOnRealRobot ? 0.4 : 0.8;
      double maxAccel = 18.0;
      double maxJerk = 270.0;

      DefaultPID3DGains gains = new DefaultPID3DGains();
      gains.setProportionalGains(kpX, kpYZ, kpYZ);
      gains.setDampingRatios(zeta, zeta, zeta);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return new PID3DConfiguration(GainCoupling.YZ, false, gains);
   }

   private PID3DConfiguration createChestOrientationControlGains()
   {
      boolean runningOnRealRobot = target == RobotTarget.REAL_ROBOT;

      double kpXY = runningOnRealRobot ? 80.0 : 100.0; // Was 80.0 before tuneup of sep 2018
      double kpZ = runningOnRealRobot ? 80.0 : 100.0; // Was 60.0 before tuneup of sep 2018
      double zetaXY = runningOnRealRobot ? 0.8 : 0.8; // Was 0.8 before tuneup of sep 2018
      double zetaZ = runningOnRealRobot ? 0.8 : 0.8; // Was 0.8 before tuneup of sep 2018
      double maxAccel = runningOnRealRobot ? 12.0 : 18.0;
      double maxJerk = runningOnRealRobot ? 360.0 : 270.0;

      DefaultPID3DGains gains = new DefaultPID3DGains();
      gains.setProportionalGains(kpXY, kpXY, kpZ);
      gains.setDampingRatios(zetaXY, zetaXY, zetaZ);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return new PID3DConfiguration(GainCoupling.XY, false, gains);
   }

   private PID3DConfiguration createHandOrientationControlGains()
   {
      boolean runningOnRealRobot = target == RobotTarget.REAL_ROBOT;

      double kp = 100.0;
      double zeta = runningOnRealRobot ? 0.6 : 1.0;
      double maxAccel = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;

      DefaultPID3DGains gains = new DefaultPID3DGains();
      gains.setProportionalGains(kp);
      gains.setDampingRatios(zeta);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return new PID3DConfiguration(GainCoupling.XYZ, false, gains);
   }

   /** {@inheritDoc} */
   @Override
   public List<GroupParameter<PID3DConfiguration>> getTaskspacePositionControlGains()
   {
      List<GroupParameter<PID3DConfiguration>> taskspaceLinearGains = new ArrayList<>();

      PID3DConfiguration handLinearGains = createHandPositionControlGains();
      taskspaceLinearGains.add(new GroupParameter<>("Hand", handLinearGains, jointMap.getHandNames()));

      return taskspaceLinearGains;
   }

   private PID3DConfiguration createHandPositionControlGains()
   {
      boolean runningOnRealRobot = target == RobotTarget.REAL_ROBOT;

      double kp = 100.0;
      double zeta = runningOnRealRobot ? 0.6 : 1.0;
      double maxAccel = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;

      DefaultPID3DGains gains = new DefaultPID3DGains();
      gains.setProportionalGains(kp);
      gains.setDampingRatios(zeta);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return new PID3DConfiguration(GainCoupling.XYZ, false, gains);
   }

   /** {@inheritDoc} */
   @Override
   public Map<String, RigidBodyControlMode> getDefaultControlModesForRigidBodies()
   {
      Map<String, RigidBodyControlMode> defaultControlModes = new HashMap<>();
      defaultControlModes.put(jointMap.getChestName(), RigidBodyControlMode.TASKSPACE);
      return defaultControlModes;
   }

   /** {@inheritDoc} */
   @Override
   public TObjectDoubleHashMap<String> getOrCreateJointHomeConfiguration()
   {
      if (jointHomeConfiguration != null)
         return jointHomeConfiguration;

      jointHomeConfiguration = new TObjectDoubleHashMap<String>();

      jointHomeConfiguration.put(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH), 0.0);
      jointHomeConfiguration.put(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL), 0.0);
      jointHomeConfiguration.put(jointMap.getSpineJointName(SpineJointName.SPINE_YAW), 0.0);

      jointHomeConfiguration.put(jointMap.getNeckJointName(NeckJointName.PROXIMAL_NECK_PITCH), 0.75);
      jointHomeConfiguration.put(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_YAW), 0.0);
      jointHomeConfiguration.put(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_PITCH), -0.1);

      for (RobotSide robotSide : RobotSide.values)
      {
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH), 0.4);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfRightSide(-1.0));
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), 0.1);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), robotSide.negateIfRightSide(-1.3));
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL), 1.0);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_ROLL), 0.0);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH), 0.0);
      }

      return jointHomeConfiguration;
   }

   /** {@inheritDoc} */
   @Override
   public Map<String, Pose3D> getOrCreateBodyHomeConfiguration()
   {
      if (bodyHomeConfiguration != null)
         return bodyHomeConfiguration;

      bodyHomeConfiguration = new HashMap<String, Pose3D>();

      Pose3D homeChestPoseInPelvisZUpFrame = new Pose3D();
      bodyHomeConfiguration.put(jointMap.getChestName(), homeChestPoseInPelvisZUpFrame);

      return bodyHomeConfiguration;
   }

   @Override
   public PIDSE3Configuration getSwingFootControlGains()
   {
      boolean runningOnRealRobot = target == RobotTarget.REAL_ROBOT;

      double kpX = runningOnRealRobot ? 100.0 : 150.0; // Was 150.0 before tuneup of sep 2018
      double kpY = runningOnRealRobot ? 100.0 : 150.0; // Was 100.0 before tuneup of sep 2018
      double kpZ = runningOnRealRobot ? 250.0 : 200.0; // Was 200.0 before tuneup of sep 2018
      // zeta was [0.8, 0.5, 0.8] before tuneup of sep 2018
      double zetaXY = runningOnRealRobot ? 0.7 : 0.7;
      double zetaZ = runningOnRealRobot ? 0.8 : 0.7;
      double kpXYOrientation = runningOnRealRobot ? 200.0 : 300.0;
      double kpZOrientation = runningOnRealRobot ? 150.0 : 200.0; // 160
      double zetaOrientationXY = runningOnRealRobot ? 0.8 : 0.7; // Was 0.7 before tuneup of sep 2018
      double zetaOrientationZ = runningOnRealRobot ? 0.8 : 0.7; // Was 0.5 before tuneup of sep 2018
      double maxLinearAcceleration = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxLinearJerk = runningOnRealRobot ? 250.0 : Double.POSITIVE_INFINITY;
      double maxAngularAcceleration = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;
      double maxAngularJerk = runningOnRealRobot ? 1500.0 : Double.POSITIVE_INFINITY;

      DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains();
      gains.setPositionProportionalGains(kpX, kpY, kpZ);
      gains.setPositionDampingRatios(zetaXY, zetaXY, zetaZ);
      gains.setPositionMaxFeedbackAndFeedbackRate(maxLinearAcceleration, maxLinearJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatios(zetaOrientationXY, zetaOrientationXY, zetaOrientationZ);
      gains.setOrientationMaxFeedbackAndFeedbackRate(maxAngularAcceleration, maxAngularJerk);

      return new PIDSE3Configuration(GainCoupling.NONE, false, gains);
   }

   @Override
   public PIDSE3Configuration getHoldPositionFootControlGains()
   {
      boolean runningOnRealRobot = target == RobotTarget.REAL_ROBOT;

      double kpXY = 0.0; //40.0;
      double kpZ = 0.0;
      double zetaXYZ = runningOnRealRobot ? 0.7 : 1.0;
      double kpXYOrientation = runningOnRealRobot ? 0.0 : 200.0; // 40.0
      double kpZOrientation = runningOnRealRobot ? 100.0 : 200.0; // 120.0
      double zetaOrientation = runningOnRealRobot ? 0.7 : 1.0;
      double maxLinearAcceleration = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxLinearJerk = runningOnRealRobot ? 150.0 : Double.POSITIVE_INFINITY;
      double maxAngularAcceleration = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;
      double maxAngularJerk = runningOnRealRobot ? 1500.0 : Double.POSITIVE_INFINITY;

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
      boolean runningOnRealRobot = target == RobotTarget.REAL_ROBOT;

      double kpXY = 40.0;
      double kpZ = 0.0;
      double zetaXYZ = runningOnRealRobot ? 0.7 : 0.4;
      double kpXYOrientation = runningOnRealRobot ? 40.0 : 200.0;
      double kpZOrientation = runningOnRealRobot ? 40.0 : 200.0;
      double zetaOrientation = runningOnRealRobot ? 0.4 : 0.4;
      double maxLinearAcceleration = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxLinearJerk = runningOnRealRobot ? 150.0 : Double.POSITIVE_INFINITY;
      double maxAngularAcceleration = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;
      double maxAngularJerk = runningOnRealRobot ? 1500.0 : Double.POSITIVE_INFINITY;

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
   public double getDefaultTransferTime()
   {
      return target == RobotTarget.REAL_ROBOT ? 1.00 : 0.25;
   }

   @Override
   public double getDefaultSwingTime()
   {
      return target == RobotTarget.REAL_ROBOT ? 1.20 : 0.60;
   }

   /** @inheritDoc */
   @Override
   public double getDefaultInitialTransferTime()
   {
      return 1.0;
   }

   @Override
   public FootSwitchFactory getFootSwitchFactory()
   {
      WrenchBasedFootSwitchFactory factory = new WrenchBasedFootSwitchFactory();
      factory.setDefaultContactThresholdForce(target == RobotTarget.SCS ? 5.0 : 50.0);
      factory.setDefaultCoPThresholdFraction(0.01);
      factory.setDefaultSecondContactThresholdForceIgnoringCoP(75.0);
      return factory;
   }

   @Override
   public String[] getJointsToIgnoreInController()
   {
      List<String> jointToIgnoreList = new ArrayList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         String[] forcedSideJointNames = ValkyrieOrderedJointMap.forcedSideDependentJointNames.get(robotSide);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftIndexFingerPitch1]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftIndexFingerPitch2]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftIndexFingerPitch3]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftMiddleFingerPitch1]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftMiddleFingerPitch2]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftMiddleFingerPitch3]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftPinkyPitch1]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftPinkyPitch2]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftPinkyPitch3]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftThumbRoll]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftThumbPitch1]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftThumbPitch2]);
         jointToIgnoreList.add(forcedSideJointNames[ValkyrieOrderedJointMap.LeftThumbPitch3]);
      }

      return jointToIgnoreList.toArray(new String[0]);
   }

   @Override
   public MomentumOptimizationSettings getMomentumOptimizationSettings()
   {
      return new ValkyrieMomentumOptimizationSettings(jointMap);
   }

   @Override
   public FeedbackControllerSettings getFeedbackControllerSettings()
   {
      return new ValkyrieFeedbackControllerSettings(jointMap, target);
   }

   @Override
   public double getMaxICPErrorBeforeSingleSupportForwardX()
   {
      return 0.02;
   }

   @Override
   public double getMaxICPErrorBeforeSingleSupportInnerY()
   {
      return 0.02;
   }

   @Override
   public boolean finishSingleSupportWhenICPPlannerIsDone()
   {
      return false;
   }

   /** {@inheritDoc} */
   @Override
   public double getHighCoPDampingDurationToPreventFootShakies()
   {
      return -1.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getCoPErrorThresholdForHighCoPDamping()
   {
      return Double.POSITIVE_INFINITY;
   }

   /** {@inheritDoc} */
   @Override
   public OneDoFJointPrivilegedConfigurationParameters getKneePrivilegedConfigurationParameters()
   {
      return kneePrivilegedConfigurationParameters;
   }

   /** {@inheritDoc} */
   @Override
   public boolean minimizeAngularMomentumRateZDuringSwing()
   {
      // For some reason it causes the test ValkyrieEndToEndCinderBlockFieldTest to fail by making the state estimator drift more than usual.
      // As there is no real need for it in sim, I'm leaving it on only for the real robot. (Sylvain)
      return target == RobotTarget.REAL_ROBOT;
   }

   /** {@inheritDoc} */
   @Override
   public ToeOffParameters getToeOffParameters()
   {
      return toeOffParameters;
   }

   /** {@inheritDoc} */
   @Override
   public SwingTrajectoryParameters getSwingTrajectoryParameters()
   {
      return swingTrajectoryParameters;
   }

   /** {@inheritDoc} */
   @Override
   public SteppingParameters getSteppingParameters()
   {
      return steppingParameters;
   }

   @Override
   public double getMinSwingTrajectoryClearanceFromStanceFoot()
   {
      return 0.18 * jointMap.getModelScale();
   }

   /** {@inheritDoc} */
   @Override
   public ICPControllerParameters getICPControllerParameters()
   {
      return icpControllerParameters;
   }

   /** {@inheritDoc} */
   @Override
   public StepAdjustmentParameters getStepAdjustmentParameters()
   {
      return stepAdjustmentParameters;
   }

   /** {@inheritDoc} */
   @Override
   public String[] getJointsWithRestrictiveLimits()
   {
      List<String> joints = new ArrayList<>();

      for (RobotSide robotSide : RobotSide.values)
      { // This is reduce frequency of joint faulting. These are the main joints that can easily fault.
         joints.add(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW));
         joints.add(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL));
         joints.add(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH));
         joints.add(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW));
      }

      return joints.toArray(new String[joints.size()]);
   }

   /** {@inheritDoc} */
   @Override
   public JointLimitParameters getJointLimitParametersForJointsWithRestrictiveLimits(String jointName)
   {
      JointLimitParameters parameters = new JointLimitParameters();
      parameters.setMaxAbsJointVelocity(1.5);
      parameters.setJointLimitDistanceForMaxVelocity(0.3);
      parameters.setJointLimitFilterBreakFrequency(15.0);
      parameters.setVelocityControlGain(20.0);
      return parameters;
   }
}

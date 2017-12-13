package us.ihmc.thor.parameters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.ICPAngularMomentumModifierParameters;
import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDGains;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;

public class ThorWalkingControllerParameters extends WalkingControllerParameters
{
   private final RobotTarget target;
   private final SideDependentList<RigidBodyTransform> handPosesWithRespectToChestFrame = new SideDependentList<RigidBodyTransform>();

   private final ThorJointMap jointMap;
   private final ToeOffParameters toeOffParameters;
   private final SwingTrajectoryParameters swingTrajectoryParameters;
   private final ThorSteppingParameters steppingParameters;

   public ThorWalkingControllerParameters(ThorJointMap jointMap)
   {
      this(RobotTarget.SCS, jointMap);
   }

   public ThorWalkingControllerParameters(RobotTarget target, ThorJointMap jointMap)
   {
      this.target = target;
      this.jointMap = jointMap;

      toeOffParameters = new ThorToeOffParameters();
      swingTrajectoryParameters = new ThorSwingTrajectoryParameters(target);
      steppingParameters = new ThorSteppingParameters();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyTransform transform = new RigidBodyTransform();

         double x = 0.20;
         double y = robotSide.negateIfRightSide(0.35);    // 0.30);
         double z = -0.40;
         transform.setTranslation(new Vector3D(x, y, z));

         RotationMatrix rotation = new RotationMatrix();
         double yaw = 0.0;    // robotSide.negateIfRightSide(-1.7);
         double pitch = 0.7;
         double roll = 0.0;    // robotSide.negateIfRightSide(-0.8);
         rotation.setYawPitchRoll(yaw, pitch, roll);
         transform.setRotation(rotation);

         handPosesWithRespectToChestFrame.put(robotSide, transform);
      }
   }

   @Override
   public double getOmega0()
   {
      // TODO probably need to be tuned.
      boolean realRobot = target == RobotTarget.REAL_ROBOT;
      return realRobot ? 3.0 : 3.0; // 3.0 seems more appropriate.
//      return 3.0;
   }

   @Override
   public boolean allowDisturbanceRecoveryBySpeedingUpSwing()
   {
      return true; // TODO Seems to work well but still need to be heavily tested on the robot.
   }

   @Override
   public boolean allowAutomaticManipulationAbort()
   {
      return true;
   }

   @Override
   public double getICPErrorThresholdToSpeedUpSwing()
   {
      return 0.05;
   }

   @Override
   public double getMinimumSwingTimeForDisturbanceRecovery()
   {
      if (target == RobotTarget.REAL_ROBOT)
         return 0.6;
      else
         return 0.3;
   }

   public boolean isNeckPositionControlled()
   {
      if (target == RobotTarget.REAL_ROBOT)
         return true;
      else
         return false;
   }

// USE THESE FOR Real Thor Robot and sims when controlling pelvis height instead of CoM.
   private final double minimumHeightAboveGround = 0.635;
   private double nominalHeightAboveGround = 0.85;
   private final double maximumHeightAboveGround = ThorPhysicalProperties.ankleHeight + ThorPhysicalProperties.thighLength + ThorPhysicalProperties.shinLength + 0.15;

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
      double defaultOffset = target == RobotTarget.REAL_ROBOT ? 0.035 : 0.0;
      return defaultOffset;
   }

   public void setNominalHeightAboveAnkle(double nominalHeightAboveAnkle)
   {
      this.nominalHeightAboveGround = nominalHeightAboveAnkle;
   }

   @Override
   public double getMaximumLegLengthForSingularityAvoidance()
   {
      return ThorPhysicalProperties.shinLength + ThorPhysicalProperties.thighLength;
   }

   @Override
   public ICPControlGains createICPControlGains()
   {
      ICPControlGains gains = new ICPControlGains();

      double kpParallel = 2.5;
      double kpOrthogonal = 1.5;
      double ki = 0.0;
      double kiBleedOff = 0.0;

      gains.setKpParallelToMotion(kpParallel);
      gains.setKpOrthogonalToMotion(kpOrthogonal);
      gains.setKi(ki);
      gains.setKiBleedOff(kiBleedOff);

//      boolean runningOnRealRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;
//      if (runningOnRealRobot) gains.setFeedbackPartMaxRate(1.0);
      return gains;
   }

   @Override
   public PDGains getCoMHeightControlGains()
   {
      PDGains gains = new PDGains();
      boolean realRobot = target == RobotTarget.REAL_ROBOT;

      double kp = 40.0;
      double zeta = realRobot ? 0.4 : 0.8;
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
      List<String> spineNames = new ArrayList<>();
      List<String> neckNames = new ArrayList<>();
      List<String> armNames = new ArrayList<>();

      Arrays.stream(jointMap.getSpineJointNames()).forEach(n -> spineNames.add(jointMap.getSpineJointName(n)));
      Arrays.stream(jointMap.getNeckJointNames()).forEach(n -> neckNames.add(jointMap.getNeckJointName(n)));
      for (RobotSide side : RobotSide.values)
      {
         Arrays.stream(jointMap.getArmJointNames()).forEach(n -> armNames.add(jointMap.getArmJointName(side, n)));
      }

      PIDGains spineGains = createSpineControlGains();
      PIDGains neckGains = createNeckControlGains();
      PIDGains armGains = createArmControlGains();

      List<GroupParameter<PIDGainsReadOnly>> jointspaceGains = new ArrayList<>();
      jointspaceGains.add(new GroupParameter<>("_SpineJointGains", spineGains, spineNames));
      jointspaceGains.add(new GroupParameter<>("_NeckJointGains", neckGains, neckNames));
      jointspaceGains.add(new GroupParameter<>("_ArmJointGains", armGains, armNames));

      return jointspaceGains;
   }

   private PIDGains createSpineControlGains()
   {
      PIDGains spineGains = new PIDGains();

      double kp = 250.0;
      double zeta = 0.6;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = target == RobotTarget.REAL_ROBOT ? 20.0 : Double.POSITIVE_INFINITY;
      double maxJerk = target == RobotTarget.REAL_ROBOT ? 100.0 : Double.POSITIVE_INFINITY;

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
      double maxAccel = realRobot ? 6.0 : 36.0;
      double maxJerk = realRobot ? 60.0 : 540.0;

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
      double maxAccel = runningOnRealRobot ? 50.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 750.0 : Double.POSITIVE_INFINITY;

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
   public List<GroupParameter<PID3DGainsReadOnly>> getTaskspaceOrientationControlGains()
   {
      List<GroupParameter<PID3DGainsReadOnly>> taskspaceAngularGains = new ArrayList<>();

      PID3DGains chestAngularGains = createChestOrientationControlGains();
      List<String> chestGainBodies = new ArrayList<>();
      chestGainBodies.add(jointMap.getChestName());
      taskspaceAngularGains.add(new GroupParameter<>("Chest", chestAngularGains, chestGainBodies));

      PID3DGains headAngularGains = createHeadOrientationControlGains();
      List<String> headGainBodies = new ArrayList<>();
      headGainBodies.add(jointMap.getHeadName());
      taskspaceAngularGains.add(new GroupParameter<>("Head", headAngularGains, headGainBodies));

      PID3DGains handAngularGains = createHandOrientationControlGains();
      List<String> handGainBodies = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         handGainBodies.add(jointMap.getHandName(robotSide));
      }
      taskspaceAngularGains.add(new GroupParameter<>("Hand", handAngularGains, handGainBodies));

      PID3DGains pelvisAngularGains = createPelvisOrientationControlGains();
      List<String> pelvisGainBodies = new ArrayList<>();
      pelvisGainBodies.add(jointMap.getPelvisName());
      taskspaceAngularGains.add(new GroupParameter<>("Pelvis", pelvisAngularGains, pelvisGainBodies));

      return taskspaceAngularGains;
   }

   private PID3DGains createPelvisOrientationControlGains()
   {
      boolean realRobot = target == RobotTarget.REAL_ROBOT;
      double kpXY = 80.0;
      double kpZ = 40.0;
      double zeta = realRobot ? 0.5 : 0.8;
      double maxAccel = realRobot ? 12.0 : 36.0;
      double maxJerk = realRobot ? 180.0 : 540.0;

      DefaultPID3DGains gains = new DefaultPID3DGains(GainCoupling.XY, false);
      gains.setProportionalGains(kpXY, kpXY, kpZ);
      gains.setDampingRatios(zeta);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return gains;
   }

   private PID3DGains createHeadOrientationControlGains()
   {
      boolean realRobot = target == RobotTarget.REAL_ROBOT;
      double kp = 40.0;
      double zeta = realRobot ? 0.4 : 0.8;
      double maxAccel = realRobot ? 6.0 : 36.0;
      double maxJerk = realRobot ? 60.0 : 540.0;

      DefaultPID3DGains gains = new DefaultPID3DGains(GainCoupling.XYZ, false);
      gains.setProportionalGains(kp);
      gains.setDampingRatios(zeta);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return gains;
   }

   private PID3DGains createChestOrientationControlGains()
   {
      boolean realRobot = target == RobotTarget.REAL_ROBOT;
      double kpXY = 40.0;
      double kpZ = 40.0;
      double zetaXY = realRobot ? 0.5 : 0.8;
      double zetaZ = realRobot ? 0.22 : 0.8;
      double maxAccel = realRobot ? 6.0 : 36.0;
      double maxJerk = realRobot ? 60.0 : 540.0;
      double maxProportionalError = 10.0 * Math.PI/180.0;

      DefaultPID3DGains gains = new DefaultPID3DGains(GainCoupling.XY, false);
      gains.setProportionalGains(kpXY, kpXY, kpZ);
      gains.setDampingRatios(zetaXY, zetaXY, zetaZ);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);
      gains.setMaxProportionalError(maxProportionalError);

      return gains;
   }

   private PID3DGains createHandOrientationControlGains()
   {
      boolean runningOnRealRobot = target == RobotTarget.REAL_ROBOT;
      double kp = 100.0;
      double zeta = runningOnRealRobot ? 0.6 : 1.0;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;

      DefaultPID3DGains gains = new DefaultPID3DGains(GainCoupling.XYZ, false);
      gains.setProportionalGains(kp);
      gains.setDampingRatios(zeta);
      gains.setIntegralGains(ki, maxIntegralError);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return gains;
   }

   /** {@inheritDoc} */
   @Override
   public List<GroupParameter<PID3DGainsReadOnly>> getTaskspacePositionControlGains()
   {
      List<GroupParameter<PID3DGainsReadOnly>> taskspaceLinearGains = new ArrayList<>();

      PID3DGains handLinearGains = createHandPositionControlGains();
      List<String> handGainBodies = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         handGainBodies.add(jointMap.getHandName(robotSide));
      }
      taskspaceLinearGains.add(new GroupParameter<>("Hand", handLinearGains, handGainBodies));

      return taskspaceLinearGains;
   }

   private PID3DGains createHandPositionControlGains()
   {
      boolean runningOnRealRobot = target == RobotTarget.REAL_ROBOT;

      double kp = 100.0;
      double zeta = runningOnRealRobot ? 0.6 : 1.0;
      double maxAccel = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;

      DefaultPID3DGains gains = new DefaultPID3DGains(GainCoupling.XYZ, false);
      gains.setProportionalGains(kp);
      gains.setDampingRatios(zeta);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return gains;
   }

   private TObjectDoubleHashMap<String> jointHomeConfiguration = null;
   /** {@inheritDoc} */
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
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH), 0.5);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfRightSide(0.2));
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), 0.0);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), -1.0);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_YAW), 0.0);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH), 0.0);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_ROLL), 0.0);
      }

      return jointHomeConfiguration;
   }

   @Override
   public PIDSE3Gains getSwingFootControlGains()
   {
      boolean realRobot = target == RobotTarget.REAL_ROBOT;

      double kpXY = 150.0;
      double kpZ = 200.0;
      double zetaXYZ = realRobot ? 0.7 : 0.7;

      double kpXYOrientation = 200.0;
      double kpZOrientation = 200.0;
      double zetaOrientation = 0.7;

      // Reduce maxPositionAcceleration from 30 to 6 to prevent too high acceleration when hitting joint limits.
      double maxPositionAcceleration = realRobot ? 20.0 : Double.POSITIVE_INFINITY;
//      double maxPositionAcceleration = realRobot ? 30.0 : Double.POSITIVE_INFINITY;
      double maxPositionJerk = realRobot ? 300.0 : Double.POSITIVE_INFINITY;
      double maxOrientationAcceleration = realRobot ? 100.0 : Double.POSITIVE_INFINITY;
      double maxOrientationJerk = realRobot ? 1500.0 : Double.POSITIVE_INFINITY;

      DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains(GainCoupling.XY, false);
      gains.setPositionProportionalGains(kpXY, kpXY, kpZ);
      gains.setPositionDampingRatios(zetaXYZ);
      gains.setPositionMaxFeedbackAndFeedbackRate(maxPositionAcceleration, maxPositionJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatios(zetaOrientation);
      gains.setOrientationMaxFeedbackAndFeedbackRate(maxOrientationAcceleration, maxOrientationJerk);

      return gains;
   }

   @Override
   public PIDSE3Gains getHoldPositionFootControlGains()
   {
      boolean realRobot = target == RobotTarget.REAL_ROBOT;

      double kpXY = 100.0;
      double kpZ = 0.0;
      double zetaXYZ = realRobot ? 0.2 : 1.0;
      double kpXYOrientation = realRobot ? 100.0 : 175.0;
      double kpZOrientation = realRobot ? 100.0 : 200.0;
      double zetaOrientation = realRobot ? 0.2 : 1.0;
      // Reduce maxPositionAcceleration from 10 to 6 to prevent too high acceleration when hitting joint limits.
      double maxLinearAcceleration = realRobot ? 6.0 : Double.POSITIVE_INFINITY;
//      double maxLinearAcceleration = realRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxLinearJerk = realRobot ? 150.0 : Double.POSITIVE_INFINITY;
      double maxAngularAcceleration = realRobot ? 100.0 : Double.POSITIVE_INFINITY;
      double maxAngularJerk = realRobot ? 1500.0 : Double.POSITIVE_INFINITY;

      DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains(GainCoupling.XY, false);
      gains.setPositionProportionalGains(kpXY, kpXY, kpZ);
      gains.setPositionDampingRatios(zetaXYZ);
      gains.setPositionMaxFeedbackAndFeedbackRate(maxLinearAcceleration, maxLinearJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatios(zetaOrientation);
      gains.setOrientationMaxFeedbackAndFeedbackRate(maxAngularAcceleration, maxAngularJerk);

      return gains;
   }

   @Override
   public PIDSE3Gains getToeOffFootControlGains()
   {
      boolean realRobot = target == RobotTarget.REAL_ROBOT;

      double kpXY = 100.0;
      double kpZ = 0.0;
      double zetaXYZ = realRobot ? 0.4 : 0.4;
      double kpXYOrientation = realRobot ? 200.0 : 200.0;
      double kpZOrientation = realRobot ? 200.0 : 200.0;
      double zetaOrientation = realRobot ? 0.4 : 0.4;
      // Reduce maxPositionAcceleration from 10 to 6 to prevent too high acceleration when hitting joint limits.
      double maxLinearAcceleration = realRobot ? 6.0 : Double.POSITIVE_INFINITY;
//      double maxLinearAcceleration = realRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxLinearJerk = realRobot ? 150.0 : Double.POSITIVE_INFINITY;
      double maxAngularAcceleration = realRobot ? 100.0 : Double.POSITIVE_INFINITY;
      double maxAngularJerk = realRobot ? 1500.0 : Double.POSITIVE_INFINITY;

      DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains(GainCoupling.XY, false);
      gains.setPositionProportionalGains(kpXY, kpXY, kpZ);
      gains.setPositionDampingRatios(zetaXYZ);
      gains.setPositionMaxFeedbackAndFeedbackRate(maxLinearAcceleration, maxLinearJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatios(zetaOrientation);
      gains.setOrientationMaxFeedbackAndFeedbackRate(maxAngularAcceleration, maxAngularJerk);

      return gains;
   }

   public double getSwingMaxHeightForPushRecoveryTrajectory()
   {
      return 0.15;
   }

   @Override
   public double getDefaultTransferTime()
   {
      return (target == RobotTarget.REAL_ROBOT) ? 0.8 : 0.25;
   }

   @Override
   public double getDefaultSwingTime()
   {
      return (target == RobotTarget.REAL_ROBOT) ? 1.2 : 0.60;
   }

   /** @inheritDoc */
   @Override
   public double getDefaultInitialTransferTime()
   {
      return (target == RobotTarget.REAL_ROBOT) ? 2.0 : 1.0;
   }

   @Override
   public double getContactThresholdForce()
   {
      switch (target)
      {
         case REAL_ROBOT :
            return 150.0;

         case GAZEBO :
            return 20.0;

         case SCS:
            return 20.0;

         default :
            throw new RuntimeException();
      }
   }

   @Override
   public double getSecondContactThresholdForceIgnoringCoP()
   {
      return 220.0;
   }

   @Override
   public double getCoPThresholdFraction()
   {
      return 0.02;
   }

   @Override
   public String[] getJointsToIgnoreInController()
   {
      return new String[0];
   }

   @Override
   public MomentumOptimizationSettings getMomentumOptimizationSettings()
   {
      return new ThorMomentumOptimizationSettings(jointMap);
   }

   @Override
   public ICPAngularMomentumModifierParameters getICPAngularMomentumModifierParameters()
   {
      return new ICPAngularMomentumModifierParameters();
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
   public double getMaxICPErrorBeforeSingleSupportX()
   {
      return 0.035;
   }

   @Override
   public double getMaxICPErrorBeforeSingleSupportY()
   {
      return 0.015;
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
      return -1.0; // 0.5;
   }

   /** {@inheritDoc} */
   @Override
   public double getCoPErrorThresholdForHighCoPDamping()
   {
      return Double.POSITIVE_INFINITY; //0.075;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxAllowedDistanceCMPSupport()
   {
      return 0.06;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useCenterOfMassVelocityFromEstimator()
   {
      return false;
   }

   /** {@inheritDoc} */
   @Override // // FIXME: 11/19/16  should be updated
   public String[] getJointsWithRestrictiveLimits()
   {
      String bkyName = jointMap.getSpineJointName(SpineJointName.SPINE_PITCH);
      String[] joints = {bkyName};
      return joints;
   }

   /** {@inheritDoc} */
   @Override
   public JointLimitParameters getJointLimitParametersForJointsWithRestictiveLimits()
   {
      JointLimitParameters parameters = new JointLimitParameters();
      parameters.setMaxAbsJointVelocity(9.0);
      parameters.setJointLimitDistanceForMaxVelocity(30.0 * Math.PI/180.0);
      parameters.setJointLimitFilterBreakFrequency(15.0);
      parameters.setVelocityControlGain(30.0);
      return parameters;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useOptimizationBasedICPController()
   {
      return false;
   }

   /** {@inheritDoc} */
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

   /** {@inheritDoc} */
   @Override
   public SteppingParameters getSteppingParameters()
   {
      return steppingParameters;
   }
}

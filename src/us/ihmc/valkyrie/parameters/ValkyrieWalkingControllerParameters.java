package us.ihmc.valkyrie.parameters;

import java.util.LinkedHashMap;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.SdfLoader.partNames.NeckJointName;
import us.ihmc.SdfLoader.partNames.SpineJointName;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.YoFootOrientationGains;
import us.ihmc.commonWalkingControlModules.controlModules.foot.YoFootSE3Gains;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.robotics.controllers.YoOrientationPIDGains;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.controllers.YoSE3PIDGains;
import us.ihmc.robotics.controllers.YoSymmetricSE3PIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.valkyrie.configuration.ValkyrieSliderBoardControlledNeckJoints;
import us.ihmc.valkyrie.fingers.ValkyrieFingerJointLimits;
import us.ihmc.valkyrie.fingers.ValkyrieRealRobotFingerJoint;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class ValkyrieWalkingControllerParameters implements WalkingControllerParameters
{
   private final DRCRobotModel.RobotTarget target;

   private final SideDependentList<RigidBodyTransform> handPosesWithRespectToChestFrame = new SideDependentList<RigidBodyTransform>();

   private final double upperNeckExtensorUpperLimit = 0.0872665;
   private final double upperNeckExtensorLowerLimit = -0.785398;
   private final double LowerNeckPitchUpperLimit = 1.162;
   private final double LowerNeckPitchLowerLimit = 0;

   private final DRCRobotJointMap jointMap;

   private final LinkedHashMap<NeckJointName, ImmutablePair<Double, Double>> sliderBoardControlledNeckJointNamesWithLimits = new LinkedHashMap<NeckJointName, ImmutablePair<Double, Double>>();
   private final SideDependentList<LinkedHashMap<String, ImmutablePair<Double, Double>>> sliderBoardControlledFingerJointNamesWithLimits = new SideDependentList<LinkedHashMap<String, ImmutablePair<Double, Double>>>();

   public ValkyrieWalkingControllerParameters(DRCRobotJointMap jointMap)
   {
      this(jointMap, DRCRobotModel.RobotTarget.SCS);
   }

   public ValkyrieWalkingControllerParameters(DRCRobotJointMap jointMap, DRCRobotModel.RobotTarget target)
   {
      this.jointMap = jointMap;
      this.target = target;

      // Genreated using ValkyrieFullRobotModelVisualizer
      RigidBodyTransform leftHandLocation = new RigidBodyTransform(new double[] { 0.8772111323383822, -0.47056204413925823, 0.09524700476706424,
            0.11738015536007923, 1.5892231999088989E-4, 0.1986725292086453, 0.980065916600275, 0.3166524835978034, -0.48010478444326166, -0.8597095955922112,
            0.1743525371234003, -0.13686311108389013, 0.0, 0.0, 0.0, 1.0 });

      RigidBodyTransform rightHandLocation = new RigidBodyTransform(new double[] { 0.8772107606751612, -0.47056267784177724, -0.09524729695945025,
            0.11738015535642271, -1.5509783447718197E-4, -0.19866600827375044, 0.9800672390715021, -0.3166524835989298, -0.48010546476828164,
            -0.8597107556492186, -0.17434494349043353, -0.13686311108617974, 0.0, 0.0, 0.0, 1.0 });

      handPosesWithRespectToChestFrame.put(RobotSide.LEFT, leftHandLocation);
      handPosesWithRespectToChestFrame.put(RobotSide.RIGHT, rightHandLocation);
      
      for(RobotSide side : RobotSide.values())
      {
         sliderBoardControlledFingerJointNamesWithLimits.put(side, new LinkedHashMap<String, ImmutablePair<Double,Double>>());
         for(ValkyrieRealRobotFingerJoint fingerJoint : ValkyrieRealRobotFingerJoint.values())
         {
            sliderBoardControlledFingerJointNamesWithLimits.get(side).put(side.getCamelCaseNameForStartOfExpression() + fingerJoint.toString(),
                  new ImmutablePair<Double,Double>(ValkyrieFingerJointLimits.getFullyExtensonPositionLimit(side, fingerJoint), ValkyrieFingerJointLimits.getFullyFlexedPositionLimit(side, fingerJoint)));
         }
      }
      
      NeckJointName[] sliderBoardControlledNeckJointNames = ValkyrieSliderBoardControlledNeckJoints.getNeckJointsControlledBySliderBoard();
      
      for (int i = 0; i < sliderBoardControlledNeckJointNames.length; i++)
      {
         NeckJointName joint = sliderBoardControlledNeckJointNames[i];
         
         sliderBoardControlledNeckJointNamesWithLimits.put(
               joint,
               new ImmutablePair<Double, Double>(ValkyrieSliderBoardControlledNeckJoints.getFullyExtendedPositionLimit(joint), ValkyrieSliderBoardControlledNeckJoints
                     .getFullyFlexedPositionLimit(joint)));
      }
   }

   @Override
   public double getOmega0()
   {
      // TODO probably need to be tuned.
      return target == DRCRobotModel.RobotTarget.REAL_ROBOT ? 3.4 : 3.3; // 3.3 seems more appropriate.
   }

   @Override
   public SideDependentList<RigidBodyTransform> getDesiredHandPosesWithRespectToChestFrame()
   {
      return handPosesWithRespectToChestFrame;
   }

   @Override
   public double getTimeToGetPreparedForLocomotion()
   {
      return 0.0;
   }

   @Override
   public boolean doToeOffIfPossible()
   {
      return !(target == DRCRobotModel.RobotTarget.REAL_ROBOT);
   }

   @Override
   public boolean checkTrailingLegJacobianDeterminantToTriggerToeOff()
   {
      return true;
   }

   @Override
   public boolean checkECMPLocationToTriggerToeOff()
   {
      return true;
   }

   @Override
   public double getMinStepLengthForToeOff()
   {
      return getFootLength();
   }

   /**
    * To enable that feature, doToeOffIfPossible() return true is required.
    */
   @Override
   public boolean doToeOffWhenHittingAnkleLimit()
   {
      return false;
   }

   @Override
   public double getMaximumToeOffAngle()
   {
      return Math.toRadians(30.0);
   }

   @Override
   public boolean doToeTouchdownIfPossible()
   {
      return false;
   }

   @Override
   public double getToeTouchdownAngle()
   {
      return Math.toRadians(20.0);
   }

   @Override
   public boolean doHeelTouchdownIfPossible()
   {
      return false;
   }

   @Override
   public double getHeelTouchdownAngle()
   {
      return Math.toRadians(-20.0);
   }

   @Override
   public boolean allowShrinkingSingleSupportFootPolygon()
   {
      return false;
   }

   @Override
   public boolean allowDisturbanceRecoveryBySpeedingUpSwing()
   {
      return false;
   }

   @Override
   public boolean allowAutomaticManipulationAbort()
   {
      return false;
   }

   @Override
   public double getICPErrorThresholdToSpeedUpSwing()
   {
      return Double.POSITIVE_INFINITY;
   }

   @Override
   public double getMinimumSwingTimeForDisturbanceRecovery()
   {
      return getDefaultSwingTime();
   }

   @Override
   public boolean isNeckPositionControlled()
   {
      return target == DRCRobotModel.RobotTarget.REAL_ROBOT;
   }

   @Override
   public LinkedHashMap<NeckJointName, ImmutablePair<Double, Double>> getSliderBoardControlledNeckJointsWithLimits()
   {
      return sliderBoardControlledNeckJointNamesWithLimits;
   }
   
   @Override
   public SideDependentList<LinkedHashMap<String, ImmutablePair<Double, Double>>> getSliderBoardControlledFingerJointsWithLimits()
   {
      return sliderBoardControlledFingerJointNamesWithLimits;
   }

   @Override
   public String[] getDefaultHeadOrientationControlJointNames()
   {
      if (controlHeadAndHandsWithSliders())
      {
         // For sliders, return none of them, to make sure that the QP whole body controller doesn't control the neck.
         return new String[]{};
      }
      
      else if (target == DRCRobotModel.RobotTarget.REAL_ROBOT)
      {
         // On the real robot, return all 3 so it knows to use them all. The real robot will use position control.
         return new String[] {jointMap.getNeckJointName(NeckJointName.UPPER_NECK_PITCH), jointMap.getNeckJointName(NeckJointName.LOWER_NECK_PITCH), jointMap.getNeckJointName(NeckJointName.NECK_YAW)};
      }
      
      // For sims using the QP and whole body controller, only allow one neck joint for now since the QP and inverse dynamics
      // don't do well with redundant joints yet. We'll have to fix that later some how.
      else return new String[] {jointMap.getNeckJointName(NeckJointName.LOWER_NECK_PITCH), jointMap.getNeckJointName(NeckJointName.NECK_YAW)};
   }

   @Override
   public String[] getDefaultChestOrientationControlJointNames()
   {
      String[] defaultChestOrientationControlJointNames = new String[] { jointMap.getSpineJointName(SpineJointName.SPINE_YAW),
            jointMap.getSpineJointName(SpineJointName.SPINE_PITCH), jointMap.getSpineJointName(SpineJointName.SPINE_ROLL) };

      return defaultChestOrientationControlJointNames;
   }

   @Override
   public double getNeckPitchUpperLimit()
   {
      return upperNeckExtensorUpperLimit + LowerNeckPitchUpperLimit; // 1.14494;
   }

   @Override
   public double getNeckPitchLowerLimit()
   {
      return upperNeckExtensorLowerLimit + LowerNeckPitchLowerLimit; // -0.602139;
   }

   @Override
   public double getHeadYawLimit()
   {
      return Math.PI / 4.0;
   }

   @Override
   public double getHeadRollLimit()
   {
      return Math.PI / 4.0;
   }

   @Override
   public boolean controlHeadAndHandsWithSliders()
   {
      return false;
   }

   // USE THESE FOR Real Robot and sims when controlling pelvis height instead of CoM.
   private final double minimumHeightAboveGround = 0.595 + 0.23;
   private double nominalHeightAboveGround = 0.675 + 0.23 - 0.01;
   private final double maximumHeightAboveGround = 0.735 + 0.23;

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
   public double getFootForwardOffset()
   {
      return ValkyriePhysicalProperties.footForward;
   }

   @Override
   public double getFootBackwardOffset()
   {
      return ValkyriePhysicalProperties.footBack;
   }

   @Override
   public double getAnkleHeight()
   {
      return ValkyriePhysicalProperties.ankleHeight;
   }

   @Override
   public double getLegLength()
   {
      return ValkyriePhysicalProperties.thighLength + ValkyriePhysicalProperties.shinLength;
   }

   @Override
   public double getMinLegLengthBeforeCollapsingSingleSupport()
   {
      // TODO: Useful values
      return 0.1;
   }

   @Override
   public double getMinMechanicalLegLength()
   {
      return 0.1;
   }

   @Override
   public double getSwingHeightMaxForPushRecoveryTrajectory()
   {
      return 0.1;
   }

   @Override
   public double getInPlaceWidth()
   {
      return 0.25;
   }

   @Override
   public double getDesiredStepForward()
   {
      return 0.5; // 0.35;
   }

   @Override
   public double getMaxStepLength()
   {
      if (target == DRCRobotModel.RobotTarget.SCS)
         return 0.6; // 0.5; //0.35;

      return 0.4;
   }

   @Override
   public double getDefaultStepLength()
   {
      return 0.5;
   }

   @Override
   public double getMinStepWidth()
   {
      return (target == DRCRobotModel.RobotTarget.REAL_ROBOT) ? 0.165 : 0.15;
   }

   @Override
   public double getMaxStepWidth()
   {
      return 0.6; // 0.4;
   }

   @Override
   public double getStepPitch()
   {
      return 0.0;
   }

   @Override
   public double getMaxStepUp()
   {
      return 0.3;
   }

   @Override
   public double getMaxStepDown()
   {
      return 0.25;
   }

   @Override
   public double getMaxSwingHeightFromStanceFoot()
   {
      return 0.3;
   }

   @Override
   public double getMaxAngleTurnOutwards()
   {
      return Math.PI / 4.0;
   }

   @Override
   public double getMaxAngleTurnInwards()
   {
      return 0;
   }

   @Override
   public double getMinAreaPercentForValidFootstep()
   {
      return 0.5;
   }

   @Override
   public double getDangerAreaPercentForValidFootstep()
   {
      return 0.75;
   }

   @Override
   public ICPControlGains getICPControlGains()
   {
      ICPControlGains gains = new ICPControlGains();

      boolean runningOnRealRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;

      double kpOrthogonal = runningOnRealRobot ? 1.9 : 1.5;
      double kpParallel = runningOnRealRobot ? 2.0 : 1.5;
      double ki = runningOnRealRobot ? 0.0 : 0.0;
      double kiBleedOff = 0.9;
      boolean useRawCMP = true;
      double cmpFilterBreakFrequencyInHertz = 16.0;
      double cmpRateLimit = runningOnRealRobot ? 6.0 : 60.0;
      double cmpAccelerationLimit = runningOnRealRobot ? 200.0 : 2000.0;

      gains.setKpParallelToMotion(kpParallel);
      gains.setKpOrthogonalToMotion(kpOrthogonal);
      gains.setKi(ki);
      gains.setKiBleedOff(kiBleedOff);
      gains.setUseRawCMP(useRawCMP);
      gains.setCMPFilterBreakFrequencyInHertz(cmpFilterBreakFrequencyInHertz);
      gains.setCMPRateLimit(cmpRateLimit);
      gains.setCMPAccelerationLimit(cmpAccelerationLimit);

      return gains;
   }

   @Override
   public YoPDGains createCoMHeightControlGains(YoVariableRegistry registry)
   {
      YoPDGains gains = new YoPDGains("CoMHeight", registry);

      boolean runningOnRealRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;

      double kp = runningOnRealRobot ? 40.0 : 50.0;
      double zeta = runningOnRealRobot ? 0.4 : 1.0;
      double maxAcceleration = 0.5 * 9.81;
      double maxJerk = maxAcceleration / 0.05;

      gains.setKp(kp);
      gains.setZeta(zeta);
      gains.setMaximumAcceleration(maxAcceleration);
      gains.setMaximumJerk(maxJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   @Override
   public boolean getCoMHeightDriftCompensation()
   {
      return true;
   }

   @Override
   public YoPDGains createPelvisICPBasedXYControlGains(YoVariableRegistry registry)
   {
      YoPDGains gains = new YoPDGains("pelvisXY", registry);

      boolean runningOnRealRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;

      gains.setKp(4.0);
      gains.setKd(runningOnRealRobot ? 0.5 : 1.2);

      return gains;
   }

   @Override
   public YoOrientationPIDGains createPelvisOrientationControlGains(YoVariableRegistry registry)
   {
      YoFootOrientationGains gains = new YoFootOrientationGains("pelvisOrientation", registry);

      boolean runningOnRealRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;

      double kpXY = runningOnRealRobot ? 160.0 : 100.0; // 120.0
      double kpZ = runningOnRealRobot ? 120.0 : 100.0; // 120.0
      double zetaXY = runningOnRealRobot ? 0.5 : 0.8; // 0.7
      double zetaZ = runningOnRealRobot ? 0.5 : 0.8; // 0.7
      double maxAccel = runningOnRealRobot ? 18.0 : 18.0;
      double maxJerk = runningOnRealRobot ? 270.0 : 270.0;

      gains.setProportionalGains(kpXY, kpZ);
      gains.setDampingRatios(zetaXY, zetaZ);
      gains.setMaximumAcceleration(maxAccel);
      gains.setMaximumJerk(maxJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   @Override
   public YoOrientationPIDGains createHeadOrientationControlGains(YoVariableRegistry registry)
   {
      YoSymmetricSE3PIDGains gains = new YoSymmetricSE3PIDGains("HeadOrientation", registry);

      boolean runningOnRealRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;

      double kp = 6.5;//40.0;
      double zeta = runningOnRealRobot ? 0.4 : 0.8;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = 18.0;
      double maxJerk = 270.0;

      gains.setProportionalGain(kp);
      gains.setDampingRatio(zeta);
      gains.setIntegralGain(ki);
      gains.setMaximumIntegralError(maxIntegralError);
      gains.setMaximumAcceleration(maxAccel);
      gains.setMaximumJerk(maxJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   @Override
   public double getTrajectoryTimeHeadOrientation()
   {
      return 2.0;
   }

   @Override
   public double[] getInitialHeadYawPitchRoll()
   {
      return new double[] { 0.0, 0.0, 0.0 };//{ 0.0, 0.67, 0.0 };
   }

   @Override
   public YoPDGains createUnconstrainedJointsControlGains(YoVariableRegistry registry)
   {
      YoPDGains gains = new YoPDGains("UnconstrainedJoints", registry);

      boolean runningOnRealRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;

      double kp = runningOnRealRobot ? 80.0 : 100.0;
      double zeta = runningOnRealRobot ? 0.6 : 0.8;
      double maxAcceleration = runningOnRealRobot ? 18.0 : 18.0;
      double maxJerk = runningOnRealRobot ? 270.0 : 270.0;

      gains.setKp(kp);
      gains.setZeta(zeta);
      gains.setMaximumAcceleration(maxAcceleration);
      gains.setMaximumJerk(maxJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   @Override
   public YoOrientationPIDGains createChestControlGains(YoVariableRegistry registry)
   {
      YoFootOrientationGains gains = new YoFootOrientationGains("ChestOrientation", registry);

      boolean runningOnRealRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;

      double kpXY = runningOnRealRobot ? 80.0 : 100.0; // 60.0
      double kpZ = runningOnRealRobot ? 40.0 : 100.0; // 60.0
      double zetaXY = runningOnRealRobot ? 0.7 : 0.8; // 0.4
      double zetaZ = runningOnRealRobot ? 0.5 : 0.8; // 0.4
      double maxAccel = runningOnRealRobot ? 12.0 : 18.0;
      double maxJerk = runningOnRealRobot ? 180.0 : 270.0;

      gains.setProportionalGains(kpXY, kpZ);
      gains.setDampingRatios(zetaXY, zetaZ);
      gains.setMaximumAcceleration(maxAccel);
      gains.setMaximumJerk(maxJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   @Override
   public YoSE3PIDGains createSwingFootControlGains(YoVariableRegistry registry)
   {
      YoFootSE3Gains gains = new YoFootSE3Gains("SwingFoot", registry);

      boolean runningOnRealRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;

      double kpXY = 100.0; // 150.0
      double kpZ = runningOnRealRobot ? 200.0 : 200.0;
      double zetaXYZ = runningOnRealRobot ? 0.5 : 0.7;
      double kpXYOrientation = runningOnRealRobot ? 200.0 : 300.0;
      double kpZOrientation = runningOnRealRobot ? 150.0 : 200.0; // 160
      double zetaOrientationXY = runningOnRealRobot ? 0.7 : 0.7;
      double zetaOrientationZ = runningOnRealRobot ? 0.5 : 0.7;
      double maxLinearAcceleration = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxLinearJerk = runningOnRealRobot ? 150.0 : Double.POSITIVE_INFINITY;
      double maxAngularAcceleration = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;
      double maxAngularJerk = runningOnRealRobot ? 1500.0 : Double.POSITIVE_INFINITY;

      gains.setPositionProportionalGains(kpXY, kpZ);
      gains.setPositionDampingRatio(zetaXYZ);
      gains.setPositionMaxAccelerationAndJerk(maxLinearAcceleration, maxLinearJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatios(zetaOrientationXY, zetaOrientationZ);
      gains.setOrientationMaxAccelerationAndJerk(maxAngularAcceleration, maxAngularJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   @Override
   public YoSE3PIDGains createHoldPositionFootControlGains(YoVariableRegistry registry)
   {
      YoFootSE3Gains gains = new YoFootSE3Gains("HoldFoot", registry);

      boolean runningOnRealRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;

      double kpXY = 40.0;
      double kpZ = 0.0;
      double zetaXYZ = runningOnRealRobot ? 0.7 : 1.0;
      double kpXYOrientation = runningOnRealRobot ? 40.0 : 200.0;
      double kpZOrientation = runningOnRealRobot ? 120.0 : 200.0;
      double zetaOrientation = runningOnRealRobot ? 0.7 : 1.0;
      double maxLinearAcceleration = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxLinearJerk = runningOnRealRobot ? 150.0 : Double.POSITIVE_INFINITY;
      double maxAngularAcceleration = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;
      double maxAngularJerk = runningOnRealRobot ? 1500.0 : Double.POSITIVE_INFINITY;

      gains.setPositionProportionalGains(kpXY, kpZ);
      gains.setPositionDampingRatio(zetaXYZ);
      gains.setPositionMaxAccelerationAndJerk(maxLinearAcceleration, maxLinearJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatio(zetaOrientation);
      gains.setOrientationMaxAccelerationAndJerk(maxAngularAcceleration, maxAngularJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   @Override
   public YoSE3PIDGains createToeOffFootControlGains(YoVariableRegistry registry)
   {
      YoFootSE3Gains gains = new YoFootSE3Gains("ToeOffFoot", registry);

      boolean runningOnRealRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;

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

      gains.setPositionProportionalGains(kpXY, kpZ);
      gains.setPositionDampingRatio(zetaXYZ);
      gains.setPositionMaxAccelerationAndJerk(maxLinearAcceleration, maxLinearJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatio(zetaOrientation);
      gains.setOrientationMaxAccelerationAndJerk(maxAngularAcceleration, maxAngularJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   @Override
   public YoSE3PIDGains createEdgeTouchdownFootControlGains(YoVariableRegistry registry)
   {
      YoFootSE3Gains gains = new YoFootSE3Gains("EdgeTouchdownFoot", registry);

      boolean runningOnRealRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;

      double kp = 0.0;
      double zetaXYZ = runningOnRealRobot ? 0.0 : 0.0;
      double kpXYOrientation = runningOnRealRobot ? 40.0 : 300.0;
      double kpZOrientation = runningOnRealRobot ? 40.0 : 300.0;
      double zetaOrientation = runningOnRealRobot ? 0.4 : 0.4;
      double maxLinearAcceleration = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxLinearJerk = runningOnRealRobot ? 150.0 : Double.POSITIVE_INFINITY;
      double maxAngularAcceleration = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;
      double maxAngularJerk = runningOnRealRobot ? 1500.0 : Double.POSITIVE_INFINITY;

      gains.setPositionProportionalGains(kp, kp);
      gains.setPositionDampingRatio(zetaXYZ);
      gains.setPositionMaxAccelerationAndJerk(maxLinearAcceleration, maxLinearJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatio(zetaOrientation);
      gains.setOrientationMaxAccelerationAndJerk(maxAngularAcceleration, maxAngularJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   @Override
   public double getSupportSingularityEscapeMultiplier()
   {
      return -30; // negative as knee axis are -y direction
   }

   @Override
   public double getSwingSingularityEscapeMultiplier()
   {
      return -(target == DRCRobotModel.RobotTarget.REAL_ROBOT ? 30.0 : 200.0); // negative as knee axis are -y direction
   }

   @Override
   public boolean doPrepareManipulationForLocomotion()
   {
      return true;
   }

   @Override
   public double getDefaultTransferTime()
   {
      return target == DRCRobotModel.RobotTarget.REAL_ROBOT ? 2.0 : 0.25;
   }

   @Override
   public double getDefaultSwingTime()
   {
      return target == DRCRobotModel.RobotTarget.REAL_ROBOT ? 1.5 : 0.60;
   }

   /** @inheritDoc */
   @Override
   public double getSpineYawLimit()
   {
      return Math.PI / 4.0;
   }

   /** @inheritDoc */
   @Override
   public double getSpinePitchUpperLimit()
   {
      return -0.13;
   }

   /** @inheritDoc */
   @Override
   public double getSpinePitchLowerLimit()
   {
      return 0.666;
   }

   /** @inheritDoc */
   @Override
   public double getSpineRollLimit()
   {
      return Math.PI / 4.0;
   }

   /** @inheritDoc */
   @Override
   public boolean isSpinePitchReversed()
   {
      return true;
   }

   @Override
   public double getFootWidth()
   {
      return ValkyriePhysicalProperties.footWidth;
   }

   @Override
   public double getToeWidth()
   {
      return ValkyriePhysicalProperties.footWidth;
   }

   @Override
   public double getFootLength()
   {
      return ValkyriePhysicalProperties.footLength;
   }

   @Override
   public double getActualFootWidth()
   {
      return ValkyriePhysicalProperties.footWidth;
   }

   @Override
   public double getActualFootLength()
   {
      return ValkyriePhysicalProperties.footLength;
   }

   @Override
   public double getFootstepArea()
   {
      return (getToeWidth() + getFootWidth()) * getFootLength() / 2.0;
   }

   @Override
   public double getFoot_start_toetaper_from_back()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public double getSideLengthOfBoundingBoxForFootstepHeight()
   {
      return (1 + 0.3) * 2 * Math.sqrt(getFootForwardOffset() * getFootForwardOffset() + 0.25 * getFootWidth() * getFootWidth());
   }

   /** {@inheritDoc} */
   @Override
   public double getDesiredTouchdownHeightOffset()
   {
      return 0;
   }

   /** {@inheritDoc} */
   @Override
   public double getDesiredTouchdownVelocity()
   {
      return -0.3;
   }

   /** {@inheritDoc} */
   @Override
   public double getDesiredTouchdownAcceleration()
   {
      if(target == DRCRobotModel.RobotTarget.GAZEBO)
      {
         return -1.0;
      }
      else
      {
         return 0.0;
      }
   }

   @Override
   public double getContactThresholdForce()
   {
      switch(target)
      {
      case REAL_ROBOT:
      case GAZEBO:
         return 50.0;
      default:
         return 5.0;
      }
   }

   @Override
   public double getSecondContactThresholdForceIgnoringCoP()
   {
      return 75.0;
   }   

   @Override
   public double getCoPThresholdFraction()
   {
      return 0.01;
   }

   @Override
   public String[] getJointsToIgnoreInController()
   {
      String[] jointsToIgnore = new String[] {
            "rightIndexFingerPitch1",
            "rightIndexFingerPitch2",
            "rightIndexFingerPitch3",
            "rightMiddleFingerPitch1",
            "rightMiddleFingerPitch2",
            "rightMiddleFingerPitch3",
            "rightPinkyPitch1",
            "rightPinkyPitch2",
            "rightPinkyPitch3",
            "rightThumbRoll",
            "rightThumbPitch1",
            "rightThumbPitch2",
            "rightThumbPitch3",
            "leftIndexFingerPitch1",
            "leftIndexFingerPitch2",
            "leftIndexFingerPitch3",
            "leftMiddleFingerPitch1",
            "leftMiddleFingerPitch2",
            "leftMiddleFingerPitch3",
            "leftPinkyPitch1",
            "leftPinkyPitch2",
            "leftPinkyPitch3",
            "leftThumbRoll",
            "leftThumbPitch1",
            "leftThumbPitch2",
            "leftThumbPitch3",
//            "upperNeckPitch",
//            "LowerNeckPitch",
//            "NeckYaw"
             };
            
      return jointsToIgnore;
   }

   @Override
   public void setupMomentumOptimizationSettings(MomentumOptimizationSettings momentumOptimizationSettings)
   {
      momentumOptimizationSettings.setDampedLeastSquaresFactor(0.05);
      momentumOptimizationSettings.setRhoPlaneContactRegularization(0.001);
      momentumOptimizationSettings.setMomentumWeight(1.4, 1.4, 10.0, 10.0);
      momentumOptimizationSettings.setRhoMin(4.0);
      momentumOptimizationSettings.setRateOfChangeOfRhoPlaneContactRegularization(0.16); // 0.06);
      momentumOptimizationSettings.setRhoPenalizerPlaneContactRegularization(0.01);
   }

   @Override
   public boolean doFancyOnToesControl()
   {
      return !(target == DRCRobotModel.RobotTarget.REAL_ROBOT);
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
   public double getMaxICPErrorBeforeSingleSupportX()
   {
      return 0.02;
   }

   @Override
   public double getMaxICPErrorBeforeSingleSupportY()
   {
      return 0.02;
   }

   /** {@inheritDoc} */
   @Override
   public double getDurationToCancelOutDesiredICPVelocityWhenStuckInTransfer()
   {
      return Double.POSITIVE_INFINITY;
   }

   @Override
   public boolean finishSingleSupportWhenICPPlannerIsDone()
   {
      return false;
   }

   @Override
   public double minimumHeightBetweenAnkleAndPelvisForHeightAdjustment()
   {
      return 0.9649;
   }

   @Override
   public double nominalHeightBetweenAnkleAndPelvisForHeightAdjustment()
   {
      return 0.8949;
   }

   @Override
   public double maximumHeightBetweenAnkleAndPelvisForHeightAdjustment()
   {
      return 0.65;
   }

   @Override
   public double pelvisToAnkleThresholdForWalking()
   {
      return 0.8157;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useICPPlannerHackN13()
   {
      return true;
   }
}

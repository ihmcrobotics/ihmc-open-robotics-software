package us.ihmc.valkyrie.parameters;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.YoFootOrientationGains;
import us.ihmc.commonWalkingControlModules.controlModules.foot.YoFootSE3Gains;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.utilities.humanoidRobot.partNames.NeckJointName;
import us.ihmc.utilities.humanoidRobot.partNames.SpineJointName;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.yoUtilities.controllers.YoOrientationPIDGains;
import us.ihmc.yoUtilities.controllers.YoPDGains;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.controllers.YoSymmetricSE3PIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class ValkyrieWalkingControllerParameters implements WalkingControllerParameters
{
   private final boolean runningOnRealRobot;

   private final SideDependentList<RigidBodyTransform> handPosesWithRespectToChestFrame = new SideDependentList<RigidBodyTransform>();

   private final double upperNeckExtensorUpperLimit = 0.785398;
   private final double upperNeckExtensorLowerLimit = -0.0872665;
   private final double lowerNeckExtensorUpperLimit = 0.0;
   private final double lowerNeckExtensorLowerLimit = -1.5708;

   private final DRCRobotJointMap jointMap;

   public ValkyrieWalkingControllerParameters(DRCRobotJointMap jointMap)
   {
      this(jointMap, false);
   }

   public ValkyrieWalkingControllerParameters(DRCRobotJointMap jointMap, boolean runningOnRealRobot)
   {
      this.jointMap = jointMap;
      this.runningOnRealRobot = runningOnRealRobot;

      // Genreated using ValkyrieFullRobotModelVisualizer
      RigidBodyTransform leftHandLocation = new RigidBodyTransform(new double[]
      {
         0.8772111323383822, -0.47056204413925823, 0.09524700476706424, 0.11738015536007923, 1.5892231999088989E-4, 0.1986725292086453, 0.980065916600275,
         0.3166524835978034, -0.48010478444326166, -0.8597095955922112, 0.1743525371234003, -0.13686311108389013, 0.0, 0.0, 0.0, 1.0
      });

      RigidBodyTransform rightHandLocation = new RigidBodyTransform(new double[]
      {
         0.8772107606751612, -0.47056267784177724, -0.09524729695945025, 0.11738015535642271, -1.5509783447718197E-4, -0.19866600827375044, 0.9800672390715021,
         -0.3166524835989298, -0.48010546476828164, -0.8597107556492186, -0.17434494349043353, -0.13686311108617974, 0.0, 0.0, 0.0, 1.0
      });

      handPosesWithRespectToChestFrame.put(RobotSide.LEFT, leftHandLocation);
      handPosesWithRespectToChestFrame.put(RobotSide.RIGHT, rightHandLocation);
   }

   @Override
   public SideDependentList<RigidBodyTransform> getDesiredHandPosesWithRespectToChestFrame()
   {
      return handPosesWithRespectToChestFrame;
   }

   @Override
   public boolean doToeOffIfPossible()
   {
      return !runningOnRealRobot;
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
   public boolean isNeckPositionControlled()
   {
      return true;
   }

   @Override
   public String[] getDefaultHeadOrientationControlJointNames()
   {
      String[] defaultHeadOrientationControlJointNames = new String[]
      {
            jointMap.getNeckJointName(NeckJointName.UPPER_NECK_PITCH)
//            jointMap.getNeckJointName(NeckJointName.LOWER_NECK_PITCH),
//            jointMap.getNeckJointName(NeckJointName.NECK_YAW),
//            jointMap.getNeckJointName(NeckJointName.UPPER_NECK_PITCH)
      };

      return defaultHeadOrientationControlJointNames;
   }

   @Override
   public String[] getDefaultChestOrientationControlJointNames()
   {
      String[] defaultChestOrientationControlJointNames = new String[] {jointMap.getSpineJointName(SpineJointName.SPINE_YAW),
              jointMap.getSpineJointName(SpineJointName.SPINE_PITCH), jointMap.getSpineJointName(SpineJointName.SPINE_ROLL)};

      return defaultChestOrientationControlJointNames;
   }

   @Override
   public double getNeckPitchUpperLimit()
   {
      return upperNeckExtensorUpperLimit + lowerNeckExtensorUpperLimit;    // 1.14494;
   }

   @Override
   public double getNeckPitchLowerLimit()
   {
      return upperNeckExtensorLowerLimit + lowerNeckExtensorLowerLimit;    // -0.602139;
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

   // USE THESE FOR Real Atlas Robot and sims when controlling pelvis height instead of CoM.
   private final double minimumHeightAboveGround = 0.595 + 0.23;
   private double nominalHeightAboveGround = 0.675 + 0.23 - 0.01;
   private final double maximumHeightAboveGround = 0.735 + 0.23;

   // USE THESE FOR DRC Atlas Model TASK 2 UNTIL WALKING WORKS BETTER WITH OTHERS.
   // private final double minimumHeightAboveGround = 0.785;
   // private double nominalHeightAboveGround = 0.865;
   // private final double maximumHeightAboveGround = 0.925;

   //// USE THESE FOR VRC Atlas Model TASK 2 UNTIL WALKING WORKS BETTER WITH OTHERS.
   // private double minimumHeightAboveGround = 0.68;
   // private double nominalHeightAboveGround = 0.76;
   // private double maximumHeightAboveGround = 0.82;

   //// USE THESE FOR IMPROVING WALKING, BUT DONT CHECK THEM IN UNTIL IT IMPROVED WALKING THROUGH MUD.
   // private double minimumHeightAboveGround = 0.68;
   // private double nominalHeightAboveGround = 0.80;  // NOTE: used to be 0.76, jojo
   // private double maximumHeightAboveGround = 0.84;  // NOTE: used to be 0.82, jojo

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
      return 0.5;    // 0.35;
   }

   @Override
   public double getMaxStepLength()
   {
      if (!runningOnRealRobot)
         return 0.6;    // 0.5; //0.35;

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
      return 0.15;
   }

   @Override
   public double getMaxStepWidth()
   {
      return 0.6;    // 0.4;
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
   public double getCaptureKpParallelToMotion()
   {
      if (!runningOnRealRobot)
         return 1.4;

      return 1.4;
   }

   @Override
   public double getCaptureKpOrthogonalToMotion()
   {
      if (!runningOnRealRobot)
         return 1.4;

      return 1.4;
   }

   @Override
   public double getCaptureKi()
   {
      if (!runningOnRealRobot)
         return 4.0;

      return 0.0;
   }

   @Override
   public double getCaptureKiBleedoff()
   {
      return 0.9;
   }

   @Override
   public double getCaptureFilterBreakFrequencyInHz()
   {
      if (!runningOnRealRobot)
         return 16.0;

      return 16.0;    // 20.0;//16.0;
   }

   @Override
   public double getCMPRateLimit()
   {
      if (!runningOnRealRobot)
         return 60.0;

      return 6.0;    // 12.0;//60.0; //6.0;
   }

   @Override
   public double getCMPAccelerationLimit()
   {
      if (!runningOnRealRobot)
         return 2000.0;

      return 200.0;    // 400.0;//2000.0; //200.0;
   }

   @Override
   public YoPDGains createCoMHeightControlGains(YoVariableRegistry registry)
   {
      YoPDGains gains = new YoPDGains("CoMHeight", registry);

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
   public YoOrientationPIDGains createPelvisOrientationControlGains(YoVariableRegistry registry)
   {
      YoFootOrientationGains gains = new YoFootOrientationGains("PelvisOrientation", registry);

      double kpXY = runningOnRealRobot ? 160.0 : 100.0;    // 120.0
      double kpZ = runningOnRealRobot ? 120.0 : 100.0;    // 120.0
      double zetaXY = runningOnRealRobot ? 0.5 : 0.8;    // 0.7
      double zetaZ = runningOnRealRobot ? 0.5 : 0.8;    // 0.7
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

      double kp = 40.0;
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
      return new double[] {0.0, 0.67, 0.0};
   }

   @Override
   public YoPDGains createUnconstrainedJointsControlGains(YoVariableRegistry registry)
   {
      YoPDGains gains = new YoPDGains("UnconstrainedJoints", registry);

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

      double kpXY = runningOnRealRobot ? 80.0 : 100.0;    // 60.0
      double kpZ = runningOnRealRobot ? 40.0 : 100.0;    // 60.0
      double zetaXY = runningOnRealRobot ? 0.7 : 0.8;    // 0.4
      double zetaZ = runningOnRealRobot ? 0.5 : 0.8;    // 0.4
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

      double kpXY = 100.0;    // 150.0
      double kpZ = runningOnRealRobot ? 200.0 : 200.0;
      double zetaXYZ = runningOnRealRobot ? 0.5 : 0.7;
      double kpXYOrientation = runningOnRealRobot ? 200.0 : 300.0;
      double kpZOrientation = runningOnRealRobot ? 150.0 : 200.0;    // 160
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
   public YoSE3PIDGains createSupportFootControlGains(YoVariableRegistry registry)
   {
      return null;
   }

   @Override
   public double getSupportSingularityEscapeMultiplier()
   {
      return -30;    // negative as knee axis are -y direction
   }

   @Override
   public double getSwingSingularityEscapeMultiplier()
   {
      return -(runningOnRealRobot ? 30.0 : 200.0);    // negative as knee axis are -y direction
   }

   @Override
   public boolean doPrepareManipulationForLocomotion()
   {
      return true;
   }

   @Override
   public double getDefaultTransferTime()
   {
      return runningOnRealRobot ? 1.4 : 0.25;
   }

   @Override
   public double getDefaultSwingTime()
   {
      return runningOnRealRobot ? 1.4 : 0.60;
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
      return 0.0872665;
   }

   /** @inheritDoc */
   @Override
   public double getSpinePitchLowerLimit()
   {
      return -0.698132;
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
      return ValkyriePhysicalProperties.footBack + ValkyriePhysicalProperties.footForward;
   }

   @Override
   public double getActualFootWidth()
   {
      return getFootWidth();
   }

   @Override
   public double getActualFootLength()
   {
      return getFootLength();
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
      return 0;
   }

   @Override
   public double getContactThresholdForce()
   {
      return runningOnRealRobot ? 50.0 : 5.0;
   }

   @Override
   public double getCoPThresholdFraction()
   {
      return Double.NaN;
   }

   @Override
   public String[] getJointsToIgnoreInController()
   {
      String[] jointsToIgnore = new String[]
      {
         // jointMap.getNeckJointName(NeckJointName.LOWER_NECK_PITCH),
         // jointMap.getNeckJointName(NeckJointName.NECK_YAW),
         // jointMap.getNeckJointName(NeckJointName.UPPER_NECK_PITCH)
      };

      return jointsToIgnore;
   }

   @Override
   public void setupMomentumOptimizationSettings(MomentumOptimizationSettings momentumOptimizationSettings)
   {
      momentumOptimizationSettings.setDampedLeastSquaresFactor(0.05);
      momentumOptimizationSettings.setRhoPlaneContactRegularization(0.001);
      momentumOptimizationSettings.setMomentumWeight(1.0, 1.0, 10.0, 10.0);
      momentumOptimizationSettings.setRhoMin(4.0);
      momentumOptimizationSettings.setRateOfChangeOfRhoPlaneContactRegularization(0.12);    // 0.06);
      momentumOptimizationSettings.setRhoPenalizerPlaneContactRegularization(0.01);
   }

   @Override
   public boolean doFancyOnToesControl()
   {
      return !runningOnRealRobot;
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
   public double getMaxICPErrorBeforeSingleSupport()
   {
      return 0.035;
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
}

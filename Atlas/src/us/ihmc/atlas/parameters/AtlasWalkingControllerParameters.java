package us.ihmc.atlas.parameters;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel.AtlasTarget;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.YoFootOrientationGains;
import us.ihmc.commonWalkingControlModules.controlModules.foot.YoFootSE3Gains;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.utilities.humanoidRobot.partNames.NeckJointName;
import us.ihmc.utilities.humanoidRobot.partNames.SpineJointName;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.yoUtilities.controllers.YoIndependentSE3PIDGains;
import us.ihmc.yoUtilities.controllers.YoOrientationPIDGains;
import us.ihmc.yoUtilities.controllers.YoPDGains;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.controllers.YoSymmetricSE3PIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;



public class AtlasWalkingControllerParameters implements WalkingControllerParameters
{
   private final AtlasTarget target;
   private final SideDependentList<RigidBodyTransform> handPosesWithRespectToChestFrame = new SideDependentList<RigidBodyTransform>();

   // Limits
   private final double neckPitchUpperLimit = 1.14494;    // 0.83;    // true limit is = 1.134460, but pitching down more just looks at more robot chest
   private final double neckPitchLowerLimit = -0.602139;    // -0.610865;    // -math.pi/2.0;
   private final double headYawLimit = Math.PI / 4.0;
   private final double headRollLimit = Math.PI / 4.0;
   private final double spineYawLimit = Math.PI / 4.0;
   private final double spinePitchUpperLimit = 0.4;
   private final double spinePitchLowerLimit = -0.1;    // -math.pi / 6.0;
   private final double spineRollLimit = Math.PI / 4.0;

   private final double min_leg_length_before_collapsing_single_support = 0.53;    // corresponds to q_kny = 1.70 rad
   private final double min_mechanical_leg_length = 0.420;    // corresponds to a q_kny that is close to knee limit

   private final AtlasJointMap jointMap;

   public AtlasWalkingControllerParameters(AtlasJointMap jointMap)
   {
      this(AtlasTarget.SIM, jointMap);
   }

   public AtlasWalkingControllerParameters(AtlasTarget target, AtlasJointMap jointMap)
   {
      this.target = target;
      this.jointMap = jointMap;

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyTransform transform = new RigidBodyTransform();

         double x = 0.20;
         double y = robotSide.negateIfRightSide(0.35);    // 0.30);
         double z = -0.40;
         transform.setTranslation(new Vector3d(x, y, z));

         Matrix3d rotation = new Matrix3d();
         double yaw = 0.0;    // robotSide.negateIfRightSide(-1.7);
         double pitch = 0.7;
         double roll = 0.0;    // robotSide.negateIfRightSide(-0.8);
         RotationFunctions.setYawPitchRoll(rotation, yaw, pitch, roll);
         transform.setRotation(rotation);

         handPosesWithRespectToChestFrame.put(robotSide, transform);
      }
   }

   @Override
   public double getTimeToGetPreparedForLocomotion()
   {
      return 0.0; // 0.3 seems to be a good starting point
   }

   @Override
   public boolean doToeOffIfPossible()
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
    * To enable that feature, doToeOffIfPossible() return true is required. John parameter
    */
   @Override
   public boolean doToeOffWhenHittingAnkleLimit()
   {
      return true;
   }

   @Override
   public double getMaximumToeOffAngle()
   {
      return Math.toRadians(45.0);
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

   public boolean isNeckPositionControlled()
   {
      if (target == AtlasTarget.REAL_ROBOT)
         return true;
      else
         return false;
   }

   @Override
   public String[] getDefaultHeadOrientationControlJointNames()
   {
         return new String[] {jointMap.getNeckJointName(NeckJointName.LOWER_NECK_PITCH)};
   }

   @Override
   public String[] getDefaultChestOrientationControlJointNames()
   {
      String[] defaultChestOrientationControlJointNames = new String[] {jointMap.getSpineJointName(SpineJointName.SPINE_YAW),
              jointMap.getSpineJointName(SpineJointName.SPINE_PITCH), jointMap.getSpineJointName(SpineJointName.SPINE_ROLL)};

      return defaultChestOrientationControlJointNames;
   }

// USE THESE FOR Real Atlas Robot and sims when controlling pelvis height instead of CoM.
   private final double minimumHeightAboveGround = 0.595 + 0.03;
   private double nominalHeightAboveGround = 0.675 + 0.03;
   private final double maximumHeightAboveGround = 0.735 + 0.03;

// USE THESE FOR DRC Atlas Model TASK 2 UNTIL WALKING WORKS BETTER WITH OTHERS.
//   private final double minimumHeightAboveGround = 0.785;                                       
//   private double nominalHeightAboveGround = 0.865; 
//   private final double maximumHeightAboveGround = 0.925; 

//   // USE THESE FOR VRC Atlas Model TASK 2 UNTIL WALKING WORKS BETTER WITH OTHERS.
//   private double minimumHeightAboveGround = 0.68;                                       
//   private double nominalHeightAboveGround = 0.76; 
//   private double maximumHeightAboveGround = 0.82; 

//   // USE THESE FOR IMPROVING WALKING, BUT DONT CHECK THEM IN UNTIL IT IMPROVED WALKING THROUGH MUD.
//   private double minimumHeightAboveGround = 0.68;                                       
//   private double nominalHeightAboveGround = 0.80;  // NOTE: used to be 0.76, jojo        
//   private double maximumHeightAboveGround = 0.84;  // NOTE: used to be 0.82, jojo        

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

   public void setNominalHeightAboveAnkle(double nominalHeightAboveAnkle)
   {
      this.nominalHeightAboveGround = nominalHeightAboveAnkle;
   }

   @Override
   public double getNeckPitchUpperLimit()
   {
      return neckPitchUpperLimit;
   }

   @Override
   public double getNeckPitchLowerLimit()
   {
      return neckPitchLowerLimit;
   }

   @Override
   public double getHeadYawLimit()
   {
      return headYawLimit;
   }

   @Override
   public double getHeadRollLimit()
   {
      return headRollLimit;
   }

   @Override
   public double getFootForwardOffset()
   {
      return AtlasPhysicalProperties.footForward;
   }

   @Override
   public double getFootBackwardOffset()
   {
      return AtlasPhysicalProperties.footBackForControl;
   }

   @Override
   public double getAnkleHeight()
   {
      return AtlasPhysicalProperties.ankleHeight;
   }

   @Override
   public double getLegLength()
   {
      return AtlasPhysicalProperties.shinLength + AtlasPhysicalProperties.thighLength;
   }

   @Override
   public double getMinLegLengthBeforeCollapsingSingleSupport()
   {
      return min_leg_length_before_collapsing_single_support;
   }

   @Override
   public double getMinMechanicalLegLength()
   {
      return min_mechanical_leg_length;
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
      return 0.6;    // 0.5; //0.35;
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
   public double getDefaultStepLength()
   {
      return 0.6;
   }

   @Override
   public double getMaxStepUp()
   {
      return 0.25;
   }

   @Override
   public double getMaxStepDown()
   {
      return 0.2;
   }

   @Override
   public double getMaxSwingHeightFromStanceFoot()
   {
      return 0.25;
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
      if (!(target == AtlasTarget.REAL_ROBOT))
         return 1.5;    // 1.0

      return 1.0;
   }

   @Override
   public double getCaptureKpOrthogonalToMotion()
   {
      if (!(target == AtlasTarget.REAL_ROBOT))
         return 1.5;    // 1.0

      return 1.0;
   }

   @Override
   public double getCaptureKi()
   {
      if (!(target == AtlasTarget.REAL_ROBOT))
         return 4.0;

      return 2.0;
   }

   @Override
   public double getCaptureKiBleedoff()
   {
      return 0.9;
   }

   @Override
   public double getCaptureFilterBreakFrequencyInHz()
   {
      if (!(target == AtlasTarget.REAL_ROBOT))
         return 16.0;    // Double.POSITIVE_INFINITY;

      return 16.0;
   }

   @Override
   public double getCMPRateLimit()
   {
      if (!(target == AtlasTarget.REAL_ROBOT))
         return 60.0;

      return 6.0;    // 3.0; //4.0; //3.0;
   }

   @Override
   public double getCMPAccelerationLimit()
   {
      if (!(target == AtlasTarget.REAL_ROBOT))
         return 2000.0;

      return 200.0;    // 80.0; //40.0;
   }

   @Override
   public YoPDGains createCoMHeightControlGains(YoVariableRegistry registry)
   {
      YoPDGains gains = new YoPDGains("CoMHeight", registry);

      double kp = (target == AtlasTarget.REAL_ROBOT) ? 40.0 : 40.0;
      double zeta = (target == AtlasTarget.REAL_ROBOT) ? 0.4 : 0.8;
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
      return false;
   }

   @Override
   public YoOrientationPIDGains createPelvisOrientationControlGains(YoVariableRegistry registry)
   {
      YoSymmetricSE3PIDGains gains = new YoSymmetricSE3PIDGains("PelvisOrientation", registry);

      double kp = 80.0;
      double zeta = (target == AtlasTarget.REAL_ROBOT) ? 0.5 : 0.8;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = (target == AtlasTarget.REAL_ROBOT) ? 12.0 : 36.0;
      double maxJerk = (target == AtlasTarget.REAL_ROBOT) ? 180.0 : 540.0;

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
   public YoOrientationPIDGains createHeadOrientationControlGains(YoVariableRegistry registry)
   {
      YoSymmetricSE3PIDGains gains = new YoSymmetricSE3PIDGains("HeadOrientation", registry);

      double kp = 40.0;
      double zeta = (target == AtlasTarget.REAL_ROBOT) ? 0.4 : 0.8;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = (target == AtlasTarget.REAL_ROBOT) ? 6.0 : 36.0;
      double maxJerk = (target == AtlasTarget.REAL_ROBOT) ? 60.0 : 540.0;

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
      return 3.0;
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

      double kp = (target == AtlasTarget.REAL_ROBOT) ? 80.0 : 80.0;
      double zeta = (target == AtlasTarget.REAL_ROBOT) ? 0.25 : 0.8;
      double maxAcceleration = (target == AtlasTarget.REAL_ROBOT) ? 6.0 : 36.0;
      double maxJerk = (target == AtlasTarget.REAL_ROBOT) ? 60.0 : 540.0;

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

      double kpXY = 80.0;
      double kpZ = 80.0;
      double zetaXY = (target == AtlasTarget.REAL_ROBOT) ? 0.5 : 0.8;
      double zetaZ = (target == AtlasTarget.REAL_ROBOT) ? 0.5 : 0.8;
      double maxAccel = (target == AtlasTarget.REAL_ROBOT) ? 6.0 : 36.0;
      double maxJerk = (target == AtlasTarget.REAL_ROBOT) ? 60.0 : 540.0;

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

      double kpXY = 100.0;
      double kpZ = 200.0;
      double zetaXYZ = (target == AtlasTarget.REAL_ROBOT) ? 0.25 : 0.7;
      double kpOrientation = 200.0;
      double zetaOrientation = 0.7;
      double maxPositionAcceleration = (target == AtlasTarget.REAL_ROBOT) ? 10.0 : Double.POSITIVE_INFINITY;
      double maxPositionJerk = (target == AtlasTarget.REAL_ROBOT) ? 150.0 : Double.POSITIVE_INFINITY;
      double maxOrientationAcceleration = (target == AtlasTarget.REAL_ROBOT) ? 100.0 : Double.POSITIVE_INFINITY;
      double maxOrientationJerk = (target == AtlasTarget.REAL_ROBOT) ? 1500.0 : Double.POSITIVE_INFINITY;

      gains.setPositionProportionalGains(kpXY, kpZ);
      gains.setPositionDampingRatio(zetaXYZ);
      gains.setPositionMaxAccelerationAndJerk(maxPositionAcceleration, maxPositionJerk);
      gains.setOrientationProportionalGains(kpOrientation, kpOrientation);
      gains.setOrientationDampingRatio(zetaOrientation);
      gains.setOrientationMaxAccelerationAndJerk(maxOrientationAcceleration, maxOrientationJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   @Override
   public YoSE3PIDGains createHoldPositionFootControlGains(YoVariableRegistry registry)
   {
      YoFootSE3Gains gains = new YoFootSE3Gains("HoldFoot", registry);

      double kpXY = 100.0;
      double kpZ = 0.0;
      double zetaXYZ = (target == AtlasTarget.REAL_ROBOT) ? 0.2 : 1.0;
      double kpXYOrientation = (target == AtlasTarget.REAL_ROBOT) ? 100.0 : 200.0;
      double kpZOrientation = (target == AtlasTarget.REAL_ROBOT) ? 100.0 : 200.0;
      double zetaOrientation = (target == AtlasTarget.REAL_ROBOT) ? 0.2 : 1.0;
      double maxLinearAcceleration = (target == AtlasTarget.REAL_ROBOT) ? 10.0 : Double.POSITIVE_INFINITY;
      double maxLinearJerk = (target == AtlasTarget.REAL_ROBOT) ? 150.0 : Double.POSITIVE_INFINITY;
      double maxAngularAcceleration = (target == AtlasTarget.REAL_ROBOT) ? 100.0 : Double.POSITIVE_INFINITY;
      double maxAngularJerk = (target == AtlasTarget.REAL_ROBOT) ? 1500.0 : Double.POSITIVE_INFINITY;

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

      double kpXY = 100.0;
      double kpZ = 0.0;
      double zetaXYZ = (target == AtlasTarget.REAL_ROBOT) ? 0.4 : 0.4;
      double kpXYOrientation = (target == AtlasTarget.REAL_ROBOT) ? 200.0 : 200.0;
      double kpZOrientation = (target == AtlasTarget.REAL_ROBOT) ? 200.0 : 200.0;
      double zetaOrientation = (target == AtlasTarget.REAL_ROBOT) ? 0.4 : 0.4;
      double maxLinearAcceleration = (target == AtlasTarget.REAL_ROBOT) ? 10.0 : Double.POSITIVE_INFINITY;
      double maxLinearJerk = (target == AtlasTarget.REAL_ROBOT) ? 150.0 : Double.POSITIVE_INFINITY;
      double maxAngularAcceleration = (target == AtlasTarget.REAL_ROBOT) ? 100.0 : Double.POSITIVE_INFINITY;
      double maxAngularJerk = (target == AtlasTarget.REAL_ROBOT) ? 1500.0 : Double.POSITIVE_INFINITY;

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
      double zetaXYZ = (target == AtlasTarget.REAL_ROBOT) ? 0.0 : 0.0;
      double kpXYOrientation = (target == AtlasTarget.REAL_ROBOT) ? 40.0 : 300.0;
      double kpZOrientation = (target == AtlasTarget.REAL_ROBOT) ? 40.0 : 300.0;
      double zetaOrientation = (target == AtlasTarget.REAL_ROBOT) ? 0.4 : 0.4;
      double maxLinearAcceleration = (target == AtlasTarget.REAL_ROBOT) ? 10.0 : Double.POSITIVE_INFINITY;
      double maxLinearJerk = (target == AtlasTarget.REAL_ROBOT) ? 150.0 : Double.POSITIVE_INFINITY;
      double maxAngularAcceleration = (target == AtlasTarget.REAL_ROBOT) ? 100.0 : Double.POSITIVE_INFINITY;
      double maxAngularJerk = (target == AtlasTarget.REAL_ROBOT) ? 1500.0 : Double.POSITIVE_INFINITY;

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
      YoIndependentSE3PIDGains gains = new YoIndependentSE3PIDGains("SupportFoot", registry);

      double maxAngularAcceleration = (target == AtlasTarget.REAL_ROBOT) ? 100.0 : Double.POSITIVE_INFINITY;
      double maxAngularJerk = (target == AtlasTarget.REAL_ROBOT) ? 1500.0 : Double.POSITIVE_INFINITY;

      gains.setOrientationDerivativeGains(20.0, 0.0, 0.0);
      gains.setOrientationMaxAccelerationAndJerk(maxAngularAcceleration, maxAngularJerk);

      return gains;
   }

   @Override
   public double getSwingHeightMaxForPushRecoveryTrajectory()
   {
      return 0.12;
   }

   public double getSwingMaxHeightForPushRecoveryTrajectory()
   {
      return 0.15;
   }

   @Override
   public double getSupportSingularityEscapeMultiplier()
   {
      return 30;
   }

   @Override
   public double getSwingSingularityEscapeMultiplier()
   {
      return (target == AtlasTarget.REAL_ROBOT) ? 50.0 : 200.0;
   }

   @Override
   public boolean doPrepareManipulationForLocomotion()
   {
      return true;
   }

   @Override
   public double getDefaultTransferTime()
   {
      return (target == AtlasTarget.REAL_ROBOT) ? 1.5 : 0.25;
   }

   @Override
   public double getDefaultSwingTime()
   {
      return (target == AtlasTarget.REAL_ROBOT) ? 1.5 : 0.60;
   }

   /** @inheritDoc */
   @Override
   public double getSpineYawLimit()
   {
      return spineYawLimit;
   }

   /** @inheritDoc */
   @Override
   public double getSpinePitchUpperLimit()
   {
      return spinePitchUpperLimit;
   }

   /** @inheritDoc */
   @Override
   public double getSpinePitchLowerLimit()
   {
      return spinePitchLowerLimit;
   }

   /** @inheritDoc */
   @Override
   public double getSpineRollLimit()
   {
      return spineRollLimit;
   }

   /** @inheritDoc */
   @Override
   public boolean isSpinePitchReversed()
   {
      return false;
   }

   @Override
   public double getFootWidth()
   {
      return AtlasPhysicalProperties.footWidthForControl;
   }

   @Override
   public double getToeWidth()
   {
      return AtlasPhysicalProperties.toeWidthForControl;
   }

   @Override
   public double getFootLength()
   {
      return AtlasPhysicalProperties.footLengthForControl;
   }

   @Override
   public double getActualFootWidth()
   {
      return AtlasPhysicalProperties.actualFootWidth;
   }

   @Override
   public double getActualFootLength()
   {
      return AtlasPhysicalProperties.actualFootLength;
   }

   @Override
   public double getFootstepArea()
   {
      return (getToeWidth() + getFootWidth()) * getFootLength() / 2.0;
   }

   @Override
   public double getFoot_start_toetaper_from_back()
   {
      return AtlasPhysicalProperties.footStartToetaperFromBack;
   }

   @Override
   public double getSideLengthOfBoundingBoxForFootstepHeight()
   {
      return (1 + 0.3) * 2 * Math.sqrt(getFootForwardOffset() * getFootForwardOffset() + 0.25 * getFootWidth() * getFootWidth());
   }

   @Override
   public SideDependentList<RigidBodyTransform> getDesiredHandPosesWithRespectToChestFrame()
   {
      return handPosesWithRespectToChestFrame;
   }

   @Override
   public double getDesiredTouchdownHeightOffset()
   {
      return 0;
   }

   @Override
   public double getDesiredTouchdownVelocity()
   {
      return -0.3;
   }

   @Override
   public double getDesiredTouchdownAcceleration()
   {
      switch (target)
      {
         case SIM :
            return -2.0;

         default :
            return -0.0;
      }
   }

   @Override
   public double getContactThresholdForce()
   {
      switch (target)
      {
         case REAL_ROBOT :
            return 80.0;

         case GAZEBO :
            return 50.0;

         case SIM :
            return 5.0;

         default :
            throw new RuntimeException();
      }
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
   public void setupMomentumOptimizationSettings(MomentumOptimizationSettings momentumOptimizationSettings)
   {
      momentumOptimizationSettings.setDampedLeastSquaresFactor(0.05);
      momentumOptimizationSettings.setRhoPlaneContactRegularization(0.001);
      momentumOptimizationSettings.setMomentumWeight(1.0, 1.0, 10.0, 10.0);
      momentumOptimizationSettings.setRhoMin(4.0);
      momentumOptimizationSettings.setRateOfChangeOfRhoPlaneContactRegularization(0.12);    // 0.01 causes ankle to flip out when rotates on edge. 0.12 prevents this.
      momentumOptimizationSettings.setRhoPenalizerPlaneContactRegularization(0.01);
   }

   @Override
   public boolean doFancyOnToesControl()
   {
      return !(target == AtlasTarget.REAL_ROBOT);
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
      return 0.025; //0.035;
   }

   @Override
   public boolean finishSingleSupportWhenICPPlannerIsDone()
   {
      return false;
   }

   @Override
   public double minimumHeightBetweenAnkleAndPelvisForHeightAdjustment()
   {
      return 0.3818;
   }

   @Override
   public double nominalHeightBetweenAnkleAndPelvisForHeightAdjustment()
   {
      return 0.7049;
   }

   @Override
   public double maximumHeightBetweenAnkleAndPelvisForHeightAdjustment()
   {
      return 0.7749;
   }

   @Override
   public double pelvisToAnkleThresholdForWalking()
   {
      return 0.623;
   }
}

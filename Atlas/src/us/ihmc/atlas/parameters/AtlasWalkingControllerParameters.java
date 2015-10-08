package us.ihmc.atlas.parameters;

import java.util.LinkedHashMap;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.SdfLoader.partNames.NeckJointName;
import us.ihmc.SdfLoader.partNames.SpineJointName;
import us.ihmc.atlas.AtlasJointMap;
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
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;



public class AtlasWalkingControllerParameters implements WalkingControllerParameters
{
   private final DRCRobotModel.RobotTarget target;
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
      this(DRCRobotModel.RobotTarget.SCS, jointMap);
   }

   public AtlasWalkingControllerParameters(DRCRobotModel.RobotTarget target, AtlasJointMap jointMap)
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
   public double getOmega0()
   {
      // TODO probably need to be tuned.
//      boolean realRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;
//      return realRobot ? 3.4 : 3.0; // 3.0 seems more appropriate.
      return 3.0;
   }

   @Override
   public double getTimeToGetPreparedForLocomotion()
   {
      boolean realRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;
      return realRobot ? 0.3 : 0.0; // 0.3 seems to be a good starting point
   }

   @Override
   public boolean doToeOffIfPossible()
   {
      return true;
   }

   @Override
   public boolean checkTrailingLegJacobianDeterminantToTriggerToeOff()
   {
      boolean realRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;
      return !realRobot;
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
//    return target == DRCRobotModel.RobotTarget.REAL_ROBOT;
    return false; // Does more bad than good
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
      if (target == DRCRobotModel.RobotTarget.REAL_ROBOT)
         return 0.4;
      else
         return 0.3;
   }

   public boolean isNeckPositionControlled()
   {
      if (target == DRCRobotModel.RobotTarget.REAL_ROBOT)
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

   @Override
   public double defaultOffsetHeightAboveAnkle()
   {
      double defaultOffset = target == DRCRobotModel.RobotTarget.REAL_ROBOT ? 0.035 : 0.0;
      return defaultOffset;
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
      return 0.30;
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
      boolean realRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;

      double kpParallel = realRobot ? 2.5 : 1.5;
      double kpOrthogonal = 1.5;
      double ki = realRobot ? 0.0 : 4.0;
      double kiBleedOff = 0.9;
      boolean useRawCMP = true;
      boolean useHackToReduceFeedForward = false;
//      double cmpFilterBreakFrequencyInHertz = 16.0;
//      double cmpRateLimit = 60.0;
//      double cmpAccelerationLimit = 2000.0;

      gains.setKpParallelToMotion(kpParallel);
      gains.setKpOrthogonalToMotion(kpOrthogonal);
      gains.setKi(ki);
      gains.setKiBleedOff(kiBleedOff);
      gains.setUseRawCMP(useRawCMP);
      //      gains.setCMPFilterBreakFrequencyInHertz(cmpFilterBreakFrequencyInHertz);
//      gains.setCMPRateLimit(cmpRateLimit);
//      gains.setCMPAccelerationLimit(cmpAccelerationLimit);
      gains.setUseHackToReduceFeedForward(useHackToReduceFeedForward);

      return gains;
   }

   @Override
   public YoPDGains createCoMHeightControlGains(YoVariableRegistry registry)
   {
      YoPDGains gains = new YoPDGains("CoMHeight", registry);
      boolean realRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;

      double kp = 40.0;
      double zeta = realRobot ? 0.4 : 0.8;
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
      YoPDGains gains = new YoPDGains("PelvisXY", registry);
      boolean realRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;

      gains.setKp(4.0);
      gains.setKd(realRobot ? 0.5 : 1.2);

      return gains;
   }

   @Override
   public YoOrientationPIDGains createPelvisOrientationControlGains(YoVariableRegistry registry)
   {
      YoSymmetricSE3PIDGains gains = new YoSymmetricSE3PIDGains("PelvisOrientation", registry);
      boolean realRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;

      double kp = 80.0;
      double zeta = realRobot ? 0.5 : 0.8;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = realRobot ? 12.0 : 36.0;
      double maxJerk = realRobot ? 180.0 : 540.0;

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
      boolean realRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;

      double kp = 40.0;
      double zeta = realRobot ? 0.4 : 0.8;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = realRobot ? 6.0 : 36.0;
      double maxJerk = realRobot ? 60.0 : 540.0;

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
      boolean realRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;

      double kp = 80.0;
      double zeta = realRobot ? 0.25 : 0.8;
      double maxAcceleration = realRobot ? 6.0 : 36.0;
      double maxJerk = realRobot ? 60.0 : 540.0;

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
      boolean realRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;

      double kpXY = 40.0; //80.0;
      double kpZ = 40.0; //80.0;
      double zetaXY = realRobot ? 0.5 : 0.8;
      double zetaZ = realRobot ? 0.22 : 0.8;
      double maxAccel = realRobot ? 6.0 : 36.0;
      double maxJerk = realRobot ? 60.0 : 540.0;

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
      boolean realRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;

      double kpXY = 150.0;
      double kpZ = 200.0;
      double zetaXYZ = realRobot ? 0.7 : 0.7;
      double kpOrientation = 200.0;
      double zetaOrientation = 0.7;
      // Reduce maxPositionAcceleration from 30 to 6 to prevent too high acceleration when hitting joint limits.
      double maxPositionAcceleration = realRobot ? 6.0 : Double.POSITIVE_INFINITY;
//      double maxPositionAcceleration = realRobot ? 30.0 : Double.POSITIVE_INFINITY;
      double maxPositionJerk = realRobot ? 150.0 : Double.POSITIVE_INFINITY;
      double maxOrientationAcceleration = realRobot ? 100.0 : Double.POSITIVE_INFINITY;
      double maxOrientationJerk = realRobot ? 1500.0 : Double.POSITIVE_INFINITY;

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
      boolean realRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;

      double kpXY = 100.0;
      double kpZ = 0.0;
      double zetaXYZ = realRobot ? 0.2 : 1.0;
      double kpXYOrientation = realRobot ? 100.0 : 200.0;
      double kpZOrientation = realRobot ? 100.0 : 200.0;
      double zetaOrientation = realRobot ? 0.2 : 1.0;
      // Reduce maxPositionAcceleration from 10 to 6 to prevent too high acceleration when hitting joint limits.
      double maxLinearAcceleration = realRobot ? 6.0 : Double.POSITIVE_INFINITY;
//      double maxLinearAcceleration = realRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxLinearJerk = realRobot ? 150.0 : Double.POSITIVE_INFINITY;
      double maxAngularAcceleration = realRobot ? 100.0 : Double.POSITIVE_INFINITY;
      double maxAngularJerk = realRobot ? 1500.0 : Double.POSITIVE_INFINITY;

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
      boolean realRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;

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
      boolean realRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;

      double kp = 0.0;
      double zetaXYZ = 0.0;
      double kpXYOrientation = realRobot ? 40.0 : 300.0;
      double kpZOrientation = realRobot ? 40.0 : 300.0;
      double zetaOrientation = 0.4;
      double maxLinearAcceleration = realRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxLinearJerk = realRobot ? 150.0 : Double.POSITIVE_INFINITY;
      double maxAngularAcceleration = realRobot ? 100.0 : Double.POSITIVE_INFINITY;
      double maxAngularJerk = realRobot ? 1500.0 : Double.POSITIVE_INFINITY;

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
      return (target == DRCRobotModel.RobotTarget.REAL_ROBOT) ? 50.0 : 200.0;
   }

   @Override
   public boolean doPrepareManipulationForLocomotion()
   {
      //this allows me to walk with hand poses locked in a reference frame
      return false;
   }

   @Override
   public double getDefaultTransferTime()
   {
      return (target == DRCRobotModel.RobotTarget.REAL_ROBOT) ? 0.8 : 0.25;
   }

   @Override
   public double getDefaultSwingTime()
   {
      return (target == DRCRobotModel.RobotTarget.REAL_ROBOT) ? 1.2 : 0.60;
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
         case SCS:
            return -2.0;

         default :
            return -1.0;
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

         case SCS:
            return 5.0;

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
   public void setupMomentumOptimizationSettings(MomentumOptimizationSettings momentumOptimizationSettings)
   {
      momentumOptimizationSettings.setDampedLeastSquaresFactor(0.05);
      momentumOptimizationSettings.setRhoPlaneContactRegularization(0.001);
      momentumOptimizationSettings.setMomentumWeight(2.0, 1.4, 10.0, 10.0); //(1.0, 1.0, 10.0, 10.0);
      momentumOptimizationSettings.setRhoMin(4.0);
      momentumOptimizationSettings.setRateOfChangeOfRhoPlaneContactRegularization(0.16);  //0.06 causes a little less oscillations possibly.  // 0.01 causes ankle to flip out when rotates on edge. 0.12 prevents this.
      momentumOptimizationSettings.setRhoPenalizerPlaneContactRegularization(0.01);
   }

   @Override
   public boolean doFancyOnToesControl()
   {
      return target != DRCRobotModel.RobotTarget.REAL_ROBOT;
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

   /** {@inheritDoc} */
   @Override
   public double getDurationToCancelOutDesiredICPVelocityWhenStuckInTransfer()
   {
      return 0.5;
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
   
   @Override
   public boolean controlHeadAndHandsWithSliders()
   {
      return false;
   }

   @Override
   public SideDependentList<LinkedHashMap<String, ImmutablePair<Double, Double>>> getSliderBoardControlledFingerJointsWithLimits()
   {
      return new SideDependentList<LinkedHashMap<String, ImmutablePair<Double,Double>>>();
   }

   @Override
   public LinkedHashMap<NeckJointName, ImmutablePair<Double, Double>> getSliderBoardControlledNeckJointsWithLimits()
   {
      return new LinkedHashMap<NeckJointName, ImmutablePair<Double,Double>>();
   }

   /** {@inheritDoc} */
   @Override
   public boolean useICPPlannerHackN13()
   {
      return false;
   }
}

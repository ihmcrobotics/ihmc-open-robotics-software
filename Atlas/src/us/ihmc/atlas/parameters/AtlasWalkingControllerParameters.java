package us.ihmc.atlas.parameters;

import static us.ihmc.atlas.ros.AtlasOrderedJointMap.back_bky;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.back_bkz;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.jointNames;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.neck_ry;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.YoFootSE3Gains;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.yoUtilities.controllers.YoIndependentSE3PIDGains;
import us.ihmc.yoUtilities.controllers.YoOrientationPIDGains;
import us.ihmc.yoUtilities.controllers.YoPDGains;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.controllers.YoSymmetricSE3PIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;



public class AtlasWalkingControllerParameters implements WalkingControllerParameters
{   
   private final boolean runningOnRealRobot;
   private final SideDependentList<RigidBodyTransform> handPosesWithRespectToChestFrame = new SideDependentList<RigidBodyTransform>();

   // Limits
   private final double neck_pitch_upper_limit = 1.14494; //0.83;    // true limit is = 1.134460, but pitching down more just looks at more robot chest
   private final double neck_pitch_lower_limit = -0.602139; //-0.610865;    // -math.pi/2.0;
   private final double head_yaw_limit = Math.PI / 4.0;
   private final double head_roll_limit = Math.PI / 4.0;
   private final double pelvis_pitch_upper_limit = 0.4;
   private final double pelvis_pitch_lower_limit = -0.1; //-math.pi / 6.0;

   private final double  min_leg_length_before_collapsing_single_support = 0.53; // corresponds to q_kny = 1.70 rad
   private final double  min_mechanical_leg_length = 0.420; // corresponds to a q_kny that is close to knee limit

   public AtlasWalkingControllerParameters()
   {
      this(false);
   }
   
   public AtlasWalkingControllerParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyTransform transform = new RigidBodyTransform();

         double x = 0.20;
         double y = robotSide.negateIfRightSide(0.35); //0.30);
         double z = -0.40;
         transform.setTranslation(new Vector3d(x, y, z));

         Matrix3d rotation = new Matrix3d();
         double yaw = 0.0;//robotSide.negateIfRightSide(-1.7);
         double pitch = 0.7;
         double roll = 0.0;//robotSide.negateIfRightSide(-0.8);
         RotationFunctions.setYawPitchRoll(rotation, yaw, pitch, roll);
         transform.setRotation(rotation);

         handPosesWithRespectToChestFrame.put(robotSide, transform);
      }
   }
   
   @Override
   public boolean stayOnToes()
   {
      return false; // Not working for now
   }

   @Override
   public boolean doToeOffIfPossible()
   {
      return true; 
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
   public String[] getDefaultHeadOrientationControlJointNames()
   {
      return new String[] {jointNames[back_bkz], jointNames[neck_ry]}; 
   }
   
   @Override
   public String[] getDefaultChestOrientationControlJointNames()
   {
      return new String[]{};
   }

   @Override
   public boolean checkOrbitalEnergyCondition()
   {
      return false;
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
   public double getGroundReactionWrenchBreakFrequencyHertz()
   {
      return 7.0;
   }

   @Override
   public boolean resetDesiredICPToCurrentAtStartOfSwing()
   {
      return false;
   }

   @Override
   public double getUpperNeckPitchLimit()
   {
      return neck_pitch_upper_limit;
   }

   @Override
   public double getLowerNeckPitchLimit()
   {
      return neck_pitch_lower_limit;
   }

   @Override
   public double getHeadYawLimit()
   {
      return head_yaw_limit;
   }

   @Override
   public double getHeadRollLimit()
   {
      return head_roll_limit;
   }

   @Override
   public String getJointNameForExtendedPitchRange()
   {
      return jointNames[back_bky];
   }

   @Override
   public boolean finishSwingWhenTrajectoryDone()
   {
      return false;
   }

   @Override
   public double getFootForwardOffset()
   {
      return AtlasPhysicalProperties.footForward;
   }
   
   @Override
   public double getFootSwitchCoPThresholdFraction()
   {
	   return 0.02;
   }

   @Override
   public double getFootBackwardOffset()
   {
      return AtlasPhysicalProperties.footBack;
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
   public double getFinalToeOffPitchAngularVelocity()
   {
      return 3.5;
   }

   @Override
   public double getInPlaceWidth()
   {
      return 0.25;
   }

   @Override
   public double getDesiredStepForward()
   {
      return 0.5; //0.35;
   }
  
   @Override
   public double getMaxStepLength()
   {
       return 0.6; //0.5; //0.35;
   }

   @Override
   public double getMinStepWidth()
   {
      return 0.15;
   }

   @Override
   public double getMaxStepWidth()
   {
      return 0.6; //0.4;
   }

   @Override
   public double getStepPitch()
   {
      return 0.0;
   }

   @Override
   public double getCaptureKpParallelToMotion()
   {
      if (!runningOnRealRobot) return 1.0;
      return 1.0; 
   }

   @Override
   public double getCaptureKpOrthogonalToMotion()
   {      
      if (!runningOnRealRobot) return 1.0; 
      return 1.0; 
   }
   
   @Override
   public double getCaptureKi()
   {      
      if (!runningOnRealRobot) return 4.0;
      return 4.0; 
   }
   
   @Override
   public double getCaptureKiBleedoff()
   {      
      return 0.9; 
   }
   
   @Override
   public double getCaptureFilterBreakFrequencyInHz()
   {
      if (!runningOnRealRobot) return 16.0; //Double.POSITIVE_INFINITY;
      return 16.0;
   }
   
   @Override
   public double getCMPRateLimit()
   {
      if (!runningOnRealRobot) return 60.0; 
      return 6.0; //3.0; //4.0; //3.0;
   }

   @Override
   public double getCMPAccelerationLimit()
   {
      if (!runningOnRealRobot) return 2000.0;
      return 200.0; //80.0; //40.0;
   }

   @Override
   public YoPDGains createCoMHeightControlGains(YoVariableRegistry registry)
   {
      YoPDGains gains = new YoPDGains("CoMHeight", registry);

      double kp = runningOnRealRobot ? 40.0 : 40.0;
      double zeta = runningOnRealRobot ? 0.4 : 0.8;
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
   public double getDefaultDesiredPelvisPitch()
   {
      return 0.0;
   }

   @Override
   public YoOrientationPIDGains createPelvisOrientationControlGains(YoVariableRegistry registry)
   {
      YoSymmetricSE3PIDGains gains = new YoSymmetricSE3PIDGains("PelvisOrientation", registry);

      double kp = 80.0;
      double zeta = runningOnRealRobot ? 0.25 : 0.8;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = runningOnRealRobot ? 12.0 : 36.0;
      double maxJerk = runningOnRealRobot ? 180.0 : 540.0;

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
      double zeta = runningOnRealRobot ? 0.4 : 0.8;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = runningOnRealRobot ? 6.0 : 36.0;
      double maxJerk = runningOnRealRobot ? 60.0 : 540.0;

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
      return new double[]{0.0, 0.67, 0.0};
   }

   @Override
   public double getKpUpperBody()
   {
      if (!runningOnRealRobot) return 80.0; //100.0;
      return 80.0; //40.0;
   }

   @Override
   public double getZetaUpperBody()
   {
      if (!runningOnRealRobot) return 0.8; //1.0;
      return 0.25;
   }
   
   @Override
   public double getMaxAccelerationUpperBody()
   {
      if (!runningOnRealRobot) return 36.0; // 18.0; //100.0;
      return 6.0;
   }
   
   @Override
   public double getMaxJerkUpperBody()
   {
      if (!runningOnRealRobot) return 540.0; // 270.0; //1000.0;
      return 60.0;
   }

   @Override
   public YoOrientationPIDGains createChestControlGains(YoVariableRegistry registry)
   {
      YoSymmetricSE3PIDGains gains = new YoSymmetricSE3PIDGains("ChestOrientation", registry);

      double kp = 80.0;
      double zeta = runningOnRealRobot ? 0.25 : 0.8;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = runningOnRealRobot ? 6.0 : 36.0;
      double maxJerk = runningOnRealRobot ? 60.0 : 540.0;

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
   public YoSE3PIDGains createSwingFootControlGains(YoVariableRegistry registry)
   {
      YoFootSE3Gains gains = new YoFootSE3Gains("SwingFoot", registry);

      double kpXY = 100.0;
      double kpZ = 200.0;
      double zetaXYZ = runningOnRealRobot ? 0.25 : 0.7;
      double kpOrientation = 200.0;
      double zetaOrientation = 0.7;
      double maxPositionAcceleration = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxPositionJerk = runningOnRealRobot ? 150.0 : Double.POSITIVE_INFINITY;
      double maxOrientationAcceleration = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;
      double maxOrientationJerk = runningOnRealRobot ? 1500.0 : Double.POSITIVE_INFINITY;
      
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
      double zetaXYZ = runningOnRealRobot ? 0.2 : 1.0;
      double kpXYOrientation = runningOnRealRobot ? 100.0 : 200.0;
      double kpZOrientation = runningOnRealRobot ? 100.0 : 200.0;
      double zetaOrientation = runningOnRealRobot ? 0.2 : 1.0;
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

      double kpXY = 100.0;
      double kpZ = 0.0;
      double zetaXYZ = runningOnRealRobot ? 0.4 : 0.4;
      double kpXYOrientation = runningOnRealRobot ? 200.0 : 200.0;
      double kpZOrientation = runningOnRealRobot ? 200.0 : 200.0;
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
   public YoSE3PIDGains createSupportFootControlGains(YoVariableRegistry registry)
   {
      YoIndependentSE3PIDGains gains = new YoIndependentSE3PIDGains("SupportFoot", registry);

      double maxAngularAcceleration = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;
      double maxAngularJerk = runningOnRealRobot ? 1500.0 : Double.POSITIVE_INFINITY;
      
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
      return runningOnRealRobot ? 50.0 : 200.0;
   }

   @Override
   public boolean doPrepareManipulationForLocomotion()
   {
      return true;
   }

   @Override
   public boolean isRunningOnRealRobot()
   {
      return runningOnRealRobot;
   }

   @Override
   public double getDefaultTransferTime()
   {
      return runningOnRealRobot ? 1.5 : 0.25;
   }

   @Override
   public double getDefaultSwingTime()
   {
      return runningOnRealRobot ? 1.5 : 0.60;
   }

   @Override
   public double getPelvisPitchUpperLimit()
   {
      return pelvis_pitch_upper_limit;
   }
   
   @Override
   public double getPelvisPitchLowerLimit()
   {
      return pelvis_pitch_lower_limit;
   }

   @Override
   public boolean isPelvisPitchReversed()
   {
      return false;
   }

   @Override
   public double getFootWidth()
   {
      return AtlasPhysicalProperties.footWidth;
   }

   @Override
   public double getToeWidth()
   {
      return AtlasPhysicalProperties.toeWidth;
   }

   @Override
   public double getFootLength()
   {
      return AtlasPhysicalProperties.footLength;
   }

   @Override
   public double getFoot_start_toetaper_from_back()
   {
      return AtlasPhysicalProperties.footStartToetaperFromBack;
   }

   @Override
   public double getSideLengthOfBoundingBoxForFootstepHeight()
   {
      return (1 + 0.3) * 2 * Math.sqrt(getFootForwardOffset() * getFootForwardOffset()
            + 0.25 * getFootWidth() * getFootWidth());
   }
   
   @Override
   public SideDependentList<RigidBodyTransform> getDesiredHandPosesWithRespectToChestFrame()
   {
      return handPosesWithRespectToChestFrame;
   }

   @Override
   public double getDesiredTouchdownVelocity()
   {
      return -0.3;
   }

   @Override
   public double getContactThresholdForce()
   {
      return runningOnRealRobot ? 80.0 : 5.0;
   }
   
   @Override
   public double getCoPThresholdFraction()
   {
      return 0.02;
   }

   @Override
   public String[] getJointsToIgnoreInController()
   {
      return null;
   }

   @Override
   public void setupMomentumOptimizationSettings(MomentumOptimizationSettings momentumOptimizationSettings)
   {
      momentumOptimizationSettings.setDampedLeastSquaresFactor(0.05);
      momentumOptimizationSettings.setRhoPlaneContactRegularization(0.001);
      momentumOptimizationSettings.setMomentumWeight(1.0, 1.0, 10.0, 10.0);
      momentumOptimizationSettings.setRhoMin(4.0);
      momentumOptimizationSettings.setRateOfChangeOfRhoPlaneContactRegularization(0.01);
      momentumOptimizationSettings.setRhoPenalizerPlaneContactRegularization(0.01);
   }
}

package us.ihmc.alexander.parameters;

import java.util.Objects;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Real robot parameters wired from YoVariables registry tree.
 * - All parameter values match the original.
 * - Constructor is decomposed into logically grouped init-methods.
 * - Helpers add null/type safety and support multiple name variants (XML vs code).
 */
public class SimulationParameters extends AbstractRobotParameters
{
   /** Root registry used for lookups. */

   // ---------------- Transitions ----------------
   private YoBoolean yoToWalkingEnableTimeBasedTransition;
   private YoBoolean yoExitWalkingEnableTimeBasedTransition;

   // ---------------- JointspaceGains ----------------
   private YoDouble yoZetaSpineJoints;
   private YoDouble yoMaximumFeedbackSpineJoints;
   private YoDouble yoMaximumFeedbackRateSpineJoints;
   private YoDouble yoZetaNeckJoints;
   private YoDouble yoMaximumFeedbackNeckJoints;
   private YoDouble yoMaximumFeedbackRateNeckJoints;
   private YoDouble yoKpArmJoints;
   private YoDouble yoZetaArmJoints;
   private YoDouble yoMaximumFeedbackArmJoints;
   private YoDouble yoMaximumFeedbackRateArmJoints;

   // ---------------- RigidBodyGains ----------------
   private YoDouble yoZetaXYChestOrientation;
   private YoDouble yoZetaZChestOrientation;
   private YoDouble yoMaximumFeedbackChestOrientation;
   private YoDouble yoMaximumFeedbackRateChestOrientation;
   private YoDouble yoZetaXYZHeadOrientation;
   private YoDouble yoMaximumFeedbackHeadOrientation;
   private YoDouble yoMaximumFeedbackRateHeadOrientation;
   private YoDouble yoKpXYZHandOrientation;
   private YoDouble yoZetaXYZHandOrientation;
   private YoDouble yoMaximumFeedbackHandOrientation;
   private YoDouble yoMaximumFeedbackRateHandOrientation;
   private YoDouble yoZetaXYPelvisOrientation;
   private YoDouble yoZetaZPelvisOrientation;
   private YoDouble yoMaximumFeedbackPelvisOrientation;
   private YoDouble yoMaximumFeedbackRatePelvisOrientation;
   private YoDouble yoKpXYZHandPosition;
   private YoDouble yoZetaXYZHandPosition;
   private YoDouble yoMaximumFeedbackHandPosition;
   private YoDouble yoMaximumFeedbackRateHandPosition;

   // ---------------- FootGains ----------------
   private YoDouble yoMaximumFeedbackSwingFootPosition;
   private YoDouble yoMaximumFeedbackRateSwingFootPosition;
   private YoDouble yoMaximumFeedbackSwingFootOrientation;
   private YoDouble yoMaximumFeedbackRateSwingFootOrientation;
   private YoDouble yoZetaXYHoldFootPosition;
   private YoDouble yoMaximumFeedbackHoldFootPosition;
   private YoDouble yoMaximumFeedbackRateHoldFootPosition;
   private YoDouble yoKpXYHoldFootOrientation;
   private YoDouble yoKpZHoldFootOrientation;
   private YoDouble yoZetaXYHoldFootOrientation;
   private YoDouble yoZetaZHoldFootOrientation;
   private YoDouble yoMaximumFeedbackHoldFootOrientation;
   private YoDouble yoMaximumFeedbackRateHoldFootOrientation;
   private YoDouble yoMaximumFeedbackToeOffFootPosition;
   private YoDouble yoMaximumFeedbackRateToeOffFootPosition;
   private YoDouble yoMaximumFeedbackToeOffFootOrientation;
   private YoDouble yoMaximumFeedbackRateToeOffFootOrientation;

   // ---------------- Pelvis / CoM height ----------------
   private YoDouble yoPelvisHeightControlStateDefaultHeight;
   private YoDouble yoPelvisHeightControlStateOffsetTrajectoryTime;
   private YoDouble yoPelvisHeightControlStateMaxDistanceAnklePelvis;
   private YoBoolean yoProcessGoHome;

   // ---------------- Feet manager misc ----------------
   private YoDouble yoAnkleLowerLimitToTriggerToeOff;
   private YoDouble yoOmegaThresholdForEstimation;
   private YoDouble yoRotationThreshold;
   private YoBoolean yoDampFootRotations;
   private YoDouble yoFootDamping;
   private YoDouble yoFootSwingTouchdownVelocityZ;
   private YoDouble yoFootSwingTouchdownAccelerationZ;
   private YoDouble yoVerificationPerpendicularCoPErrorThreshold;
   private YoDouble yoVerificationDistanceFromLineToComputeDesiredCoPOccupancy;
   private YoDouble yoVelocityOmegaMagnitudeThreshold;
   private YoDouble yoVelocityRotationAngleDecayBreakFrequency;
   private YoDouble yoVelocityRotationAngleThreshold;
   private YoInteger yoCroppingThresholdForCoPRegionOccupancy;
   private YoDouble yoCroppingDistanceFromLineOfRotationToComputeCoPOccupancy;
   private YoDouble yoVerificationInlineCoPHistoryStdDev;
   private YoDouble yoVerificationTransverseCoPHistoryStdDev;
   private YoDouble yoCroppingFootDropThresholdForCrop;

   // ---------------- Balance / ICP / Momentum weights ----------------
   private YoBoolean yoComputeSplitFractionsFromPositions;
   private YoDouble yoEllipticICPErrorForMomentumRecovery;
   private YoDouble yoControllerThresholdForStuck;
   private YoDouble yoCaptureKi;
   private YoDouble yoCaptureIntegralLeakRatio;
   private YoDouble yoLinearMomentumRateWeightX;
   private YoDouble yoLinearMomentumRateWeightY;
   private YoDouble yoRecoveryLinearMomentumRateWeightX;
   private YoDouble yoRecoveryLinearMomentumRateWeightY;

   // ---------------- MomentumOptimizationSettings ----------------
   private YoDouble yoChestAngularWeightX;
   private YoDouble yoChestAngularWeightY;

   // ---------------- Estimator (AtlasHeadPoseEstimator) ----------------
   private YoDouble yoAngularVelocityVariance;
   private YoDouble yoLinearAccelerationVariance;
   private YoDouble yoMagneticFieldVariance;
   private YoDouble yoPositionVariance;
   private YoDouble yoHeadAngularAccelerationVariance;
   private YoDouble yoHeadLinearAccelerationVariance;
   private YoDouble yoLinearAccelerationBiasVariance;

   public SimulationParameters(YoRegistry registry)
   {
      super(registry);
      initTransitions();
      initHighLevelHumanoidControllerFactory();
      initFootGains();
      initHighLevelControlManagerFactory_ComHeightAndPelvis();
      initCenterOfMassHeightControlState();
      initFeetManagerAndDetectors();
      initBalanceManagerAndWalkingCoP();
      initWalkingControllerAndICP();
      initLinearMomentumWeights();
      initEstimatorThread();
      initMomentumOptimizationSettings();
   }

   // --------------------------------------------------------------------------
   //  Section: Transitions (toWalking / exitWalking)
   // --------------------------------------------------------------------------
   private void initTransitions()
   {
      YoRegistry toWalking = findReg("toWalkingSmoothTransitionControllerState");
      yoToWalkingEnableTimeBasedTransition = setAnyBoolean(toWalking, new String[] {"toWalkingEnableTimeBasedTransition"}, false);

      YoRegistry exitWalking = findReg("exitWalkingSmoothTransitionControllerState");
      yoExitWalkingEnableTimeBasedTransition = setAnyBoolean(exitWalking, new String[] {"exitWalkingEnableTimeBasedTransition"}, true);
   }

   // --------------------------------------------------------------------------
   //  Section: HighLevelHumanoidControllerFactory -> JointspaceGains / RigidBodyGains
   // --------------------------------------------------------------------------
   private void initHighLevelHumanoidControllerFactory()
   {
      YoRegistry factory = findReg("HighLevelHumanoidControllerFactory");

      // JointspaceGains
      YoRegistry jointspace = findReg(factory, "JointspaceGains");
      yoZetaSpineJoints = setAnyDouble(jointspace, new String[] {"zetaSpineJoints"}, 0.3);
      yoMaximumFeedbackSpineJoints = setAnyDouble(jointspace, new String[] {"maximumFeedbackSpineJoints"}, 10.0);
      yoMaximumFeedbackRateSpineJoints = setAnyDouble(jointspace, new String[] {"maximumFeedbackRateSpineJoints"}, 100.0);
      yoZetaNeckJoints = setAnyDouble(jointspace, new String[] {"zetaNeckJoints"}, 0.4);
      yoMaximumFeedbackNeckJoints = setAnyDouble(jointspace, new String[] {"maximumFeedbackNeckJoints"}, 6.0);
      yoMaximumFeedbackRateNeckJoints = setAnyDouble(jointspace, new String[] {"maximumFeedbackRateNeckJoints"}, 60.0);
      yoKpArmJoints = setAnyDouble(jointspace, new String[] {"kpArmJoints"}, 100.0);
      yoZetaArmJoints = setAnyDouble(jointspace, new String[] {"zetaArmJoints"}, 0.1);
      yoMaximumFeedbackArmJoints = setAnyDouble(jointspace, new String[] {"maximumFeedbackArmJoints"}, 20.0);
      yoMaximumFeedbackRateArmJoints = setAnyDouble(jointspace, new String[] {"maximumFeedbackRateArmJoints"}, 700.0);

      // RigidBodyGains
      YoRegistry rigid = findReg(factory, "RigidBodyGains");
      yoZetaXYChestOrientation = setAnyDouble(rigid, new String[] {"zetaXYChestOrientation"}, 0.5);
      yoZetaZChestOrientation = setAnyDouble(rigid, new String[] {"zetaZChestOrientation"}, 0.22);
      yoMaximumFeedbackChestOrientation = setAnyDouble(rigid, new String[] {"maximumFeedbackChestOrientation"}, 6.0);
      yoMaximumFeedbackRateChestOrientation = setAnyDouble(rigid, new String[] {"maximumFeedbackRateChestOrientation"}, 60.0);
      yoZetaXYZHeadOrientation = setAnyDouble(rigid, new String[] {"zetaXYZHeadOrientation"}, 0.4);
      yoMaximumFeedbackHeadOrientation = setAnyDouble(rigid, new String[] {"maximumFeedbackHeadOrientation"}, 6.0);
      yoMaximumFeedbackRateHeadOrientation = setAnyDouble(rigid, new String[] {"maximumFeedbackRateHeadOrientation"}, 60.0);
      yoKpXYZHandOrientation = setAnyDouble(rigid, new String[] {"kpXYZHandOrientation"}, 40.0);
      yoZetaXYZHandOrientation = setAnyDouble(rigid, new String[] {"zetaXYZHandOrientation"}, 0.0);
      yoMaximumFeedbackHandOrientation = setAnyDouble(rigid, new String[] {"maximumFeedbackHandOrientation"}, 10.0);
      yoMaximumFeedbackRateHandOrientation = setAnyDouble(rigid, new String[] {"maximumFeedbackRateHandOrientation"}, 100.0);
      yoZetaXYPelvisOrientation = setAnyDouble(rigid, new String[] {"zetaXYPelvisOrientation"}, 0.2);
      yoZetaZPelvisOrientation = setAnyDouble(rigid, new String[] {"zetaZPelvisOrientation"}, 0.5);
      yoMaximumFeedbackPelvisOrientation = setAnyDouble(rigid, new String[] {"maximumFeedbackPelvisOrientation"}, 12.0);
      yoMaximumFeedbackRatePelvisOrientation = setAnyDouble(rigid, new String[] {"maximumFeedbackRatePelvisOrientation"}, 180.0);
      yoKpXYZHandPosition = setAnyDouble(rigid, new String[] {"kpXYZHandPosition"}, 40.0);
      yoZetaXYZHandPosition = setAnyDouble(rigid, new String[] {"zetaXYZHandPosition"}, 0.0);
      yoMaximumFeedbackHandPosition = setAnyDouble(rigid, new String[] {"maximumFeedbackHandPosition"}, 10.0);
      yoMaximumFeedbackRateHandPosition = setAnyDouble(rigid, new String[] {"maximumFeedbackRateHandPosition"}, 100.0);
   }

   // --------------------------------------------------------------------------
   //  Section: HighLevelHumanoidControllerFactory -> FootGains
   // --------------------------------------------------------------------------
   private void initFootGains()
   {
      YoRegistry factory = findReg("HighLevelHumanoidControllerFactory");
      YoRegistry foot = findReg(factory, "FootGains");

      yoMaximumFeedbackSwingFootPosition = setAnyDouble(foot, new String[] {"maximumFeedbackSwingFootPosition"}, 20.0);
      yoMaximumFeedbackRateSwingFootPosition = setAnyDouble(foot, new String[] {"maximumFeedbackRateSwingFootPosition"}, 300.0);
      yoMaximumFeedbackSwingFootOrientation = setAnyDouble(foot, new String[] {"maximumFeedbackSwingFootOrientation"}, 100.0);
      yoMaximumFeedbackRateSwingFootOrientation = setAnyDouble(foot, new String[] {"maximumFeedbackRateSwingFootOrientation"}, 1500.0);
      yoZetaXYHoldFootPosition = setAnyDouble(foot, new String[] {"zetaXYHoldFootPosition"}, 0.2);
      yoMaximumFeedbackHoldFootPosition = setAnyDouble(foot, new String[] {"maximumFeedbackHoldFootPosition"}, 6.0);
      yoMaximumFeedbackRateHoldFootPosition = setAnyDouble(foot, new String[] {"maximumFeedbackRateHoldFootPosition"}, 150.0);
      yoKpXYHoldFootOrientation = setAnyDouble(foot, new String[] {"kpXYHoldFootOrientation"}, 100.0);
      yoKpZHoldFootOrientation = setAnyDouble(foot, new String[] {"kpZHoldFootOrientation"}, 100.0);
      yoZetaXYHoldFootOrientation = setAnyDouble(foot, new String[] {"zetaXYHoldFootOrientation"}, 0.2);
      yoZetaZHoldFootOrientation = setAnyDouble(foot, new String[] {"zetaZHoldFootOrientation"}, 0.2);
      yoMaximumFeedbackHoldFootOrientation = setAnyDouble(foot, new String[] {"maximumFeedbackHoldFootOrientation"}, 100.0);
      yoMaximumFeedbackRateHoldFootOrientation = setAnyDouble(foot, new String[] {"maximumFeedbackRateHoldFootOrientation"}, 1500.0);
      yoMaximumFeedbackToeOffFootPosition = setAnyDouble(foot, new String[] {"maximumFeedbackToeOffFootPosition"}, 6.0);
      yoMaximumFeedbackRateToeOffFootPosition = setAnyDouble(foot, new String[] {"maximumFeedbackRateToeOffFootPosition"}, 150.0);
      yoMaximumFeedbackToeOffFootOrientation = setAnyDouble(foot, new String[] {"maximumFeedbackToeOffFootOrientation"}, 100.0);
      yoMaximumFeedbackRateToeOffFootOrientation = setAnyDouble(foot, new String[] {"maximumFeedbackRateToeOffFootOrientation"}, 1500.0);
   }

   // --------------------------------------------------------------------------
   //  Section: HighLevelControlManagerFactory -> CoM Height / Pelvis height
   // --------------------------------------------------------------------------
   private void initHighLevelControlManagerFactory_ComHeightAndPelvis()
   {
      YoRegistry factory = findReg("HighLevelHumanoidControllerFactory");
      YoRegistry hlcmf = findReg(factory, "HighLevelControlManagerFactory");
      YoRegistry comHeightMgr = findReg(hlcmf, "CenterOfMassHeightManager");
      YoRegistry pelvisState = findReg(comHeightMgr, "PelvisHeightControlState");

      yoPelvisHeightControlStateDefaultHeight = setAnyDouble(pelvisState, new String[] {"defaultHeight", "PelvisHeightControlStateDefaultHeight"}, 0.8);
      yoPelvisHeightControlStateOffsetTrajectoryTime = setAnyDouble(pelvisState,
                                                                    new String[] {"offsetTrajectoryTime", "PelvisHeightControlStateOffsetTrajectoryTime"},
                                                                    0.5);
      yoPelvisHeightControlStateMaxDistanceAnklePelvis = setAnyDouble(pelvisState,
                                                                      new String[] {"maxDistanceAnklePelvis", "PelvisHeightControlStateMaxDistanceAnklePelvis"},
                                                                      0.9);
   }

   // --------------------------------------------------------------------------
   //  Section: CenterOfMassHeightControlState -> LookAheadCoM / BetterLookAheadCoM
   // --------------------------------------------------------------------------
   private void initCenterOfMassHeightControlState()
   {
      YoRegistry factory = findReg("HighLevelHumanoidControllerFactory");
      YoRegistry comState = findReg(factory, "CenterOfMassHeightControlState");

      YoRegistry lookAhead = findReg(comState, "LookAheadCoMHeightTrajectoryGenerator");
      yoProcessGoHome = setAnyBoolean(lookAhead, new String[] {"processGoHome", "ProcessGoHome"}, true);

      YoRegistry betterLookAhead = findReg(comState, "BetterLookAheadCoMHeightTrajectoryGenerator");
      yoProcessGoHome = setAnyBoolean(betterLookAhead, new String[] {"processGoHome", "ProcessGoHome"}, true);
   }

   // --------------------------------------------------------------------------
   //  Section: FeetManager (Toe-off, rotation detectors, support parameters, touchdown, verification, velocity, cropping)
   // --------------------------------------------------------------------------
   private void initFeetManagerAndDetectors()
   {
      YoRegistry factory = findReg("HighLevelHumanoidControllerFactory");
      YoRegistry hlcmf = findReg(factory, "HighLevelControlManagerFactory");
      YoRegistry feet = findReg(hlcmf, "FeetManager");

      // Toe-off trigger threshold
      YoRegistry toeOffMgr = findReg(feet, "GeometricToeOffManager", "GeometricToeOffManagerRegistry");
      YoRegistry legLimits = findReg(toeOffMgr, "LegJointLimitsInspector");
      yoAnkleLowerLimitToTriggerToeOff = setAnyDouble(legLimits, new String[] {"ankleLowerLimitToTriggerToeOff"}, -0.94); // -54 deg

      // Rotation detector
      YoRegistry rotationParams = findReg(feet, "FootRotationDetectorParameters");
      yoOmegaThresholdForEstimation = setAnyDouble(rotationParams, new String[] {"omegaThresholdForEstimation"}, 2.0);
      yoRotationThreshold = setAnyDouble(rotationParams, new String[] {"rotationThreshold"}, 0.05);

      // Support state parameters
      YoRegistry supportParams = findReg(feet, "SupportStateParameters");
      yoDampFootRotations = setAnyBoolean(supportParams, new String[] {"dampFootRotations"}, true);
      yoFootDamping = setAnyDouble(supportParams, new String[] {"footDamping"}, 20.0);

      // Swing touchdown & verification/velocity thresholds
      yoFootSwingTouchdownVelocityZ = setAnyDouble(feet, new String[] {"FootSwingTouchdownVelocityZ"}, -0.25);
      yoFootSwingTouchdownAccelerationZ = setAnyDouble(feet, new String[] {"FootSwingTouchdownAccelerationZ"}, -2.0);
      yoVerificationPerpendicularCoPErrorThreshold = setAnyDouble(feet, new String[] {"Verification_PerpendicularCoPErrorThreshold"}, 0.005);
      yoVerificationDistanceFromLineToComputeDesiredCoPOccupancy = setAnyDouble(feet,
                                                                                new String[] {"Verification_DistanceFromLineToComputeDesiredCoPOccupancy"},
                                                                                0.005);

      yoVelocityOmegaMagnitudeThreshold = setAnyDouble(feet, new String[] {"Velocity_omegaMagnitudeThreshold"}, 0.5);
      yoVelocityRotationAngleDecayBreakFrequency = setAnyDouble(feet, new String[] {"Velocity_rotationAngleDecayBreakFrequency"}, 5.0);
      yoVelocityRotationAngleThreshold = setAnyDouble(feet, new String[] {"Velocity_rotationAngleThreshold"}, 0.15);

      yoCroppingThresholdForCoPRegionOccupancy = setAnyInt(feet, new String[] {"Cropping_ThresholdForCoPRegionOccupancy"}, 4);
      yoCroppingDistanceFromLineOfRotationToComputeCoPOccupancy = setAnyDouble(feet,
                                                                               new String[] {"Cropping_DistanceFromLineOfRotationToComputeCoPOccupancy"},
                                                                               0.005);
      yoVerificationInlineCoPHistoryStdDev = setAnyDouble(feet, new String[] {"Verification_InlineCoPHistoryStdDev"}, 0.002);
      yoVerificationTransverseCoPHistoryStdDev = setAnyDouble(feet, new String[] {"Verification_TransverseCoPHistoryStdDev"}, 5.0E-4);
      yoCroppingFootDropThresholdForCrop = setAnyDouble(feet, new String[] {"Cropping_FootDropThresholdForCrop"}, 0.02);
   }

   // --------------------------------------------------------------------------
   //  Section: BalanceManager / WalkingCoPTrajectoryGenerator
   // --------------------------------------------------------------------------
   private void initBalanceManagerAndWalkingCoP()
   {
      YoRegistry factory = findReg("HighLevelHumanoidControllerFactory");
      YoRegistry hlcmf = findReg(factory, "HighLevelControlManagerFactory");
      YoRegistry balance = findReg(hlcmf, "BalanceManager");

      yoEllipticICPErrorForMomentumRecovery = setAnyDouble(balance, new String[] {"ellipticICPErrorForMomentumRecovery"}, Double.POSITIVE_INFINITY);

      YoRegistry walkingCoP = findReg(balance, "WalkingCoPTrajectoryGenerator");
      yoComputeSplitFractionsFromPositions = setAnyBoolean(walkingCoP, new String[] {"computeSplitFractionsFromPositions"}, true);
   }

   // --------------------------------------------------------------------------
   //  Section: WalkingControllerState -> LinearMomentumRateControlModule -> ICP
   // --------------------------------------------------------------------------
   private void initWalkingControllerAndICP()
   {
      YoRegistry hhcm = findReg("HumanoidHighLevelControllerManager");
      YoRegistry walkingState = findReg(hhcm, "WalkingControllerState");
      YoRegistry lmrcm = findReg(walkingState, "LinearMomentumRateControlModule");

      // ICP Optimization Controller
      YoRegistry icpOpt = findReg(lmrcm, "ICPOptimizationController");
      yoControllerThresholdForStuck = setAnyDouble(icpOpt, new String[] {"controllerThresholdForStuck"}, 0.12);

      // ICP Controller
      YoRegistry icp = findReg(lmrcm, "ICPController");
      yoCaptureKi = setAnyDouble(icp, new String[] {"captureKi"}, 1.5);
      yoCaptureIntegralLeakRatio = setAnyDouble(icp, new String[] {"captureIntegralLeakRatio"}, 0.98);
      yoControllerThresholdForStuck = setAnyDouble(icp, new String[] {"controllerThresholdForStuck"}, 0.135);
   }

   // --------------------------------------------------------------------------
   //  Section: Linear Momentum Weights
   // --------------------------------------------------------------------------
   private void initLinearMomentumWeights()
   {
      YoRegistry hhcm = findReg("HumanoidHighLevelControllerManager");
      YoRegistry walkingState = findReg(hhcm, "WalkingControllerState");
      YoRegistry lmrcm = findReg(walkingState, "LinearMomentumRateControlModule");

      yoLinearMomentumRateWeightX = setAnyDouble(lmrcm, new String[] {"linearMomentumRateWeightX", "LinearMomentumRateWeightX"}, 0.1);
      yoLinearMomentumRateWeightY = setAnyDouble(lmrcm, new String[] {"linearMomentumRateWeightY", "LinearMomentumRateWeightY"}, 0.1);
      yoRecoveryLinearMomentumRateWeightX = setAnyDouble(lmrcm, new String[] {"recoveryLinearMomentumRateWeightX", "RecoveryLinearMomentumRateWeightX"}, 0.3);
      yoRecoveryLinearMomentumRateWeightY = setAnyDouble(lmrcm, new String[] {"recoveryLinearMomentumRateWeightY", "RecoveryLinearMomentumRateWeightY"}, 0.3);
   }

   // --------------------------------------------------------------------------
   //  Section: Estimator Thread (AtlasHeadPoseEstimator)
   // --------------------------------------------------------------------------
   private void initEstimatorThread()
   {
      YoRegistry estimator = findReg("DRCEstimatorThread");
      YoRegistry headPose = findReg(estimator, "AtlasHeadPoseEstimator");

      yoAngularVelocityVariance = setAnyDouble(headPose, new String[] {"angularVelocityVariance", "AngularVelocityVariance"}, 3.0E-6);
      yoLinearAccelerationVariance = setAnyDouble(headPose, new String[] {"linearAccelerationVariance", "LinearAccelerationVariance"}, 10.0);
      yoMagneticFieldVariance = setAnyDouble(headPose, new String[] {"magneticFieldVariance", "MagneticFieldVariance"}, 0.001);
      yoPositionVariance = setAnyDouble(headPose, new String[] {"positionVariance", "PositionVariance"}, 1.0E-8);
      yoHeadAngularAccelerationVariance = setAnyDouble(headPose, new String[] {"headAngularAccelerationVariance", "HeadAngularAccelerationVariance"}, 10.0);
      yoHeadLinearAccelerationVariance = setAnyDouble(headPose, new String[] {"headLinearAccelerationVariance", "HeadLinearAccelerationVariance"}, 10.0);
      yoLinearAccelerationBiasVariance = setAnyDouble(headPose, new String[] {"linearAccelerationBiasVariance", "LinearAccelerationBiasVariance"}, 10.0);
   }

   // --------------------------------------------------------------------------
   //  Section: MomentumOptimizationSettings
   // --------------------------------------------------------------------------
   private void initMomentumOptimizationSettings()
   {
      // Might be under HighLevelHumanoidControllerFactory OR directly under root (XML differences)
      YoRegistry factory = findReg("HighLevelHumanoidControllerFactory");
      YoRegistry momentum = findRegIfPresent(factory, "MomentumOptimizationSettings");
      if (momentum == null)
         momentum = findReg("MomentumOptimizationSettings");

      yoChestAngularWeightX = setAnyDouble(momentum, new String[] {"ChestAngularWeightX", "chestAngularWeightX"}, 30.0);
      yoChestAngularWeightY = setAnyDouble(momentum, new String[] {"ChestAngularWeightY", "chestAngularWeightY"}, 30.0);
   }

   // ---------- Optional Getters (add as needed for UI/tuning) ----------
   public YoDouble getYoKpArmJoints()
   {
      return yoKpArmJoints;
   }

   public YoDouble getYoZetaArmJoints()
   {
      return yoZetaArmJoints;
   }

   public YoDouble getYoCaptureKi()
   {
      return yoCaptureKi;
   }

   public YoDouble getYoCaptureIntegralLeakRatio()
   {
      return yoCaptureIntegralLeakRatio;
   }

   public YoBoolean getYoDampFootRotations()
   {
      return yoDampFootRotations;
   }
}

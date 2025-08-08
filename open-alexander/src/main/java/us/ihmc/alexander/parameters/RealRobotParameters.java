package us.ihmc.alexander.parameters;

import us.ihmc.scs2.sessionVisualizer.jfx.yoRobot.YoRobotFX;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Parameters initializer generated from the provided XML structure.
 * - Uses alias helpers to tolerate XML vs YoVariable naming differences (case/prefix).
 * - Groups initialization by registry path for readability.
 * - Relies on AbstractRobotParameters for registry/variable helper methods.
 */
public class RealRobotParameters extends AbstractRobotParameters
{
   // ---------- Transitions ----------
   private YoBoolean toWalkingEnableTimeBasedTransition;
   private YoBoolean exitWalkingEnableTimeBasedTransition;

   // ---------- JointspaceGains ----------
   private YoDouble zetaSpineJoints;
   private YoDouble maximumFeedbackSpineJoints;
   private YoDouble maximumFeedbackRateSpineJoints;
   private YoDouble zetaNeckJoints;
   private YoDouble maximumFeedbackNeckJoints;
   private YoDouble maximumFeedbackRateNeckJoints;
   private YoDouble kpArmJoints;
   private YoDouble zetaArmJoints;
   private YoDouble maximumFeedbackArmJoints;
   private YoDouble maximumFeedbackRateArmJoints;

   // ---------- RigidBodyGains ----------
   private YoDouble zetaXYChestOrientation;
   private YoDouble zetaZChestOrientation;
   private YoDouble maximumFeedbackChestOrientation;
   private YoDouble maximumFeedbackRateChestOrientation;
   private YoDouble zetaXYZHeadOrientation;
   private YoDouble maximumFeedbackHeadOrientation;
   private YoDouble maximumFeedbackRateHeadOrientation;
   private YoDouble kpXYZHandOrientation;
   private YoDouble zetaXYZHandOrientation;
   private YoDouble maximumFeedbackHandOrientation;
   private YoDouble maximumFeedbackRateHandOrientation;
   private YoDouble zetaXYPelvisOrientation;
   private YoDouble zetaZPelvisOrientation;
   private YoDouble maximumFeedbackPelvisOrientation;
   private YoDouble maximumFeedbackRatePelvisOrientation;
   private YoDouble kpXYZHandPosition;
   private YoDouble zetaXYZHandPosition;
   private YoDouble maximumFeedbackHandPosition;
   private YoDouble maximumFeedbackRateHandPosition;

   // ---------- FootGains ----------
   private YoDouble maximumFeedbackSwingFootPosition;
   private YoDouble maximumFeedbackRateSwingFootPosition;
   private YoDouble maximumFeedbackSwingFootOrientation;
   private YoDouble maximumFeedbackRateSwingFootOrientation;
   private YoDouble zetaXYHoldFootPosition;
   private YoDouble maximumFeedbackHoldFootPosition;
   private YoDouble maximumFeedbackRateHoldFootPosition;
   private YoDouble kpXYHoldFootOrientation;
   private YoDouble kpZHoldFootOrientation;
   private YoDouble zetaXYHoldFootOrientation;
   private YoDouble zetaZHoldFootOrientation;
   private YoDouble maximumFeedbackHoldFootOrientation;
   private YoDouble maximumFeedbackRateHoldFootOrientation;
   private YoDouble maximumFeedbackToeOffFootPosition;
   private YoDouble maximumFeedbackRateToeOffFootPosition;
   private YoDouble maximumFeedbackToeOffFootOrientation;
   private YoDouble maximumFeedbackRateToeOffFootOrientation;

   // ---------- Pelvis / CoM height ----------
   private YoDouble pelvisHeightDefaultHeight;
   private YoDouble pelvisHeightOffsetTrajectoryTime;
   private YoDouble pelvisHeightMaxDistanceAnklePelvis;
   private YoBoolean processGoHome_LookAhead;
   private YoBoolean processGoHome_BetterLookAhead;

   // ---------- FeetManager ----------
   private YoDouble ankleLowerLimitToTriggerToeOff;
   private YoDouble omegaThresholdForEstimation;
   private YoDouble rotationThreshold;
   private YoBoolean dampFootRotations;
   private YoDouble footDamping;
   private YoDouble footSwingTouchdownVelocityZ;
   private YoDouble footSwingTouchdownAccelerationZ;
   private YoDouble verificationPerpendicularCoPErrorThreshold;
   private YoDouble verificationDistanceFromLineToComputeDesiredCoPOccupancy;
   private YoDouble velocityOmegaMagnitudeThreshold;
   private YoDouble velocityRotationAngleDecayBreakFrequency;
   private YoDouble velocityRotationAngleThreshold;
   private YoInteger croppingThresholdForCoPRegionOccupancy;
   private YoDouble croppingDistanceFromLineOfRotationToComputeCoPOccupancy;
   private YoDouble verificationInlineCoPHistoryStdDev;
   private YoDouble verificationTransverseCoPHistoryStdDev;
   private YoDouble croppingFootDropThresholdForCrop;

   // ---------- Balance / CoP ----------
   private YoBoolean computeSplitFractionsFromPositions;
   private YoDouble ellipticICPErrorForMomentumRecovery;

   // ---------- WalkingController / ICP ----------
   private YoDouble controllerThresholdForStuck_ICPOpt;
   private YoDouble captureKi;
   private YoDouble captureIntegralLeakRatio;
   private YoDouble controllerThresholdForStuck_ICP;

   // ---------- Linear Momentum Weights ----------
   private YoDouble linearMomentumRateWeightX;
   private YoDouble linearMomentumRateWeightY;
   private YoDouble recoveryLinearMomentumRateWeightX;
   private YoDouble recoveryLinearMomentumRateWeightY;

   // ---------- MomentumOptimizationSettings ----------
   private YoDouble chestAngularWeightX;
   private YoDouble chestAngularWeightY;

   // ---------- Estimator (AtlasHeadPoseEstimator) ----------
   private DoubleParameter angularVelocityVariance;
   private DoubleParameter linearAccelerationVariance;
   private DoubleParameter magneticFieldVariance;
   private DoubleParameter positionVariance;
   private DoubleParameter headAngularAccelerationVariance;
   private DoubleParameter headLinearAccelerationVariance;
   private DoubleParameter linearAccelerationBiasVariance;
//   private BooleanParameter zeroEstimatedRootYawAtInitialization;

   public RealRobotParameters(YoRegistry registry)
   {
      super(registry);

      // Top path in XML:
      // DRCControllerThread -> DRCMomentumBasedController -> HumanoidHighLevelControllerManager
      
//      YoRegistry DRCControllerThreadRoot = findReg(root, "DRCControllerThread");
      if (root.getName() == "DRCControllerThread")
      {
         YoRegistry DRCMomentumBasedControllerRegistry = findReg(root, "DRCMomentumBasedController");
         YoRegistry HumanoidHighLevelControllerManagerRegistry = findReg(DRCMomentumBasedControllerRegistry, "HumanoidHighLevelControllerManager");

         initTransitions(HumanoidHighLevelControllerManagerRegistry);
         initHighLevelHumanoidControllerFactory(HumanoidHighLevelControllerManagerRegistry);
         initHighLevelControlManagerFactory(HumanoidHighLevelControllerManagerRegistry);
         initWalkingController(HumanoidHighLevelControllerManagerRegistry);
      }
//      YoRegistry DRCEstimatorThreadRoot = findReg(root, "DRCEstimatorThread");
      if (root.getName() == "DRCEstimatorThread")
      {
         initEstimatorThread(root);
                  
      }
      

    
   }

   // ---------------- init sections ----------------

   private void initTransitions(YoRegistry hhcm)
   {
      YoRegistry toWalking = findReg(hhcm, "toWalkingSmoothTransitionControllerState");
      toWalkingEnableTimeBasedTransition =
            setAnyBoolean(toWalking, new String[]{"toWalkingEnableTimeBasedTransition"}, false);

      YoRegistry exitWalking = findReg(hhcm, "exitWalkingSmoothTransitionControllerState");
      exitWalkingEnableTimeBasedTransition =
            setAnyBoolean(exitWalking, new String[]{"exitWalkingEnableTimeBasedTransition"}, true);
   }

   private void initHighLevelHumanoidControllerFactory(YoRegistry hhcm)
   {
      YoRegistry factory = findReg(hhcm, "HighLevelHumanoidControllerFactory");

      // JointspaceGains
      YoRegistry jointspace = findReg(factory, "JointspaceGains");
      zetaSpineJoints = setAnyDouble(jointspace, new String[]{"zetaSpineJoints"}, 0.3);
      maximumFeedbackSpineJoints = setAnyDouble(jointspace, new String[]{"maximumFeedbackSpineJoints"}, 10.0);
      maximumFeedbackRateSpineJoints = setAnyDouble(jointspace, new String[]{"maximumFeedbackRateSpineJoints"}, 100.0);
      zetaNeckJoints = setAnyDouble(jointspace, new String[]{"zetaNeckJoints"}, 0.4);
      maximumFeedbackNeckJoints = setAnyDouble(jointspace, new String[]{"maximumFeedbackNeckJoints"}, 6.0);
      maximumFeedbackRateNeckJoints = setAnyDouble(jointspace, new String[]{"maximumFeedbackRateNeckJoints"}, 60.0);
      kpArmJoints = setAnyDouble(jointspace, new String[]{"kpArmJoints"}, 100.0);
      zetaArmJoints = setAnyDouble(jointspace, new String[]{"zetaArmJoints"}, 0.1);
      maximumFeedbackArmJoints = setAnyDouble(jointspace, new String[]{"maximumFeedbackArmJoints"}, 20.0);
      maximumFeedbackRateArmJoints = setAnyDouble(jointspace, new String[]{"maximumFeedbackRateArmJoints"}, 700.0);

      // RigidBodyGains
      YoRegistry rigid = findReg(factory, "RigidBodyGains");
      zetaXYChestOrientation = setAnyDouble(rigid, new String[]{"zetaXYChestOrientation"}, 0.5);
      zetaZChestOrientation = setAnyDouble(rigid, new String[]{"zetaZChestOrientation"}, 0.22);
      maximumFeedbackChestOrientation = setAnyDouble(rigid, new String[]{"maximumFeedbackChestOrientation"}, 6.0);
      maximumFeedbackRateChestOrientation = setAnyDouble(rigid, new String[]{"maximumFeedbackRateChestOrientation"}, 60.0);
      zetaXYZHeadOrientation = setAnyDouble(rigid, new String[]{"zetaXYZHeadOrientation"}, 0.4);
      maximumFeedbackHeadOrientation = setAnyDouble(rigid, new String[]{"maximumFeedbackHeadOrientation"}, 6.0);
      maximumFeedbackRateHeadOrientation = setAnyDouble(rigid, new String[]{"maximumFeedbackRateHeadOrientation"}, 60.0);
      kpXYZHandOrientation = setAnyDouble(rigid, new String[]{"kpXYZHandOrientation"}, 40.0);
      zetaXYZHandOrientation = setAnyDouble(rigid, new String[]{"zetaXYZHandOrientation"}, 0.0);
      maximumFeedbackHandOrientation = setAnyDouble(rigid, new String[]{"maximumFeedbackHandOrientation"}, 10.0);
      maximumFeedbackRateHandOrientation = setAnyDouble(rigid, new String[]{"maximumFeedbackRateHandOrientation"}, 100.0);
      zetaXYPelvisOrientation = setAnyDouble(rigid, new String[]{"zetaXYPelvisOrientation"}, 0.2);
      zetaZPelvisOrientation = setAnyDouble(rigid, new String[]{"zetaZPelvisOrientation"}, 0.5);
      maximumFeedbackPelvisOrientation = setAnyDouble(rigid, new String[]{"maximumFeedbackPelvisOrientation"}, 12.0);
      maximumFeedbackRatePelvisOrientation = setAnyDouble(rigid, new String[]{"maximumFeedbackRatePelvisOrientation"}, 180.0);
      kpXYZHandPosition = setAnyDouble(rigid, new String[]{"kpXYZHandPosition"}, 40.0);
      zetaXYZHandPosition = setAnyDouble(rigid, new String[]{"zetaXYZHandPosition"}, 0.0);
      maximumFeedbackHandPosition = setAnyDouble(rigid, new String[]{"maximumFeedbackHandPosition"}, 10.0);
      maximumFeedbackRateHandPosition = setAnyDouble(rigid, new String[]{"maximumFeedbackRateHandPosition"}, 100.0);

      // FootGains
      YoRegistry foot = findReg(factory, "FootGains");
      maximumFeedbackSwingFootPosition = setAnyDouble(foot, new String[]{"maximumFeedbackSwingFootPosition"}, 20.0);
      maximumFeedbackRateSwingFootPosition = setAnyDouble(foot, new String[]{"maximumFeedbackRateSwingFootPosition"}, 300.0);
      maximumFeedbackSwingFootOrientation = setAnyDouble(foot, new String[]{"maximumFeedbackSwingFootOrientation"}, 100.0);
      maximumFeedbackRateSwingFootOrientation = setAnyDouble(foot, new String[]{"maximumFeedbackRateSwingFootOrientation"}, 1500.0);
      zetaXYHoldFootPosition = setAnyDouble(foot, new String[]{"zetaXYHoldFootPosition"}, 0.2);
      maximumFeedbackHoldFootPosition = setAnyDouble(foot, new String[]{"maximumFeedbackHoldFootPosition"}, 6.0);
      maximumFeedbackRateHoldFootPosition = setAnyDouble(foot, new String[]{"maximumFeedbackRateHoldFootPosition"}, 150.0);
      kpXYHoldFootOrientation = setAnyDouble(foot, new String[]{"kpXYHoldFootOrientation"}, 100.0);
      kpZHoldFootOrientation = setAnyDouble(foot, new String[]{"kpZHoldFootOrientation"}, 100.0);
      zetaXYHoldFootOrientation = setAnyDouble(foot, new String[]{"zetaXYHoldFootOrientation"}, 0.2);
      zetaZHoldFootOrientation = setAnyDouble(foot, new String[]{"zetaZHoldFootOrientation"}, 0.2);
      maximumFeedbackHoldFootOrientation = setAnyDouble(foot, new String[]{"maximumFeedbackHoldFootOrientation"}, 100.0);
      maximumFeedbackRateHoldFootOrientation = setAnyDouble(foot, new String[]{"maximumFeedbackRateHoldFootOrientation"}, 1500.0);
      maximumFeedbackToeOffFootPosition = setAnyDouble(foot, new String[]{"maximumFeedbackToeOffFootPosition"}, 6.0);
      maximumFeedbackRateToeOffFootPosition = setAnyDouble(foot, new String[]{"maximumFeedbackRateToeOffFootPosition"}, 150.0);
      maximumFeedbackToeOffFootOrientation = setAnyDouble(foot, new String[]{"maximumFeedbackToeOffFootOrientation"}, 100.0);
      maximumFeedbackRateToeOffFootOrientation = setAnyDouble(foot, new String[]{"maximumFeedbackRateToeOffFootOrientation"}, 1500.0);

      // MomentumOptimizationSettings
      YoRegistry momentum = findReg(factory, "MomentumOptimizationSettings");
      chestAngularWeightX = setAnyDouble(momentum, new String[]{"ChestAngularWeightX", "chestAngularWeightX"}, 30.0);
      chestAngularWeightY = setAnyDouble(momentum, new String[]{"ChestAngularWeightY", "chestAngularWeightY"}, 30.0);
   }

   private void initHighLevelControlManagerFactory(YoRegistry hhcm)
   {
      YoRegistry hlcmf = findReg(hhcm, "HighLevelControlManagerFactory");

      // CenterOfMassHeightManager / PelvisHeightControlState
      YoRegistry comHeightMgr = findReg(hlcmf, "CenterOfMassHeightManager");
      YoRegistry pelvis = findReg(comHeightMgr, "PelvisHeightControlState");
      pelvisHeightDefaultHeight = setAnyDouble(pelvis, new String[]{"PelvisHeightControlStateDefaultHeight", "defaultHeight"}, 0.8);
      pelvisHeightOffsetTrajectoryTime = setAnyDouble(pelvis, new String[]{"PelvisHeightControlStateOffsetTrajectoryTime", "offsetTrajectoryTime"}, 0.5);
      pelvisHeightMaxDistanceAnklePelvis = setAnyDouble(pelvis, new String[]{"PelvisHeightControlStateMaxDistanceAnklePelvis", "maxDistanceAnklePelvis"}, 0.9);

      // CenterOfMassHeightControlState -> {LookAhead, BetterLookAhead}
      YoRegistry comState = findReg(comHeightMgr, "CenterOfMassHeightControlState");
      YoRegistry lookAhead = findReg(comState, "LookAheadCoMHeightTrajectoryGenerator");
      processGoHome_LookAhead = setAnyBoolean(lookAhead, new String[]{"ProcessGoHome", "processGoHome"}, true);
      YoRegistry betterLookAhead = findReg(comState, "BetterLookAheadCoMHeightTrajectoryGenerator");
      processGoHome_BetterLookAhead = setAnyBoolean(betterLookAhead, new String[]{"ProcessGoHome", "processGoHome"}, true);

      // FeetManager
      YoRegistry feet = findReg(hlcmf, "FeetManager");

      // GeometricToeOffManager / LegJointLimitsInspector
      YoRegistry toeOff = findReg(feet, "GeometricToeOffManager", "GeometricToeOffManagerRegistry");
      YoRegistry legLimits = findReg(toeOff, "LegJointLimitsInspector");
      ankleLowerLimitToTriggerToeOff = setAnyDouble(legLimits, new String[]{"ankleLowerLimitToTriggerToeOff"}, -0.94);

      // FootRotationDetectorParameters
      YoRegistry rot = findReg(feet, "FootRotationDetectorParameters");
      omegaThresholdForEstimation = setAnyDouble(rot, new String[]{"omegaThresholdForEstimation"}, 2.0);
      rotationThreshold = setAnyDouble(rot, new String[]{"rotationThreshold"}, 0.05);

      // SupportStateParameters
      YoRegistry support = findReg(feet, "SupportStateParameters");
      dampFootRotations = setAnyBoolean(support, new String[]{"dampFootRotations"}, true);
      footDamping = setAnyDouble(support, new String[]{"footDamping"}, 20.0);

      // FeetManager (direct parameters)
      footSwingTouchdownVelocityZ = setAnyDouble(feet, new String[]{"FootSwingTouchdownVelocityZ"}, -0.25);
      footSwingTouchdownAccelerationZ = setAnyDouble(feet, new String[]{"FootSwingTouchdownAccelerationZ"}, -2.0);
      verificationPerpendicularCoPErrorThreshold = setAnyDouble(feet, new String[]{"Verification_PerpendicularCoPErrorThreshold"}, 0.005);
      verificationDistanceFromLineToComputeDesiredCoPOccupancy = setAnyDouble(feet,
                                                                              new String[]{"Verification_DistanceFromLineToComputeDesiredCoPOccupancy"}, 0.005);
      velocityOmegaMagnitudeThreshold = setAnyDouble(feet, new String[]{"Velocity_omegaMagnitudeThreshold"}, 0.5);
      velocityRotationAngleDecayBreakFrequency = setAnyDouble(feet, new String[]{"Velocity_rotationAngleDecayBreakFrequency"}, 5.0);
      velocityRotationAngleThreshold = setAnyDouble(feet, new String[]{"Velocity_rotationAngleThreshold"}, 0.15);
      croppingThresholdForCoPRegionOccupancy = setAnyInt(feet, new String[]{"Cropping_ThresholdForCoPRegionOccupancy"}, 4);
      croppingDistanceFromLineOfRotationToComputeCoPOccupancy = setAnyDouble(feet,
                                                                             new String[]{"Cropping_DistanceFromLineOfRotationToComputeCoPOccupancy"}, 0.005);
      verificationInlineCoPHistoryStdDev = setAnyDouble(feet, new String[]{"Verification_InlineCoPHistoryStdDev"}, 0.002);
      verificationTransverseCoPHistoryStdDev = setAnyDouble(feet, new String[]{"Verification_TransverseCoPHistoryStdDev"}, 5.0E-4);
      croppingFootDropThresholdForCrop = setAnyDouble(feet, new String[]{"Cropping_FootDropThresholdForCrop"}, 0.02);

      // BalanceManager
      YoRegistry balance = findReg(hlcmf, "BalanceManager");
      YoRegistry walkingCoP = findReg(balance, "WalkingCoPTrajectoryGenerator");
      computeSplitFractionsFromPositions = setAnyBoolean(walkingCoP, new String[]{"computeSplitFractionsFromPositions"}, true);
      ellipticICPErrorForMomentumRecovery = setAnyDouble(balance, new String[]{"ellipticICPErrorForMomentumRecovery"}, Double.POSITIVE_INFINITY);
   }

   private void initWalkingController(YoRegistry hhcm)
   {
      YoRegistry walking = findReg(hhcm, "WalkingControllerState");
      YoRegistry lmrcm = findReg(walking, "LinearMomentumRateControlModule");

      // ICPOptimizationController
      YoRegistry icpOpt = findReg(lmrcm, "ICPOptimizationController");
      controllerThresholdForStuck_ICPOpt = setAnyDouble(icpOpt, new String[]{"controllerThresholdForStuck"}, 0.12);

      // ICPController
      YoRegistry icp = findReg(lmrcm, "ICPController");
      captureKi = setAnyDouble(icp, new String[]{"captureKi"}, 1.5);
      captureIntegralLeakRatio = setAnyDouble(icp, new String[]{"captureIntegralLeakRatio"}, 0.98);
      controllerThresholdForStuck_ICP = setAnyDouble(icp, new String[]{"controllerThresholdForStuck"}, 0.135);

      // Linear momentum weights (variables live under lmrcm)
      linearMomentumRateWeightX = setAnyDouble(lmrcm, new String[]{"LinearMomentumRateWeightX", "linearMomentumRateWeightX"}, 0.1);
      linearMomentumRateWeightY = setAnyDouble(lmrcm, new String[]{"LinearMomentumRateWeightY", "linearMomentumRateWeightY"}, 0.1);
      recoveryLinearMomentumRateWeightX = setAnyDouble(lmrcm, new String[]{"RecoveryLinearMomentumRateWeightX", "recoveryLinearMomentumRateWeightX"}, 0.3);
      recoveryLinearMomentumRateWeightY = setAnyDouble(lmrcm, new String[]{"RecoveryLinearMomentumRateWeightY", "recoveryLinearMomentumRateWeightY"}, 0.3);
   }

   private void initEstimatorThread(YoRegistry root)
   {
//      YoRegistry estimator = findReg(root,"DRCEstimatorThread");
      YoRegistry AtlasHeadPoseEstimator = new YoRegistry("AtlasHeadPoseEstimator");
      root.addChild(AtlasHeadPoseEstimator);
      YoRegistry estimatorControllerRegistry;
      if (root.findRegistry("EstimatorController") != null)
      {
         estimatorControllerRegistry = root.findRegistry("EstimatorController");
      }
      else
      {
         estimatorControllerRegistry = new YoRegistry("EstimatorController");
         root.addChild(estimatorControllerRegistry);
      }

      YoRegistry DRCKinematicsBasedStateEstimator;
      if (estimatorControllerRegistry.findRegistry("DRCKinematicsBasedStateEstimator") != null)
      {
         DRCKinematicsBasedStateEstimator = estimatorControllerRegistry.findRegistry("DRCKinematicsBasedStateEstimator");
      }
      else
      {
         DRCKinematicsBasedStateEstimator = new YoRegistry("DRCKinematicsBasedStateEstimator");
         estimatorControllerRegistry.addChild(DRCKinematicsBasedStateEstimator);
      }
      YoRegistry IMUBasedPelvisRotationalStateUpdater;
      if (DRCKinematicsBasedStateEstimator.findRegistry("IMUBasedPelvisRotationalStateUpdater") != null)
      {
         IMUBasedPelvisRotationalStateUpdater = DRCKinematicsBasedStateEstimator.findRegistry("IMUBasedPelvisRotationalStateUpdater");
      }
      else
      {
         IMUBasedPelvisRotationalStateUpdater = new YoRegistry("IMUBasedPelvisRotationalStateUpdater");
         DRCKinematicsBasedStateEstimator.addChild(IMUBasedPelvisRotationalStateUpdater);   
      }          
      
      angularVelocityVariance = new DoubleParameter("AngularVelocityVariance", AtlasHeadPoseEstimator, 3.0E-6,0.0,1.0);
      linearAccelerationVariance = new DoubleParameter("LinearAccelerationVariance",AtlasHeadPoseEstimator, 10.0,0.0,1.0);
      magneticFieldVariance = new DoubleParameter("MagneticFieldVariance", AtlasHeadPoseEstimator, 0.001,0.0,1.0);
      positionVariance = new DoubleParameter("PositionVariance", AtlasHeadPoseEstimator, 1.0E-8,0.0,1.0);
      headAngularAccelerationVariance = new DoubleParameter("HeadAngularAccelerationVariance", AtlasHeadPoseEstimator, 10.0,0.0,1.0);
      headLinearAccelerationVariance = new DoubleParameter("HeadLinearAccelerationVariance", AtlasHeadPoseEstimator, 10.0,0.0,1.0);
      linearAccelerationBiasVariance = new DoubleParameter("LinearAccelerationBiasVariance", AtlasHeadPoseEstimator, 10.0,0.0,1.0);      
   }
}

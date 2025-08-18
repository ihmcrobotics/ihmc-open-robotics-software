package us.ihmc.alexander.parameters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Initializes YoVariables by BUILDING missing registries/variables and
 * REUSING existing ones when present (no duplicate parameters).
 */
public class RealRobotParameters extends AbstractRobotParameters
{
   // (필요하면 필드 보관; 여기서는 set만 해도 동작)
   public RealRobotParameters(YoRegistry registry)
   {
      super(registry);

      // 컨트롤러 쪽 초기화 (DRCControllerThread 루트를 받았을 때)
      if ("DRCControllerThread".equals(root.getName()))
      {
         YoRegistry drcMomentum = ensureRegistry(root, "DRCMomentumBasedController");
         YoRegistry hhcm = ensureRegistry(drcMomentum, "HumanoidHighLevelControllerManager");

         initTransitions(hhcm);
         initHighLevelHumanoidControllerFactory(hhcm);
         initHighLevelControlManagerFactory(hhcm);
         initWalkingController(hhcm);
      }

      // 에스티메이터 쪽 초기화 (DRCEstimatorThread 루트를 받았을 때)
      if ("DRCEstimatorThread".equals(root.getName()))
      {
         initEstimatorThread(root);
      }
   }

   // ---------------- Controller sections ----------------

   private void initTransitions(YoRegistry hhcm)
   {
      YoRegistry toWalking = ensureRegistry(hhcm, "toWalkingSmoothTransitionControllerState");
      putBoolean(toWalking, "toWalkingEnableTimeBasedTransition", false);

      YoRegistry exitWalking = ensureRegistry(hhcm, "exitWalkingSmoothTransitionControllerState");
      putBoolean(exitWalking, "exitWalkingEnableTimeBasedTransition", true);
   }

   private void initHighLevelHumanoidControllerFactory(YoRegistry hhcm)
   {
      YoRegistry factory = ensureRegistry(hhcm, "HighLevelHumanoidControllerFactory");

      // JointspaceGains
      YoRegistry jointspace = ensureRegistry(factory, "JointspaceGains");
      putDouble(jointspace, "zetaSpineJoints", 0.3);
      putDouble(jointspace, "maximumFeedbackSpineJoints", 10.0);
      putDouble(jointspace, "maximumFeedbackRateSpineJoints", 100.0);
      putDouble(jointspace, "zetaNeckJoints", 0.4);
      putDouble(jointspace, "maximumFeedbackNeckJoints", 6.0);
      putDouble(jointspace, "maximumFeedbackRateNeckJoints", 60.0);
      putDouble(jointspace, "kpArmJoints", 100.0);
      putDouble(jointspace, "zetaArmJoints", 0.1);
      putDouble(jointspace, "maximumFeedbackArmJoints", 20.0);
      putDouble(jointspace, "maximumFeedbackRateArmJoints", 700.0);

      // RigidBodyGains
      YoRegistry rigid = ensureRegistry(factory, "RigidBodyGains");
      putDouble(rigid, "zetaXYChestOrientation", 0.5);
      putDouble(rigid, "zetaZChestOrientation", 0.22);
      putDouble(rigid, "maximumFeedbackChestOrientation", 6.0);
      putDouble(rigid, "maximumFeedbackRateChestOrientation", 60.0);
      putDouble(rigid, "zetaXYZHeadOrientation", 0.4);
      putDouble(rigid, "maximumFeedbackHeadOrientation", 6.0);
      putDouble(rigid, "maximumFeedbackRateHeadOrientation", 60.0);
      putDouble(rigid, "kpXYZHandOrientation", 40.0);
      putDouble(rigid, "zetaXYZHandOrientation", 0.0);
      putDouble(rigid, "maximumFeedbackHandOrientation", 10.0);
      putDouble(rigid, "maximumFeedbackRateHandOrientation", 100.0);
      putDouble(rigid, "zetaXYPelvisOrientation", 0.2);
      putDouble(rigid, "zetaZPelvisOrientation", 0.5);
      putDouble(rigid, "maximumFeedbackPelvisOrientation", 12.0);
      putDouble(rigid, "maximumFeedbackRatePelvisOrientation", 180.0);
      putDouble(rigid, "kpXYZHandPosition", 40.0);
      putDouble(rigid, "zetaXYZHandPosition", 0.0);
      putDouble(rigid, "maximumFeedbackHandPosition", 10.0);
      putDouble(rigid, "maximumFeedbackRateHandPosition", 100.0);

      // FootGains
      YoRegistry foot = ensureRegistry(factory, "FootGains");
      putDouble(foot, "maximumFeedbackSwingFootPosition", 20.0);
      putDouble(foot, "maximumFeedbackRateSwingFootPosition", 300.0);
      putDouble(foot, "maximumFeedbackSwingFootOrientation", 100.0);
      putDouble(foot, "maximumFeedbackRateSwingFootOrientation", 1500.0);
      putDouble(foot, "zetaXYHoldFootPosition", 0.2);
      putDouble(foot, "maximumFeedbackHoldFootPosition", 6.0);
      putDouble(foot, "maximumFeedbackRateHoldFootPosition", 150.0);
      putDouble(foot, "kpXYHoldFootOrientation", 100.0);
      putDouble(foot, "kpZHoldFootOrientation", 100.0);
      putDouble(foot, "zetaXYHoldFootOrientation", 0.2);
      putDouble(foot, "zetaZHoldFootOrientation", 0.2);
      putDouble(foot, "maximumFeedbackHoldFootOrientation", 100.0);
      putDouble(foot, "maximumFeedbackRateHoldFootOrientation", 1500.0);
      putDouble(foot, "maximumFeedbackToeOffFootPosition", 6.0);
      putDouble(foot, "maximumFeedbackRateToeOffFootPosition", 150.0);
      putDouble(foot, "maximumFeedbackToeOffFootOrientation", 100.0);
      putDouble(foot, "maximumFeedbackRateToeOffFootOrientation", 1500.0);

      // MomentumOptimizationSettings
      YoRegistry momentum = ensureRegistry(factory, "MomentumOptimizationSettings");
      putDouble(momentum, "ChestAngularWeightX", 30.0);
      putDouble(momentum, "ChestAngularWeightY", 30.0);
   }

   private void initHighLevelControlManagerFactory(YoRegistry hhcm)
   {
      YoRegistry hlcmf = ensureRegistry(hhcm, "HighLevelControlManagerFactory");

      // CenterOfMassHeightManager / PelvisHeightControlState
      YoRegistry comHeightMgr = ensureRegistry(hlcmf, "CenterOfMassHeightManager");
      YoRegistry pelvis = ensureRegistry(comHeightMgr, "PelvisHeightControlState");
      // XML에선 alias가 있었지만, 여기선 최종 이름으로 생성(이미 있으면 set만)
      putDouble(pelvis, "defaultHeight", 0.8);
      putDouble(pelvis, "offsetTrajectoryTime", 0.5);
      putDouble(pelvis, "maxDistanceAnklePelvis", 0.9);

      // CenterOfMassHeightControlState -> {LookAhead, BetterLookAhead}
      YoRegistry comState = ensureRegistry(comHeightMgr, "CenterOfMassHeightControlState");
      YoRegistry lookAhead = ensureRegistry(comState, "LookAheadCoMHeightTrajectoryGenerator");
      putBoolean(lookAhead, "processGoHome", true);
      YoRegistry betterLookAhead = ensureRegistry(comState, "BetterLookAheadCoMHeightTrajectoryGenerator");
      putBoolean(betterLookAhead, "processGoHome", true);

      // FeetManager
      YoRegistry feet = ensureRegistry(hlcmf, "FeetManager");

      // GeometricToeOffManager / LegJointLimitsInspector
      YoRegistry toeOff = ensureRegistry(feet, "GeometricToeOffManager");
      YoRegistry legLimits = ensureRegistry(toeOff, "LegJointLimitsInspector");
      putDouble(legLimits, "ankleLowerLimitToTriggerToeOff", -0.94);

      // FootRotationDetectorParameters
      YoRegistry rot = ensureRegistry(feet, "FootRotationDetectorParameters");
      putDouble(rot, "omegaThresholdForEstimation", 2.0);
      putDouble(rot, "rotationThreshold", 0.05);

      // SupportStateParameters
      YoRegistry support = ensureRegistry(feet, "SupportStateParameters");
      putBoolean(support, "dampFootRotations", true);
      putDouble(support, "footDamping", 20.0);

      // FeetManager (direct parameters)
      putDouble(feet, "FootSwingTouchdownVelocityZ", -0.25);
      putDouble(feet, "FootSwingTouchdownAccelerationZ", -2.0);
      putDouble(feet, "Verification_PerpendicularCoPErrorThreshold", 0.005);
      putDouble(feet, "Verification_DistanceFromLineToComputeDesiredCoPOccupancy", 0.005);
      putDouble(feet, "Velocity_omegaMagnitudeThreshold", 0.5);
      putDouble(feet, "Velocity_rotationAngleDecayBreakFrequency", 5.0);
      putDouble(feet, "Velocity_rotationAngleThreshold", 0.15);
      putInteger(feet, "Cropping_ThresholdForCoPRegionOccupancy", 4);
      putDouble(feet, "Cropping_DistanceFromLineOfRotationToComputeCoPOccupancy", 0.005);
      putDouble(feet, "Verification_InlineCoPHistoryStdDev", 0.002);
      putDouble(feet, "Verification_TransverseCoPHistoryStdDev", 5.0E-4);
      putDouble(feet, "Cropping_FootDropThresholdForCrop", 0.02);

      // BalanceManager
      YoRegistry balance = ensureRegistry(hlcmf, "BalanceManager");
      YoRegistry walkingCoP = ensureRegistry(balance, "WalkingCoPTrajectoryGenerator");
      putBoolean(walkingCoP, "computeSplitFractionsFromPositions", true);
      // Infinity는 범위 의미가 없으니 그냥 값 세팅
      putDouble(balance, "ellipticICPErrorForMomentumRecovery", Double.POSITIVE_INFINITY);
   }

   private void initWalkingController(YoRegistry hhcm)
   {
      YoRegistry walking = ensureRegistry(hhcm, "WalkingControllerState");
      YoRegistry lmrcm = ensureRegistry(walking, "LinearMomentumRateControlModule");

      // ICPOptimizationController
      YoRegistry icpOpt = ensureRegistry(lmrcm, "ICPOptimizationController");
      putDouble(icpOpt, "controllerThresholdForStuck", 0.12);

      // ICPController
      YoRegistry icp = ensureRegistry(lmrcm, "ICPController");
      putDouble(icp, "captureKi", 1.5);
      putDouble(icp, "captureIntegralLeakRatio", 0.98);
      putDouble(icp, "controllerThresholdForStuck", 0.135);

      // Linear momentum weights
      putDouble(lmrcm, "LinearMomentumRateWeightX", 0.1);
      putDouble(lmrcm, "LinearMomentumRateWeightY", 0.1);
      putDouble(lmrcm, "RecoveryLinearMomentumRateWeightX", 0.3);
      putDouble(lmrcm, "RecoveryLinearMomentumRateWeightY", 0.3);
   }

   // ---------------- Estimator section ----------------

   private void initEstimatorThread(YoRegistry base)
   {
      // base == DRCEstimatorThread
      YoRegistry atlasHeadPoseEstimator = ensureRegistry(base, "AtlasHeadPoseEstimator");

      // (옵션) 레거시 경로
      YoRegistry estimatorController = ensureRegistry(base, "EstimatorController");
      YoRegistry drcKinematics = ensureRegistry(estimatorController, "DRCKinematicsBasedStateEstimator");
      YoRegistry IMUBasedPelvisRotationalStateUpdater = ensureRegistry(drcKinematics, "IMUBasedPelvisRotationalStateUpdater");

      // 값 세팅 (이미 동일 변수 있으면 set만)
      putDouble(atlasHeadPoseEstimator, "AngularVelocityVariance", 3.0E-6);
      putDouble(atlasHeadPoseEstimator, "LinearAccelerationVariance", 10.0);
      putDouble(atlasHeadPoseEstimator, "MagneticFieldVariance", 0.001);
      putDouble(atlasHeadPoseEstimator, "PositionVariance", 1.0E-8);
      putDouble(atlasHeadPoseEstimator, "HeadAngularAccelerationVariance", 10.0);
      putDouble(atlasHeadPoseEstimator, "HeadLinearAccelerationVariance", 10.0);
      putDouble(atlasHeadPoseEstimator, "LinearAccelerationBiasVariance", 10.0);
      
      putBoolean(IMUBasedPelvisRotationalStateUpdater, "useIMUBasedPelvisRotationalStateUpdater", true);
   }

   // ---------------- Helpers ----------------

   /** Ensure a child registry exists; create & attach if missing. */
   private YoRegistry ensureRegistry(YoRegistry parent, String name)
   {
      YoRegistry r = parent.findRegistry(name);
      if (r == null)
      {
         r = new YoRegistry(name);
         parent.addChild(r);
      }
      return r;
   }

   private YoDouble putDouble(YoRegistry reg, String name, double value)
   {
      var existing = reg.findVariable(name);
      if (existing instanceof YoDouble d)
      {
         d.set(value);
         return d;
      }
      // 동일 이름이 다른 타입으로 이미 있다면 충돌 → 명확히 터뜨려서 빨리 찾게
      if (existing != null)
         throw new IllegalStateException("Variable '" + name + "' already exists in " + reg.getName()
                                         + " as " + existing.getClass().getSimpleName());
      YoDouble v = new YoDouble(name, reg);
      v.set(value);
      return v;
   }

   private YoBoolean putBoolean(YoRegistry reg, String name, boolean value)
   {
      var existing = reg.findVariable(name);
      if (existing instanceof YoBoolean b)
      {
         b.set(value);
         return b;
      }
      if (existing != null)
         throw new IllegalStateException("Variable '" + name + "' already exists in " + reg.getName()
                                         + " as " + existing.getClass().getSimpleName());
      YoBoolean v = new YoBoolean(name, reg);
      v.set(value);
      return v;
   }

   private YoInteger putInteger(YoRegistry reg, String name, int value)
   {
      var existing = reg.findVariable(name);
      if (existing instanceof YoInteger i)
      {
         i.set(value);
         return i;
      }
      if (existing != null)
         throw new IllegalStateException("Variable '" + name + "' already exists in " + reg.getName()
                                         + " as " + existing.getClass().getSimpleName());
      YoInteger v = new YoInteger(name, reg);
      v.set(value);
      return v;
   }
}

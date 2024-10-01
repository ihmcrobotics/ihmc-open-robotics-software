package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import gnu.trove.map.hash.TObjectIntHashMap;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OneDoFJointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.CenterOfMassStabilityMarginRegionCalculator;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.SensitivityBasedCoMMarginCalculator;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.WholeBodyContactState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.math.filters.RateLimitedYoFrameOrientation;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class KinematicsToolboxMultiContactManager
{
   private static final boolean MODIFY_POSTURE_WITH_PRIVILEGED_CONFIGURATION = false;
   private static final double JOINT_LIMIT_REDUCTION_PERCENTAGE = 0.05;
   private static final double JOINTSPACE_KP = 50.0;
   private static final double PELVIS_POSTURE_KP = 1200.0;
   private static final double MAX_PELVIS_HEIGHT_ERROR = 0.05;
   private static final double MAX_PELVIS_ORIENTATION_ERROR = Math.toRadians(25.0);
   private static final int SPATIAL_DIMENSIONS = 6;

   // nominal is around 0.72
   private static final double DEFAULT_MIN_PELVIS_HEIGHT = 0.62;
   private static final double DEFAULT_MAX_PELVIS_HEIGHT = 0.78;

   private final FramePoint3D zeroPoint = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();
   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FramePose3D tempPose = new FramePose3D();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final ExecutionTimer postureOptimizationTimer = new ExecutionTimer("postureOptimizationTimer", registry);
   private final double updateDT;

   /* Region managers and sensitivity calculator */
   private final TObjectIntHashMap<OneDoFJointBasics> jointIndexMap = new TObjectIntHashMap<>();
   private final WholeBodyContactState wholeBodyContactState;
   private final CenterOfMassStabilityMarginRegionCalculator multiContactRegionCalculator;
   private final SensitivityBasedCoMMarginCalculator postureOptimizer;
   private final FullHumanoidRobotModel fullRobotModel;
   private final PDGains jointspaceGains = new PDGains();

   private final RateLimitedYoVariable optimizedPelvisHeight;
   private final YoDouble minOptimizedPelvisHeight = new YoDouble("minOptimizedPelvisHeight", registry);
   private final YoDouble maxOptimizedPelvisHeight = new YoDouble("maxOptimizedPelvisHeight", registry);

   private final FrameQuaternion integratedPelvisOrientation = new FrameQuaternion();
   private final YoFrameYawPitchRoll optimizedPelvisOrientation = new YoFrameYawPitchRoll("optimizedPelvisOrientation", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll initialPelvisOrientation = new YoFrameYawPitchRoll("initialPelvisOrientation", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll actualPelvisOrientation = new YoFrameYawPitchRoll("actualPelvisOrientation", ReferenceFrame.getWorldFrame(), registry);
   private final RateLimitedYoFrameOrientation optimizedPelvisOrientationRateLimited;
   private final Vector3D pelvisRotationVectorAdjustment = new Vector3D();
   private final Quaternion pelvisRotationQuaternionAdjustment = new Quaternion();

   private final YoEnum<PostureOptimizerState> mode = new YoEnum<>("postureOptimizerMode", registry, PostureOptimizerState.class, false);

   /* Posture optimization state */
   private double postureSensitivityHysteresisEpsilon = 0.01;
   private final YoDouble postureSensitivityThreshold = new YoDouble("postureSensitivityThreshold", registry);
   private final GlitchFilteredYoBoolean isPostureSensitivityHigh = new GlitchFilteredYoBoolean("isPostureSensitivityHigh", registry, 12);

   /* Stability margin state */
   private final YoDouble stabilityMarginThresholdLow = new YoDouble("stabilityMarginThresholdLow", registry);
   private final YoDouble stabilityMarginThresholdHigh = new YoDouble("stabilityMarginThresholdHigh", registry);
   private final YoEnum<StabilityMarginLevel> stabilityMarginLevel = new YoEnum<>("stabilityMarginLevel", registry, StabilityMarginLevel.class, false);

   private final YoDouble qdMultiplier = new YoDouble("qdMultiplier", registry);
   private final YoDouble postureOptimizationWeight = new YoDouble("postureOptimizationWeight", registry);

   /* The nominal privileged configuration */
   private final YoDouble[] qPrivNominal;
   /* The optimized privileged configuration. In the NOMINAL state, this is (modulo rate-limiting) the same as qPrivNominal. */
   private final YoDouble[] qPrivOptimized;
   /* The optimized privileged configuration, rate-limited */
   private final RateLimitedYoVariable[] qPrivOptimizedRateLimited;

   private final YoDouble pelvisPostureWeight = new YoDouble("pelvisPostureWeight", registry);
   private final SelectionMatrix3D pelvisHeightSelection = new SelectionMatrix3D();
   private final DefaultPID3DGains pelvisHeightGains = new DefaultPID3DGains();
   private final DefaultPID3DGains pelvisOrientationGains = new DefaultPID3DGains();

   private final YoDouble maxPostureAdjustmentRate = new YoDouble("maxPostureAdjustmentRate", registry);
   private final RateLimitedYoVariable activationAlpha;

   public enum PostureOptimizerState
   {
      /* Low margin, high sensitivity */
      OPTIMIZER,
      /* Medium margin or low margin + low sensitivity */
      FREEZE,
      /* High margin */
      NOMINAL;

      private static final PostureOptimizerState[] values = values();

      public byte toByte()
      {
         return (byte) ordinal();
      }

      public static PostureOptimizerState fromByte(byte enumAsByte)
      {
         if (enumAsByte == -1)
            return null;
         return values[enumAsByte];
      }
   }

   private enum StabilityMarginLevel
   {
      /* Low margin, run the optimizer if possible */
      LOW,
      /* Medium margin, freeze posture adjustment */
      MEDIUM,
      /* High margin, return to nominal configuration */
      HIGH,
   }

   // q_priv = q_priv_baseline + q_priv_offset
   //
   // Desired behavior:
   //    By default:
   //       q_priv_baseline = nominal mapped value
   //       q_priv_offset = 0.0
   //    When activated:
   //       q_priv_baseline = the posture of the robot when this activated
   //       q_priv_offset = calculated by the posture adjustment calculator

   public KinematicsToolboxMultiContactManager(WholeBodyContactState wholeBodyContactState,
                                               CenterOfMassStabilityMarginRegionCalculator multiContactRegionCalculator,
                                               FullHumanoidRobotModel fullRobotModel,
                                               ReferenceFrame centerOfMassFrame,
                                               double updateDT,
                                               YoRegistry parentRegistry)
   {
      this.wholeBodyContactState = wholeBodyContactState;
      this.multiContactRegionCalculator = multiContactRegionCalculator;
      this.updateDT = updateDT;
      this.fullRobotModel = fullRobotModel;
      this.postureOptimizer = new SensitivityBasedCoMMarginCalculator(centerOfMassFrame,
                                                                      fullRobotModel,
                                                                      wholeBodyContactState,
                                                                      multiContactRegionCalculator,
                                                                      registry);

      int numberOfJoints = wholeBodyContactState.getNumberOfJoints();
      OneDoFJointBasics[] oneDoFJoints = wholeBodyContactState.getOneDoFJoints();

      qPrivNominal = new YoDouble[numberOfJoints];
      qPrivOptimized = new YoDouble[numberOfJoints];
      qPrivOptimizedRateLimited = new RateLimitedYoVariable[numberOfJoints];

      jointspaceGains.setKp(JOINTSPACE_KP);

      pelvisHeightGains.setProportionalGains(PELVIS_POSTURE_KP);
      pelvisHeightGains.setMaxProportionalError(MAX_PELVIS_HEIGHT_ERROR);
      pelvisOrientationGains.setProportionalGains(PELVIS_POSTURE_KP);
      pelvisOrientationGains.setMaxProportionalError(MAX_PELVIS_ORIENTATION_ERROR);

      pelvisHeightSelection.setAxisSelection(false, false, true);
      pelvisPostureWeight.set(3.0);
      optimizedPelvisHeight = new RateLimitedYoVariable("optimizedPelvisHeight", registry, maxPostureAdjustmentRate, updateDT);
      optimizedPelvisOrientationRateLimited = new RateLimitedYoFrameOrientation("optimizedPelvisOrientationRL", "", registry, maxPostureAdjustmentRate, updateDT, optimizedPelvisOrientation);

      for (int i = 0; i < numberOfJoints; i++)
      {
         OneDoFJointBasics joint = oneDoFJoints[i];
         jointIndexMap.put(joint, i);

         qPrivNominal[i] = new YoDouble("q_priv_nom_" + joint.getName(), registry);
         qPrivOptimized[i] = new YoDouble("q_priv_opt_" + joint.getName(), registry);
         qPrivOptimizedRateLimited[i] = new RateLimitedYoVariable("q_priv_opt_rl_" + joint.getName(), registry, maxPostureAdjustmentRate, updateDT);
      }

      activationAlpha = new RateLimitedYoVariable("activationAlpha", registry, 0.4, updateDT);

      double defaultPostureSensitivityThreshold = 0.065;
      double defaultStabilityMarginThresholdLow = 0.12; // should be higher than 5cm, which is the IK solver's threshold to keep the CoM safe
      double defaultStabilityMarginThresholdHigh = 0.16; // 0.12;

      postureSensitivityThreshold.set(defaultPostureSensitivityThreshold);
      stabilityMarginThresholdLow.set(defaultStabilityMarginThresholdLow);
      stabilityMarginThresholdHigh.set(defaultStabilityMarginThresholdHigh);

      maxPostureAdjustmentRate.set(0.15);
      qdMultiplier.set(0.3);
      postureOptimizationWeight.set(0.25);
      mode.set(PostureOptimizerState.NOMINAL);

      parentRegistry.addChild(registry);
   }

   public void initialize(PrivilegedConfigurationCommand privilegedConfigurationCommand)
   {
      mode.set(PostureOptimizerState.NOMINAL);
      isPostureSensitivityHigh.set(false);
      activationAlpha.set(0.0);

      // Set baseline to match default privileged configuration
      for (int i = 0; i < privilegedConfigurationCommand.getNumberOfJoints(); i++)
      {
         OneDoFJointBasics joint = privilegedConfigurationCommand.getJoint(i);
         OneDoFJointPrivilegedConfigurationParameters jointSpecificParameters = privilegedConfigurationCommand.getJointSpecificParameters(i);

         if (jointSpecificParameters.hasPrivilegedConfiguration())
         {
            int jointIndex = jointIndexMap.get(joint);
            double qPrivilegedConfiguration = jointSpecificParameters.getPrivilegedConfiguration();
            qPrivNominal[jointIndex].set(qPrivilegedConfiguration);
            qPrivOptimized[jointIndex].set(qPrivilegedConfiguration);
         }
         else
         {
            throw new RuntimeException("Expecting full joint-specific params");
         }
      }
   }

   public void update()
   {
      // These conditions must be true in order to activate the posture optimizer
      boolean hasPostureSensitivity = false;

      tempPose.setToZero(fullRobotModel.getPelvis().getBodyFixedFrame());
      tempPose.changeFrame(ReferenceFrame.getWorldFrame());
      actualPelvisOrientation.set(tempPose.getOrientation());

      if (multiContactRegionCalculator.hasSolvedWholeRegion())
      {
         postureOptimizationTimer.startMeasurement();
         hasPostureSensitivity = postureOptimizer.updateAll();
         postureOptimizationTimer.stopMeasurement();
      }
      else
      {
         postureOptimizer.initialize();
      }

      if (hasPostureSensitivity)
      {
         double postureSensitivityThreshold = this.postureSensitivityThreshold.getValue() + postureSensitivityHysteresisEpsilon * (mode.getValue() == PostureOptimizerState.OPTIMIZER ? -1.0 : 1.0);
         isPostureSensitivityHigh.update(postureOptimizer.getPostureSensitivity() > postureSensitivityThreshold);

         if (multiContactRegionCalculator.getCenterOfMassStabilityMargin() < stabilityMarginThresholdLow.getValue())
         {
            stabilityMarginLevel.set(StabilityMarginLevel.LOW);
         }
         else if (multiContactRegionCalculator.getCenterOfMassStabilityMargin() < stabilityMarginThresholdHigh.getValue())
         {
            stabilityMarginLevel.set(StabilityMarginLevel.MEDIUM);
         }
         else
         {
            stabilityMarginLevel.set(StabilityMarginLevel.HIGH);
         }
      }
      else
      {
         isPostureSensitivityHigh.set(false);
         stabilityMarginLevel.set(StabilityMarginLevel.HIGH);
      }

      // Update activation status
      PostureOptimizerState currentMode = mode.getValue();
      PostureOptimizerState newMode;

      if (stabilityMarginLevel.getValue() == StabilityMarginLevel.HIGH)
      {
         newMode = PostureOptimizerState.NOMINAL;
      }
      else if (stabilityMarginLevel.getValue() == StabilityMarginLevel.LOW && isPostureSensitivityHigh.getValue())
      {
         newMode = PostureOptimizerState.OPTIMIZER;
      }
      else
      {
         newMode = PostureOptimizerState.FREEZE;
      }

      mode.set(newMode);

      if (newMode == PostureOptimizerState.OPTIMIZER && currentMode != PostureOptimizerState.OPTIMIZER)
      {
         onOptimizerEnabled();
      }

      if (mode.getValue() == PostureOptimizerState.OPTIMIZER)
      {
         // do optimizer update
         updateTowardsOptimizedPosture();
      }
      else if (mode.getValue() == PostureOptimizerState.NOMINAL)
      {
         // do nominal update
         updateTowardsNominalPosture();
      }
      else
      {
         // freeze, do nothing
      }

      if (mode.getValue() == PostureOptimizerState.NOMINAL)
      {
         activationAlpha.update(0.0);
      }
      else if (mode.getValue() == PostureOptimizerState.OPTIMIZER)
      {
         activationAlpha.update(1.0);
      }
   }

   private void updateTowardsOptimizedPosture()
   {
      // Update privileged configuration
      for (int i = 0; i < qPrivOptimized.length; i++)
      {
         double qdOffsetSetpoint = qdMultiplier.getValue() * postureOptimizer.getOptimizedWholeBodyVelocity().get(SPATIAL_DIMENSIONS + i);
         qdOffsetSetpoint = EuclidCoreTools.clamp(qdOffsetSetpoint, maxPostureAdjustmentRate.getValue());
         double qOffsetSetpoint = qPrivOptimized[i].getValue() + updateDT * qdOffsetSetpoint;
         qPrivOptimized[i].set(qOffsetSetpoint);

         double qPrivilieged = qPrivOptimized[i].getValue();
         OneDoFJointBasics joint = wholeBodyContactState.getOneDoFJoints()[i];
         double jointRangeOfMotion = joint.getJointLimitUpper() - joint.getJointLimitLower();
         double jointLimitReduction = jointRangeOfMotion * JOINT_LIMIT_REDUCTION_PERCENTAGE;
         qPrivilieged = EuclidCoreTools.clamp(qPrivilieged, joint.getJointLimitLower() + jointLimitReduction, joint.getJointLimitUpper() - jointLimitReduction);
         qPrivOptimizedRateLimited[i].update(qPrivilieged);
      }

      int linearZIndex = 5;
      double qdPelvisHeight = qdMultiplier.getValue() * postureOptimizer.getOptimizedWholeBodyVelocity().get(linearZIndex);
      double qPelvisHeight = optimizedPelvisHeight.getValue() + updateDT * qdPelvisHeight;
      qPelvisHeight = EuclidCoreTools.clamp(qPelvisHeight, minOptimizedPelvisHeight.getValue(), maxOptimizedPelvisHeight.getValue());
      optimizedPelvisHeight.update(qPelvisHeight);

      tempVector.set(0, postureOptimizer.getOptimizedWholeBodyVelocity());
      pelvisRotationVectorAdjustment.setAndScale(qdMultiplier.getValue() * updateDT, tempVector);
      pelvisRotationQuaternionAdjustment.setRotationVector(pelvisRotationVectorAdjustment);
      integratedPelvisOrientation.append(pelvisRotationQuaternionAdjustment);

      // TODO clamp to max yaw/pitch/roll

      optimizedPelvisOrientation.set(integratedPelvisOrientation);
      optimizedPelvisOrientationRateLimited.update();
   }

   private void updateTowardsNominalPosture()
   {
      // Update privileged configuration
      for (int i = 0; i < qPrivOptimized.length; i++)
      {
         qPrivOptimized[i].set(qPrivNominal[i].getValue());
         double qPrivileged = qPrivOptimized[i].getValue();
         qPrivOptimizedRateLimited[i].update(qPrivileged);
      }

      // TODO towards nominal pelvis height/orientation
   }

   private void onOptimizerEnabled()
   {
      tempPose.setToZero(fullRobotModel.getPelvis().getBodyFixedFrame());
      tempPose.changeFrame(ReferenceFrame.getWorldFrame());
      double initialPelvisHeight = tempPose.getZ();

      minOptimizedPelvisHeight.set(Math.min(DEFAULT_MIN_PELVIS_HEIGHT, initialPelvisHeight));
      maxOptimizedPelvisHeight.set(Math.max(DEFAULT_MAX_PELVIS_HEIGHT, initialPelvisHeight));
      optimizedPelvisHeight.set(initialPelvisHeight);

      integratedPelvisOrientation.set(tempPose.getOrientation());
      initialPelvisOrientation.set(tempPose.getOrientation());

      for (int i = 0; i < qPrivNominal.length; i++)
      { // In case it's partially decayed and switches back to OPTIMIZER, go ahead and reinitialize here
         qPrivOptimized[i].set(qPrivOptimizedRateLimited[i].getValue());
      }
   }

   public boolean isActivated()
   {
      return mode.getValue() != PostureOptimizerState.NOMINAL;
   }

   public double getActivationAlpha()
   {
      return activationAlpha.getValue();
   }

   public PostureOptimizerState getCurrentState()
   {
      return mode.getValue();
   }

   public void addPostureFeedbackCommands(FeedbackControlCommandBuffer bufferToPack)
   {
      boolean isActivated = activationAlpha.getValue() > 1.0e-5;

      if (!MODIFY_POSTURE_WITH_PRIVILEGED_CONFIGURATION && isActivated)
      {
         // Set optimized posture
         OneDoFJointBasics[] oneDoFJoints = wholeBodyContactState.getOneDoFJoints();

         for (int i = 0; i < wholeBodyContactState.getNumberOfJoints(); i++)
         {
            OneDoFJointBasics joint = oneDoFJoints[i];
            OneDoFJointFeedbackControlCommand jointFeedbackCommand = bufferToPack.addOneDoFJointFeedbackControlCommand();
            jointFeedbackCommand.setJoint(joint);
            jointFeedbackCommand.setWeightForSolver(activationAlpha.getValue() * postureOptimizationWeight.getValue());
            jointFeedbackCommand.setInverseKinematics(qPrivOptimizedRateLimited[i].getValue(), 0.0);
            jointFeedbackCommand.setGains(jointspaceGains);
         }
      }

      if (isActivated)
      {
         zeroPoint.setToZero(fullRobotModel.getPelvis().getBodyFixedFrame());
         tempPoint.setZ(optimizedPelvisHeight.getValue());

         PointFeedbackControlCommand pelvisHeightCommand = bufferToPack.addPointFeedbackControlCommand();
         pelvisHeightCommand.set(fullRobotModel.getRootBody(), fullRobotModel.getPelvis());
         pelvisHeightCommand.setBodyFixedPointToControl(zeroPoint);
         pelvisHeightCommand.setInverseKinematics(tempPoint, null);
         pelvisHeightCommand.setWeightForSolver(pelvisPostureWeight.getValue() * activationAlpha.getValue());
         pelvisHeightCommand.setSelectionMatrix(pelvisHeightSelection);
         pelvisHeightCommand.setGains(pelvisHeightGains);

         OrientationFeedbackControlCommand pelvisOrientationCommand = bufferToPack.addOrientationFeedbackControlCommand();
         pelvisOrientationCommand.set(fullRobotModel.getRootBody(), fullRobotModel.getPelvis());
         pelvisOrientationCommand.setInverseKinematics(optimizedPelvisOrientationRateLimited, null);
         pelvisOrientationCommand.setWeightForSolver(pelvisPostureWeight.getValue() * activationAlpha.getValue());
         pelvisOrientationCommand.setGains(pelvisOrientationGains);
         pelvisOrientationCommand.setSelectionMatrixToIdentity();
         pelvisOrientationCommand.getControlFrameOrientation().setToZero();
      }
   }

   public void addPostureInverseKinematicsCommands(InverseKinematicsCommandBuffer bufferToPack)
   {
      if (MODIFY_POSTURE_WITH_PRIVILEGED_CONFIGURATION)
      {
         PrivilegedConfigurationCommand privilegedConfigurationCommand = bufferToPack.addPrivilegedConfigurationCommand();
         privilegedConfigurationCommand.clear();

         privilegedConfigurationCommand.setDefaultWeight(2.0);
         privilegedConfigurationCommand.setDefaultConfigurationGain(50.0);

         // For some reason the IK controller requires having each joint populated
         OneDoFJointBasics[] oneDoFJoints = wholeBodyContactState.getOneDoFJoints();
         for (int i = 0; i < oneDoFJoints.length; i++)
         {
            privilegedConfigurationCommand.addJoint(oneDoFJoints[i], qPrivOptimizedRateLimited[i].getValue());
            //         privilegedConfigurationCommand.addJoint(oneDoFJoints[i], qPrivNominal[i].getValue());
         }
      }
   }
}

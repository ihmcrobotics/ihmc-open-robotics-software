package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import gnu.trove.map.hash.TObjectIntHashMap;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OneDoFJointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.StabilityMarginRegionCalculator;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.SensitivityBasedStabilityGradientCalculator;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.WholeBodyContactState;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.PostureOptimizerState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.FrameYawPitchRoll;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class KinematicsToolboxMultiContactManager
{
   public static final boolean DO_SMOOTH_STATE_TRANSITION = true;

   private static final double JOINT_LIMIT_REDUCTION_PERCENTAGE = 0.05;
   private static final double JOINTSPACE_KP = 50.0;
   private static final double PELVIS_POSTURE_KP = 1200.0;
   private static final double MAX_PELVIS_HEIGHT_ERROR = 0.05;
   private static final double MAX_PELVIS_ORIENTATION_ERROR = Math.toRadians(25.0);
   private static final int SPATIAL_DIMENSIONS = 6;

   // Filter params
   private static final double WHOLE_BODY_POSTURE_ADJUSTMENT_MAGNITUDE = 1.0 / 3.0;
   private static final double ALPHA_ENABLED_TIME_CONSTANT = 3.0;

   // nominal is around 1.05
   private static final double DEFAULT_MIN_PELVIS_HEIGHT = 0.85;
   private static final double DEFAULT_MAX_PELVIS_HEIGHT = 1.15;

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
   private final StabilityMarginRegionCalculator multiContactRegionCalculator;
   private final SensitivityBasedStabilityGradientCalculator postureOptimizer;
   private final FullHumanoidRobotModel fullRobotModel;
   private final PDGains jointspaceGains = new PDGains();

   private final ReferenceFrame midFeetZUpFrame;
   private final YoDouble optimizedPelvisHeight = new YoDouble("optimizedPelvisHeight", registry);
   private final YoDouble minOptimizedPelvisHeight = new YoDouble("minOptimizedPelvisHeight", registry);
   private final YoDouble maxOptimizedPelvisHeight = new YoDouble("maxOptimizedPelvisHeight", registry);

   private final FrameYawPitchRoll integratedPelvisOrientation = new FrameYawPitchRoll();
   private final YoFrameYawPitchRoll optimizedPelvisOrientation = new YoFrameYawPitchRoll("optimizedPelvisOrientation", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll initialPelvisOrientation = new YoFrameYawPitchRoll("initialPelvisOrientation", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll actualPelvisOrientation = new YoFrameYawPitchRoll("actualPelvisOrientation", ReferenceFrame.getWorldFrame(), registry);
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

   private final YoDouble postureOptimizationWeight = new YoDouble("postureOptimizationWeight", registry);

   /* The nominal privileged configuration */
   private final YoDouble[] qPrivNominal;
   /* The optimized privileged configuration. In the NOMINAL state, this is (modulo rate-limiting) the same as qPrivNominal. */
   private final YoDouble[] qPrivOptimized;
   private final DMatrixRMaj qdNominal = new DMatrixRMaj(0);

   private final SelectionMatrix3D pelvisHeightSelection = new SelectionMatrix3D();
   private final DefaultPID3DGains pelvisHeightGains = new DefaultPID3DGains();
   private final DefaultPID3DGains pelvisOrientationGains = new DefaultPID3DGains();

   private final RateLimitedYoVariable activationAlpha;

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
                                               StabilityMarginRegionCalculator multiContactRegionCalculator,
                                               FullHumanoidRobotModel fullRobotModel,
                                               ReferenceFrame centerOfMassFrame,
                                               ReferenceFrame midFeetZUpFrame,
                                               double updateDT,
                                               YoRegistry parentRegistry)
   {
      this.wholeBodyContactState = wholeBodyContactState;
      this.multiContactRegionCalculator = multiContactRegionCalculator;
      this.updateDT = updateDT;
      this.fullRobotModel = fullRobotModel;
      this.postureOptimizer = new SensitivityBasedStabilityGradientCalculator(centerOfMassFrame,
                                                                              fullRobotModel,
                                                                              wholeBodyContactState,
                                                                              multiContactRegionCalculator,
                                                                              registry);
      this.midFeetZUpFrame = midFeetZUpFrame;

      int numberOfJoints = wholeBodyContactState.getNumberOfJoints();
      OneDoFJointBasics[] oneDoFJoints = wholeBodyContactState.getOneDoFJoints();

      qPrivNominal = new YoDouble[numberOfJoints];
      qPrivOptimized = new YoDouble[numberOfJoints];
      qdNominal.reshape(numberOfJoints, 1);

      jointspaceGains.setKp(JOINTSPACE_KP);

      pelvisHeightGains.setProportionalGains(PELVIS_POSTURE_KP);
      pelvisHeightGains.setMaxProportionalError(MAX_PELVIS_HEIGHT_ERROR);
      pelvisOrientationGains.setProportionalGains(PELVIS_POSTURE_KP);
      pelvisOrientationGains.setMaxProportionalError(MAX_PELVIS_ORIENTATION_ERROR);

      pelvisHeightSelection.setAxisSelection(false, false, true);

      for (int i = 0; i < numberOfJoints; i++)
      {
         OneDoFJointBasics joint = oneDoFJoints[i];
         jointIndexMap.put(joint, i);

         qPrivNominal[i] = new YoDouble("q_priv_nom_" + joint.getName(), registry);
         qPrivOptimized[i] = new YoDouble("q_priv_opt_" + joint.getName(), registry);
      }

      activationAlpha = new RateLimitedYoVariable("activationAlpha", registry, 1.0 / ALPHA_ENABLED_TIME_CONSTANT, updateDT);

      double defaultPostureSensitivityThreshold = 0.04;
      double defaultStabilityMarginThresholdLow = 0.12; // should be higher than 5cm, which is the IK solver's threshold to keep the CoM safe
      double defaultStabilityMarginThresholdHigh = 0.18; // 0.12;

      postureSensitivityThreshold.set(defaultPostureSensitivityThreshold);
      stabilityMarginThresholdLow.set(defaultStabilityMarginThresholdLow);
      stabilityMarginThresholdHigh.set(defaultStabilityMarginThresholdHigh);

      postureOptimizationWeight.set(0.3);
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

      if (!hasPostureSensitivity)
      {
         isPostureSensitivityHigh.set(false);
         stabilityMarginLevel.set(StabilityMarginLevel.HIGH);
         return;
      }

      // TODO add if/else for abrupt transition
      if (DO_SMOOTH_STATE_TRANSITION)
      {
         doSmoothStateTransition();
      }
      else
      {
         doDiscreteStateTransition();
      }
   }

   private void doSmoothStateTransition()
   {
      double postureSensitivityThreshold = this.postureSensitivityThreshold.getValue() + postureSensitivityHysteresisEpsilon * (mode.getValue() == PostureOptimizerState.OPTIMIZER ? -1.0 : 1.0);
      boolean isPostureSensitivityHigh = postureOptimizer.getPostureSensitivity() > postureSensitivityThreshold;
      this.isPostureSensitivityHigh.update(isPostureSensitivityHigh);

      if (multiContactRegionCalculator.getStabilityMargin() < stabilityMarginThresholdLow.getValue())
      {
         stabilityMarginLevel.set(StabilityMarginLevel.LOW);
      }
      else if (multiContactRegionCalculator.getStabilityMargin() < stabilityMarginThresholdHigh.getValue())
      {
         stabilityMarginLevel.set(StabilityMarginLevel.MEDIUM);
      }
      else
      {
         stabilityMarginLevel.set(StabilityMarginLevel.HIGH);
      }

      // Update activation status
      PostureOptimizerState currentMode = mode.getValue();
      PostureOptimizerState newMode;

      if (stabilityMarginLevel.getValue() == StabilityMarginLevel.HIGH)
      {
         newMode = PostureOptimizerState.NOMINAL;
      }
      else if (stabilityMarginLevel.getValue() == StabilityMarginLevel.LOW && this.isPostureSensitivityHigh.getValue())
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

   private void doDiscreteStateTransition()
   {
      if (stabilityMarginLevel.getValue() == StabilityMarginLevel.LOW)
      {
         // update towards optimized posture
         activationAlpha.set(1.0);
      }
      else
      {
         activationAlpha.set(0.0);

         for (int i = 0; i < qPrivOptimized.length; i++)
         {
            qPrivOptimized[i].set(qPrivNominal[i].getValue());
         }
      }
   }

   private void updateTowardsOptimizedPosture()
   {
      // Update privileged configuration
      for (int i = 0; i < qPrivOptimized.length; i++)
      {
         double qdOffsetSetpoint = WHOLE_BODY_POSTURE_ADJUSTMENT_MAGNITUDE * postureOptimizer.getNomalizedStabilityGradient().get(SPATIAL_DIMENSIONS + i);
         double qOffsetSetpoint = qPrivOptimized[i].getValue() + updateDT * qdOffsetSetpoint;
         OneDoFJointBasics joint = wholeBodyContactState.getOneDoFJoints()[i];
         double jointRangeOfMotion = joint.getJointLimitUpper() - joint.getJointLimitLower();
         double jointLimitReduction = jointRangeOfMotion * JOINT_LIMIT_REDUCTION_PERCENTAGE;
         qOffsetSetpoint = EuclidCoreTools.clamp(qOffsetSetpoint, joint.getJointLimitLower() + jointLimitReduction, joint.getJointLimitUpper() - jointLimitReduction);
         qPrivOptimized[i].set(qOffsetSetpoint);
      }

      int linearZIndex = 5;
      double qdPelvisHeight = WHOLE_BODY_POSTURE_ADJUSTMENT_MAGNITUDE * postureOptimizer.getNomalizedStabilityGradient().get(linearZIndex);
      double qPelvisHeight = optimizedPelvisHeight.getValue() + updateDT * qdPelvisHeight;
      qPelvisHeight = EuclidCoreTools.clamp(qPelvisHeight, minOptimizedPelvisHeight.getValue(), maxOptimizedPelvisHeight.getValue());
      optimizedPelvisHeight.set(qPelvisHeight);

      tempVector.set(0, postureOptimizer.getNomalizedStabilityGradient());
      pelvisRotationVectorAdjustment.setAndScale(WHOLE_BODY_POSTURE_ADJUSTMENT_MAGNITUDE * updateDT, tempVector);
      pelvisRotationQuaternionAdjustment.setRotationVector(pelvisRotationVectorAdjustment);
      integratedPelvisOrientation.append(pelvisRotationQuaternionAdjustment);

      // clamp the yaw/pitch/roll of the pelvis orientation setpoint
      integratedPelvisOrientation.changeFrame(midFeetZUpFrame);
      double clampedYaw = EuclidCoreTools.clamp(integratedPelvisOrientation.getYaw(), Math.toRadians(40.0));
      double clampedPitch = EuclidCoreTools.clamp(integratedPelvisOrientation.getPitch(), Math.toRadians(45.0));
      double clampedRoll = EuclidCoreTools.clamp(integratedPelvisOrientation.getYaw(), Math.toRadians(35.0));
      integratedPelvisOrientation.setYawPitchRoll(clampedYaw, clampedPitch, clampedRoll);
      integratedPelvisOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      optimizedPelvisOrientation.set(integratedPelvisOrientation);
   }

   private void updateTowardsNominalPosture()
   {
      double deltaQNominalMagnitude = 0.0;
      for (int i = 0; i < qPrivOptimized.length; i++)
      {
         deltaQNominalMagnitude += MathTools.square(qPrivNominal[i].getValue() - qPrivOptimized[i].getValue());
      }
      deltaQNominalMagnitude = Math.sqrt(deltaQNominalMagnitude);

      boolean isEpsilonFromNominal = deltaQNominalMagnitude < WHOLE_BODY_POSTURE_ADJUSTMENT_MAGNITUDE * updateDT;
      for (int i = 0; i < qPrivOptimized.length; i++)
      {
         if (isEpsilonFromNominal)
         {
            qPrivOptimized[i].set(qPrivNominal[i].getValue());
         }
         else
         {
            double q0 = qPrivOptimized[i].getValue();
            double qd = (qPrivNominal[i].getValue() - qPrivOptimized[i].getValue()) * WHOLE_BODY_POSTURE_ADJUSTMENT_MAGNITUDE / deltaQNominalMagnitude;
            qPrivOptimized[i].set(q0 + qd * updateDT);
         }
      }
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

      if (isActivated)
      {
         // Set optimized posture
         OneDoFJointBasics[] oneDoFJoints = wholeBodyContactState.getOneDoFJoints();

         for (int i = 0; i < wholeBodyContactState.getNumberOfJoints(); i++)
         {
            OneDoFJointBasics joint = oneDoFJoints[i];
            OneDoFJointFeedbackControlCommand jointFeedbackCommand = bufferToPack.addOneDoFJointFeedbackControlCommand();
            jointFeedbackCommand.setJoint(joint);
            jointFeedbackCommand.setWeightForSolver(activationAlpha.getValue() * postureOptimizationWeight.getValue());
            jointFeedbackCommand.setInverseKinematics(qPrivOptimized[i].getValue(), 0.0);
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
         pelvisHeightCommand.setWeightForSolver(postureOptimizationWeight.getValue() * activationAlpha.getValue());
         pelvisHeightCommand.setSelectionMatrix(pelvisHeightSelection);
         pelvisHeightCommand.setGains(pelvisHeightGains);

         OrientationFeedbackControlCommand pelvisOrientationCommand = bufferToPack.addOrientationFeedbackControlCommand();
         pelvisOrientationCommand.set(fullRobotModel.getRootBody(), fullRobotModel.getPelvis());
         pelvisOrientationCommand.setInverseKinematics(optimizedPelvisOrientation, null);
         pelvisOrientationCommand.setWeightForSolver(postureOptimizationWeight.getValue() * activationAlpha.getValue());
         pelvisOrientationCommand.setGains(pelvisOrientationGains);
         pelvisOrientationCommand.setSelectionMatrixToIdentity();
         pelvisOrientationCommand.getControlFrameOrientation().setToZero();
      }
   }

   public void addPostureInverseKinematicsCommands(InverseKinematicsCommandBuffer bufferToPack)
   {

   }

   public PostureOptimizerState getMode()
   {
      return mode.getValue();
   }

   public double getPostureSensitivity()
   {
      return postureOptimizer.getPostureSensitivity();
   }

   public double getAlphaEnabled()
   {
      return activationAlpha.getValue();
   }
}

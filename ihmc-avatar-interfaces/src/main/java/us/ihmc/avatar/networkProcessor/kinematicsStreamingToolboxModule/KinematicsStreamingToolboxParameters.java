package us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule;

import toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInitialConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.output.KSTFBOutputProcessor;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import java.util.HashMap;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class KinematicsStreamingToolboxParameters
{
   public enum InputStateEstimatorType
   {
      /**
       * Simple first-order estimation of the input velocities to then perform an extrapolation in the future using again first-order integration.
       */
      FIRST_ORDER_LPF,
      /**
       * Slightly more complex estimation of the input velocities based on first order finite difference plus a correction based on the error in pose. This
       * provides a continuous estimation of the input velocities.
       */
      FBC_STYLE
   }

   public enum ClockType
   {
      /**
       * Compute the time based on the system clock, i.e. {@code System.nanoTime()}.
       * Helpful when the toolbox is running in a non-real-time environment.
       */
      CPU_CLOCK,
      /**
       * Compute the time based on the toolbox internal clock.
       * Helpful when the toolbox is running in a real-time environment or for test purposes.
       */
      FIXED_DT;
   }

   protected ClockType clockType;
   /**
    * Period at which the toolbox will update its internal state.
    * It's best to shoot for a multiple of the controller update period.
    */
   protected double toolboxUpdatePeriod;
   /**
    * Duration after which the controller will go to sleep if no input is received.
    */
   protected double timeThresholdForSleeping;

   /**
    * Upon reception of a new streaming message from the IK, the controller will extrapolate the solution to the future to avoid discontinuities.
    */
   protected double streamIntegrationDuration;
   /**
    * Safety margin to keep the center of mass within the support polygon.
    */
   protected double centerOfMassSafeMargin;
   /**
    * Center of mass offset with respect to center of support region
    */
   protected final Vector2D centerOfMassOffset = new Vector2D();
   /**
    * Weight used to hold the center of mass in place.
    */
   protected double centerOfMassHoldWeight;
   protected double centerOfMassTrackingWeight;
   /**
    * Number of toolbox update ticks between two published solutions.
    * For instance, 10 means the solution is published once every 10 toolbox updates.
    */
   protected int publishSolutionEveryNTicks;

   /**
    * Default weight for holding the arms at the robot initial configuration when no arm message is received.
    */
   protected double holdArmWeight;
   /**
    * Default weight for holding the neck at the robot initial configuration when no neck message is received.
    */
   protected double holdNeckWeight;
   /**
    * Default weight for holding the pelvis at the robot initial configuration when no pelvis message is received.
    */
   protected final Vector3D holdPelvisLinearWeight = new Vector3D();
   /**
    * Default weight for holding the pelvis at the robot initial configuration when no pelvis message is received.
    */
   protected final Vector3D holdPelvisAngularWeight = new Vector3D();
   /**
    * Default weight for holding the chest at the robot initial configuration when no chest message is received.
    */
   protected final Vector3D holdChestAngularWeight = new Vector3D();

   /**
    * Default weight for locking the pelvis at the robot initial configuration when no pelvis message is received.
    */
   protected double lockPelvisWeight;
   /**
    * Default weight for locking the chest at the robot initial configuration when no chest message is received.
    */
   protected double lockChestWeight;

   /**
    * Default weight for input messages for which no weight is provided.
    */
   protected Vector3D defaultLinearWeight = new Vector3D();
   /**
    * Default weight for input messages for which no weight is provided.
    */
   protected Vector3D defaultAngularWeight = new Vector3D();
   /**
    * Default weight for pelvis input messages for which no weight is provided.
    */
   protected Vector3D defaultPelvisLinearWeight = new Vector3D();
   /**
    * Default weight for pelvis input messages for which no weight is provided.
    */
   protected Vector3D defaultPelvisAngularWeight = new Vector3D();
   /**
    * Default weight for chest input messages for which no weight is provided.
    */
   protected Vector3D defaultChestLinearWeight = new Vector3D();
   /**
    * Default weight for chest input messages for which no weight is provided.
    */
   protected Vector3D defaultChestAngularWeight = new Vector3D();
   /**
    * Default weight for hand input messages for which no weight is provided.
    */
   protected Vector3D defaultHandLinearWeight = new Vector3D();
   /**
    * Default weight for hand input messages for which no weight is provided.
    */
   protected Vector3D defaultHandAngularWeight = new Vector3D();
   /**
    * Default gain for the linear part of a taskspace objective.
    */
   protected double defaultLinearGain;
   /**
    * Default gain for the angular part of a taskspace objective.
    */
   protected double defaultAngularGain;
   /**
    * Default gain for a jointspace objective.
    */
   protected double defaultSingleJointGain;

   /**
    * Default rate limit for input messages for which no rate limit is provided.
    */
   protected double defaultLinearRateLimit;
   /**
    * Default rate limit for input messages for which no rate limit is provided.
    */
   protected double defaultAngularRateLimit;
   /**
    * Scale factor used to downscale the joint velocity solution before sending it to the controller.
    */
   protected double outputJointVelocityScale;
   /**
    * When using {@link KSTFBOutputProcessor}, this is the proportional gain used in the processor.
    */
   protected double outputFeedbackGain;
   /**
    * When using {@link KSTFBOutputProcessor}, this is the damping ratio used in the processor.
    */
   protected double outputFeedbackDampingRatio;
   /**
    * Break frequency used for the low-pass filter used to filter the output of the IK solver.
    */
   protected double outputLPFBreakFrequency;

   /**
    * Whether to minimize the angular momentum in the kinematics solution.
    * Useful to reduce fast motions with the parts of the robot and instead privilege the motion of the extremities.
    */
   protected boolean minimizeAngularMomentum;
   /**
    * Whether to minimize the linear momentum in the kinematics solution.
    * Useful to reduce fast motions with the parts of the robot and instead privilege the motion of the extremities.
    */
   protected boolean minimizeLinearMomentum;
   /**
    * Weight used to minimize the angular momentum in the kinematics solution.
    */
   protected Vector3D angularMomentumWeight;
   /**
    * Weight used to minimize the linear momentum in the kinematics solution.
    */
   protected Vector3D linearMomentumWeight;
   /**
    * Whether to minimize the rate of change of the angular momentum in the kinematics solution.
    */
   protected boolean minimizeAngularMomentumRate;
   /**
    * Whether to minimize the rate of change of the linear momentum in the kinematics solution.
    */
   protected boolean minimizeLinearMomentumRate;
   /**
    * Weight used to minimize the rate of change of the angular momentum in the kinematics solution.
    */
   protected Vector3D angularMomentumRateWeight;
   /**
    * Weight used to minimize the rate of change of the linear momentum in the kinematics solution.
    */
   protected Vector3D linearMomentumRateWeight;
   /**
    * Duration used to smoothly initiate the streaming to the controller.
    */
   protected double defaultStreamingBlendingDuration;

   /**
    * Break frequency used for the low-pass filter used to estimate the input pose.
    * Only used when {@link #inputStateEstimatorType} is set to {@link InputStateEstimatorType#FIRST_ORDER_LPF}.
    */
   protected double inputPoseLPFBreakFrequency;
   /**
    * Duration use to decay the estimated input weight.
    * When no input is received, the weight is decayed to zero, then the control for that end-effector is disabled.
    */
   protected double inputWeightDecayDuration;
   /**
    * Duration use to decay the estimated input velocity.
    * Safety used to limit the extrapolation in the future when inputs are not being received.
    */
   protected double inputVelocityDecayDuration;
   /**
    * Duration used to correct the input pose in the state estimator.
    * Should be greater than the period at which inputs are received.
    * Only used when {@link #inputStateEstimatorType} is set to {@link InputStateEstimatorType#FBC_STYLE}.
    */
   protected double inputVelocityRawAlpha;
   /**
    * Duration used to correct the input pose in the state estimator. Should be greater than the period at which inputs are received. Only used when
    * {@link #inputStateEstimatorType} is set to {@link InputStateEstimatorType#FBC_STYLE}.
    */
   protected double inputPoseCorrectionDuration;
   /**
    * When {@code true}, a bounding box filter is used to reject input poses that are too far from the current pose.
    * The bottom center of the bounding box is set to line up with the robot's mid-foot z-up frame.
    */
   protected boolean useBBXInputFilter;
   /**
    * Size of the bounding box filter used to reject input poses that are too far from the current pose.
    */
   protected Vector3D inputFilterBBXSize;
   /**
    * Center of the bounding box with respect to the robot's mid-foot z-up frame.
    */
   protected Point3D inputFilterBBXCenter;
   protected double inputFilterMaxLinearDelta;
   protected double inputFilterMaxAngularDelta;
   protected double inputFilterMaxLinearVelocity;
   protected double inputFilterMaxAngularVelocity;

   protected boolean useStreamingPublisher;
   protected double publishingPeriod;

   protected InputStateEstimatorType inputStateEstimatorType;

   /**
    * Map from joint name to initial 1-DoF joint position.
    */
   protected Map<String, Double> initialConfigurationMap;
   protected Map<String, Double> jointCustomPositionUpperLimits;
   protected Map<String, Double> jointCustomPositionLowerLimits;
   protected final KinematicsStreamingToolboxConfigurationMessage defaultConfiguration = new KinematicsStreamingToolboxConfigurationMessage();
   protected final KinematicsStreamingToolboxInitialConfigurationMessage defaultInitialConfiguration = new KinematicsStreamingToolboxInitialConfigurationMessage();
   protected final KinematicsToolboxConfigurationMessage defaultSolverConfiguration = new KinematicsToolboxConfigurationMessage();
   protected double solverNullspaceAlpha;
   protected double solverPrivilegedDefaultWeight;
   protected double solverPrivilegedDefaultGain;

   protected final List<String> inactiveJoints = new ArrayList<>();

   public static KinematicsStreamingToolboxParameters defaultParameters()
   {
      KinematicsStreamingToolboxParameters parameters = new KinematicsStreamingToolboxParameters();
      parameters.setDefault();
      return parameters;
   }

   public void setDefault()
   {
      clockType = ClockType.CPU_CLOCK;
      toolboxUpdatePeriod = 0.003;
      timeThresholdForSleeping = 3.0;

      useStreamingPublisher = true;
      publishingPeriod = 0.006;
      streamIntegrationDuration = 2.0 * publishingPeriod;

      holdChestAngularWeight.set(1.0, 1.0, 0.5);
      holdPelvisLinearWeight.set(10.0, 10.0, 20.0);
      holdPelvisAngularWeight.set(1.0, 1.0, 1.0);
      holdArmWeight = 10.0;
      holdNeckWeight = 10.0;

      centerOfMassSafeMargin = 0.01;
      centerOfMassHoldWeight = 0.001;
      centerOfMassTrackingWeight = 0.001;
      publishSolutionEveryNTicks = 6;

      lockPelvisWeight = 1000.0;
      lockChestWeight = 1000.0;

      defaultLinearWeight.set(20.0, 20.0, 20.0);
      defaultAngularWeight.set(2.5, 2.5, 2.5);
      defaultPelvisLinearWeight.set(defaultLinearWeight);
      defaultPelvisAngularWeight.set(defaultAngularWeight);
      defaultChestLinearWeight.set(defaultLinearWeight);
      defaultChestAngularWeight.set(defaultAngularWeight);
      defaultHandLinearWeight.set(defaultLinearWeight);
      defaultHandAngularWeight.set(defaultAngularWeight);

      defaultLinearGain = 50.0;
      defaultAngularGain = 50.0;
      defaultSingleJointGain = 50.0;

      defaultLinearRateLimit = 10.0;
      defaultAngularRateLimit = 100.0;

      minimizeAngularMomentum = true;
      minimizeLinearMomentum = true;
      angularMomentumWeight = new Vector3D(0.2, 0.2, 0.2);
      linearMomentumWeight = new Vector3D(0.01, 0.01, 0.0);

      minimizeAngularMomentumRate = true;
      minimizeLinearMomentumRate = true;
      angularMomentumRateWeight = new Vector3D(1.0, 1.0, 1.0);
      linearMomentumRateWeight = new Vector3D(1.0, 1.0, 1.0);

      defaultStreamingBlendingDuration = 2.0;
      inputPoseLPFBreakFrequency = 15.0;
      inputPoseCorrectionDuration = 0.05; // Need to send inputs at 30Hz.
      inputWeightDecayDuration = 3.0;
      inputVelocityDecayDuration = 0.5;
      inputVelocityRawAlpha = 0.65; // TODO This prob can be 1.0, afraid of overshoots.
      inputStateEstimatorType = InputStateEstimatorType.FBC_STYLE;
      useBBXInputFilter = true;
      inputFilterBBXSize = new Vector3D(2.0, 2.8, 2.6);
      inputFilterBBXCenter = new Point3D(0.4, 0.0, 1.25);

      outputJointVelocityScale = 0.5;
      outputFeedbackGain = 500.0;
      outputFeedbackDampingRatio = 1.0;
      outputLPFBreakFrequency = 10.0;

      defaultConfiguration.setLockPelvis(false);
      defaultConfiguration.setLockChest(false);
      defaultConfiguration.setEnableLeftArmJointspace(true);
      defaultConfiguration.setEnableRightArmJointspace(true);
      defaultConfiguration.setEnableNeckJointspace(true);
      defaultConfiguration.setEnableLeftHandTaskspace(false);
      defaultConfiguration.setEnableRightHandTaskspace(false);
      defaultConfiguration.setEnableChestTaskspace(true);
      defaultConfiguration.setEnablePelvisTaskspace(true);
      defaultConfiguration.setEnableCenterOfMassControl(false);
      defaultConfiguration.setLeftHandTrajectoryFrameId(ReferenceFrame.getWorldFrame().getFrameNameHashCode());
      defaultConfiguration.setRightHandTrajectoryFrameId(ReferenceFrame.getWorldFrame().getFrameNameHashCode());
      defaultConfiguration.setChestTrajectoryFrameId(ReferenceFrame.getWorldFrame().getFrameNameHashCode());
      defaultConfiguration.setPelvisTrajectoryFrameId(ReferenceFrame.getWorldFrame().getFrameNameHashCode());

      defaultSolverConfiguration.setJointVelocityWeight(0.05);
      defaultSolverConfiguration.setJointAccelerationWeight(0.0); // As soon as we increase this guy, we inject springy behavior.
      defaultSolverConfiguration.setEnableJointVelocityLimits(true);

      // Use the solver's default values for the following:
      solverNullspaceAlpha = -1.0;
      solverPrivilegedDefaultWeight = -1.0;
      solverPrivilegedDefaultGain = -1.0;

      inputFilterMaxLinearDelta = 0.5;
      inputFilterMaxAngularDelta = Double.POSITIVE_INFINITY;
      inputFilterMaxLinearVelocity = 6.0;
      inputFilterMaxAngularVelocity = Double.POSITIVE_INFINITY;
   }

   public ClockType getClockType()
   {
      return clockType;
   }

   public double getToolboxUpdatePeriod()
   {
      return toolboxUpdatePeriod;
   }

   public double getTimeThresholdForSleeping()
   {
      return timeThresholdForSleeping;
   }

   public double getStreamIntegrationDuration()
   {
      return streamIntegrationDuration;
   }

   public double getCenterOfMassSafeMargin()
   {
      return centerOfMassSafeMargin;
   }

   public double getCenterOfMassHoldWeight()
   {
      return centerOfMassHoldWeight;
   }

   public double getCenterOfMassTrackingWeight()
   {
      return centerOfMassTrackingWeight;
   }

   public int getPublishSolutionEveryNTicks()
   {
      return publishSolutionEveryNTicks;
   }

   public double getHoldArmWeight()
   {
      return holdArmWeight;
   }

   public double getHoldNeckWeight()
   {
      return holdNeckWeight;
   }

   public Vector3D getHoldPelvisLinearWeight()
   {
      return holdPelvisLinearWeight;
   }

   public Vector3D getHoldPelvisAngularWeight()
   {
      return holdPelvisAngularWeight;
   }

   public Vector3D getHoldChestAngularWeight()
   {
      return holdChestAngularWeight;
   }

   public double getLockPelvisWeight()
   {
      return lockPelvisWeight;
   }

   public double getLockChestWeight()
   {
      return lockChestWeight;
   }

   public Vector3D getDefaultLinearWeight()
   {
      return defaultLinearWeight;
   }

   public Vector3D getDefaultAngularWeight()
   {
      return defaultAngularWeight;
   }

   public Vector3D getDefaultPelvisLinearWeight()
   {
      return defaultPelvisLinearWeight;
   }

   public Vector3D getDefaultPelvisAngularWeight()
   {
      return defaultPelvisAngularWeight;
   }

   public Vector3D getDefaultChestLinearWeight()
   {
      return defaultChestLinearWeight;
   }

   public Vector3D getDefaultChestAngularWeight()
   {
      return defaultChestAngularWeight;
   }

   public Vector3D getDefaultHandLinearWeight()
   {
      return defaultHandLinearWeight;
   }

   public Vector3D getDefaultHandAngularWeight()
   {
      return defaultHandAngularWeight;
   }

   public double getDefaultLinearGain()
   {
      return defaultLinearGain;
   }

   public double getDefaultAngularGain()
   {
      return defaultAngularGain;
   }

   public double getDefaultSingleJointGain()
   {
      return defaultSingleJointGain;
   }

   public double getDefaultLinearRateLimit()
   {
      return defaultLinearRateLimit;
   }

   public double getDefaultAngularRateLimit()
   {
      return defaultAngularRateLimit;
   }

   public double getOutputJointVelocityScale()
   {
      return outputJointVelocityScale;
   }

   public double getOutputFeedbackGain()
   {
      return outputFeedbackGain;
   }

   public double getOutputFeedbackDampingRatio()
   {
      return outputFeedbackDampingRatio;
   }

   public double getOutputLPFBreakFrequency()
   {
      return outputLPFBreakFrequency;
   }

   public boolean isMinimizeAngularMomentum()
   {
      return minimizeAngularMomentum;
   }

   public boolean isMinimizeLinearMomentum()
   {
      return minimizeLinearMomentum;
   }

   public Vector3D getAngularMomentumWeight()
   {
      return angularMomentumWeight;
   }

   public Vector3D getLinearMomentumWeight()
   {
      return linearMomentumWeight;
   }

   public boolean isMinimizeAngularMomentumRate()
   {
      return minimizeAngularMomentumRate;
   }

   public boolean isMinimizeLinearMomentumRate()
   {
      return minimizeLinearMomentumRate;
   }

   public Vector3D getAngularMomentumRateWeight()
   {
      return angularMomentumRateWeight;
   }

   public Vector3D getLinearMomentumRateWeight()
   {
      return linearMomentumRateWeight;
   }

   public double getDefaultStreamingBlendingDuration()
   {
      return defaultStreamingBlendingDuration;
   }

   public double getInputPoseLPFBreakFrequency()
   {
      return inputPoseLPFBreakFrequency;
   }

   public double getInputWeightDecayDuration()
   {
      return inputWeightDecayDuration;
   }

   public double getInputVelocityDecayDuration()
   {
      return inputVelocityDecayDuration;
   }

   public double getInputVelocityRawAlpha()
   {
      return inputVelocityRawAlpha;
   }

   public double getInputPoseCorrectionDuration()
   {
      return inputPoseCorrectionDuration;
   }

   public boolean isUseBBXInputFilter()
   {
      return useBBXInputFilter;
   }

   public Vector3D getInputFilterBBXSize()
   {
      return inputFilterBBXSize;
   }

   public Point3D getInputFilterBBXCenter()
   {
      return inputFilterBBXCenter;
   }

   public double getInputFilterMaxLinearDelta()
   {
      return inputFilterMaxLinearDelta;
   }

   public double getInputFilterMaxAngularDelta()
   {
      return inputFilterMaxAngularDelta;
   }

   public double getInputFilterMaxLinearVelocity()
   {
      return inputFilterMaxLinearVelocity;
   }

   public double getInputFilterMaxAngularVelocity()
   {
      return inputFilterMaxAngularVelocity;
   }

   public boolean getUseStreamingPublisher()
   {
      return useStreamingPublisher;
   }

   public double getPublishingPeriod()
   {
      return publishingPeriod;
   }

   public InputStateEstimatorType getInputStateEstimatorType()
   {
      return inputStateEstimatorType;
   }

   public Map<String, Double> getJointCustomPositionUpperLimits()
   {
      return jointCustomPositionUpperLimits;
   }

   public Map<String, Double> getJointCustomPositionLowerLimits()
   {
      return jointCustomPositionLowerLimits;
   }

   public Map<String, Double> getInitialConfigurationMap()
   {
      return initialConfigurationMap;
   }

   public KinematicsStreamingToolboxConfigurationMessage getDefaultConfiguration()
   {
      return defaultConfiguration;
   }

   public KinematicsToolboxConfigurationMessage getDefaultSolverConfiguration()
   {
      return defaultSolverConfiguration;
   }

   public double getSolverNullspaceAlpha()
   {
      return solverNullspaceAlpha;
   }

   public double getSolverPrivilegedDefaultWeight()
   {
      return solverPrivilegedDefaultWeight;
   }

   public double getSolverPrivilegedDefaultGain()
   {
      return solverPrivilegedDefaultGain;
   }

   public void setClockType(ClockType clockType)
   {
      this.clockType = clockType;
   }

   public void setToolboxUpdatePeriod(double toolboxUpdatePeriod)
   {
      this.toolboxUpdatePeriod = toolboxUpdatePeriod;
   }

   public void setTimeThresholdForSleeping(double timeThresholdForSleeping)
   {
      this.timeThresholdForSleeping = timeThresholdForSleeping;
   }

   public void setStreamIntegrationDuration(double streamIntegrationDuration)
   {
      this.streamIntegrationDuration = streamIntegrationDuration;
   }

   public void setCenterOfMassSafeMargin(double centerOfMassSafeMargin)
   {
      this.centerOfMassSafeMargin = centerOfMassSafeMargin;
   }

   public void setCenterOfMassOffset(Vector2D offset)
   {
      this.centerOfMassOffset.set(offset);
   }

   public Vector2D getCenterOfMassOffset()
   {
      return centerOfMassOffset;
   }

   public void setCenterOfMassHoldWeight(double centerOfMassHoldWeight)
   {
      this.centerOfMassHoldWeight = centerOfMassHoldWeight;
   }

   public void setPublishSolutionEveryNTicks(int publishSolutionEveryNTicks)
   {
      this.publishSolutionEveryNTicks = Math.max(1, publishSolutionEveryNTicks);
   }

   public void setHoldArmWeight(double holdArmWeight)
   {
      this.holdArmWeight = holdArmWeight;
   }

   public void setHoldNeckWeight(double holdNeckWeight)
   {
      this.holdNeckWeight = holdNeckWeight;
   }

   public void setHoldPelvisLinearWeight(double xWeight, double yWeight, double zWeight)
   {
      this.holdPelvisLinearWeight.set(xWeight, yWeight, zWeight);
   }

   public void setHoldPelvisAngularWeight(double xWeight, double yWeight, double zWeight)
   {
      this.holdPelvisAngularWeight.set(xWeight, yWeight, zWeight);
   }

   public void setHoldChestAngularWeight(double xWeight, double yWeight, double zWeight)
   {
      this.holdChestAngularWeight.set(xWeight, yWeight, zWeight);
   }

   public void setLockPelvisWeight(double lockPelvisWeight)
   {
      this.lockPelvisWeight = lockPelvisWeight;
   }

   public void setLockChestWeight(double lockChestWeight)
   {
      this.lockChestWeight = lockChestWeight;
   }

   public void setDefaultLinearWeight(double defaultLinearWeight)
   {
      this.defaultLinearWeight = new Vector3D(defaultLinearWeight, defaultLinearWeight, defaultLinearWeight);
   }

   public void setDefaultLinearWeight(Tuple3DReadOnly defaultLinearWeight)
   {
      this.defaultLinearWeight = new Vector3D(defaultLinearWeight);
   }

   public void setDefaultAngularWeight(double defaultAngularWeight)
   {
      this.defaultAngularWeight = new Vector3D(defaultAngularWeight, defaultAngularWeight, defaultAngularWeight);
   }

   public void setDefaultAngularWeight(Tuple3DReadOnly defaultAngularWeight)
   {
      this.defaultAngularWeight = new Vector3D(defaultAngularWeight);
   }

   public void setDefaultPelvisLinearWeight(double defaultPelvisLinearWeight)
   {
      this.defaultPelvisLinearWeight = new Vector3D(defaultPelvisLinearWeight, defaultPelvisLinearWeight, defaultPelvisLinearWeight);
   }

   public void setDefaultPelvisLinearWeight(Tuple3DReadOnly defaultPelvisLinearWeight)
   {
      this.defaultPelvisLinearWeight = new Vector3D(defaultPelvisLinearWeight);
   }

   public void setDefaultPelvisAngularWeight(double defaultPelvisAngularWeight)
   {
      this.defaultPelvisAngularWeight = new Vector3D(defaultPelvisAngularWeight, defaultPelvisAngularWeight, defaultPelvisAngularWeight);
   }

   public void setDefaultPelvisAngularWeight(Tuple3DReadOnly defaultPelvisAngularWeight)
   {
      this.defaultPelvisAngularWeight = new Vector3D(defaultPelvisAngularWeight);
   }

   public void setDefaultChestLinearWeight(double defaultChestLinearWeight)
   {
      this.defaultChestLinearWeight = new Vector3D(defaultChestLinearWeight, defaultChestLinearWeight, defaultChestLinearWeight);
   }

   public void setDefaultChestLinearWeight(Tuple3DReadOnly defaultChestLinearWeight)
   {
      this.defaultChestLinearWeight = new Vector3D(defaultChestLinearWeight);
   }

   public void setDefaultChestAngularWeight(double defaultChestAngularWeight)
   {
      this.defaultChestAngularWeight = new Vector3D(defaultChestAngularWeight, defaultChestAngularWeight, defaultChestAngularWeight);
   }

   public void setDefaultChestAngularWeight(Tuple3DReadOnly defaultChestAngularWeight)
   {
      this.defaultChestAngularWeight = new Vector3D(defaultChestAngularWeight);
   }

   public void setDefaultHandLinearWeight(double defaultHandLinearWeight)
   {
      this.defaultHandLinearWeight = new Vector3D(defaultHandLinearWeight, defaultHandLinearWeight, defaultHandLinearWeight);
   }

   public void setDefaultHandLinearWeight(Tuple3DReadOnly defaultHandLinearWeight)
   {
      this.defaultHandLinearWeight = new Vector3D(defaultHandLinearWeight);
   }

   public void setDefaultHandAngularWeight(double defaultHandAngularWeight)
   {
      this.defaultHandAngularWeight = new Vector3D(defaultHandAngularWeight, defaultHandAngularWeight, defaultHandAngularWeight);
   }

   public void setDefaultHandAngularWeight(Tuple3DReadOnly defaultHandAngularWeight)
   {
      this.defaultHandAngularWeight = new Vector3D(defaultHandAngularWeight);
   }

   public void setDefaultLinearGain(double defaultLinearGain)
   {
      this.defaultLinearGain = defaultLinearGain;
   }

   public void setDefaultAngularGain(double defaultAngularGain)
   {
      this.defaultAngularGain = defaultAngularGain;
   }

   public void setDefaultSingleJointGain(double defaultSingleJointGain)
   {
      this.defaultSingleJointGain = defaultSingleJointGain;
   }

   public void setDefaultLinearRateLimit(double defaultLinearRateLimit)
   {
      this.defaultLinearRateLimit = defaultLinearRateLimit;
   }

   public void setDefaultAngularRateLimit(double defaultAngularRateLimit)
   {
      this.defaultAngularRateLimit = defaultAngularRateLimit;
   }

   public void setOutputJointVelocityScale(double outputJointVelocityScale)
   {
      this.outputJointVelocityScale = outputJointVelocityScale;
   }

   public void setOutputFeedbackGain(double outputFeedbackGain)
   {
      this.outputFeedbackGain = outputFeedbackGain;
   }

   public void setOutputFeedbackDampingRatio(double outputFeedbackDampingRatio)
   {
      this.outputFeedbackDampingRatio = outputFeedbackDampingRatio;
   }

   public void setOutputLPFBreakFrequency(double outputLPFBreakFrequency)
   {
      this.outputLPFBreakFrequency = outputLPFBreakFrequency;
   }

   public void setMinimizeAngularMomentum(boolean minimizeAngularMomentum)
   {
      this.minimizeAngularMomentum = minimizeAngularMomentum;
   }

   public void setMinimizeLinearMomentum(boolean minimizeLinearMomentum)
   {
      this.minimizeLinearMomentum = minimizeLinearMomentum;
   }

   public void setAngularMomentumWeight(double angularMomentumWeight)
   {
      this.angularMomentumWeight = new Vector3D(angularMomentumWeight, angularMomentumWeight, angularMomentumWeight);
   }

   public void setLinearMomentumWeight(double linearMomentumWeight)
   {
      this.linearMomentumWeight = new Vector3D(linearMomentumWeight, linearMomentumWeight, linearMomentumWeight);
   }

   public void setAngularMomentumWeight(Vector3D angularMomentumWeight)
   {
      this.angularMomentumWeight = angularMomentumWeight;
   }

   public void setLinearMomentumWeight(Vector3D linearMomentumWeight)
   {
      this.linearMomentumWeight = linearMomentumWeight;
   }

   public void setMinimizeAngularMomentumRate(boolean minimizeAngularMomentumRate)
   {
      this.minimizeAngularMomentumRate = minimizeAngularMomentumRate;
   }

   public void setMinimizeLinearMomentumRate(boolean minimizeLinearMomentumRate)
   {
      this.minimizeLinearMomentumRate = minimizeLinearMomentumRate;
   }

   public void setAngularMomentumRateWeight(double angularMomentumRateWeight)
   {
      this.angularMomentumRateWeight = new Vector3D(angularMomentumRateWeight, angularMomentumRateWeight, angularMomentumRateWeight);
   }

   public void setAngularMomentumRateWeight(Tuple3DReadOnly angularMomentumRateWeight)
   {
      this.angularMomentumRateWeight = new Vector3D(angularMomentumRateWeight);
   }

   public void setLinearMomentumRateWeight(double linearMomentumRateWeight)
   {
      this.linearMomentumRateWeight = new Vector3D(linearMomentumRateWeight, linearMomentumRateWeight, linearMomentumRateWeight);
   }

   public void setLinearMomentumRateWeight(Tuple3DReadOnly linearMomentumRateWeight)
   {
      this.linearMomentumRateWeight = new Vector3D(linearMomentumRateWeight);
   }

   public void setDefaultStreamingBlendingDuration(double defaultStreamingBlendingDuration)
   {
      this.defaultStreamingBlendingDuration = defaultStreamingBlendingDuration;
   }

   public void setInputPoseLPFBreakFrequency(double inputPoseLPFBreakFrequency)
   {
      this.inputPoseLPFBreakFrequency = inputPoseLPFBreakFrequency;
   }

   public void setInputWeightDecayDuration(double inputWeightDecayDuration)
   {
      this.inputWeightDecayDuration = inputWeightDecayDuration;
   }

   public void setInputVelocityDecayDuration(double inputVelocityDecayDuration)
   {
      this.inputVelocityDecayDuration = inputVelocityDecayDuration;
   }

   public void setInputVelocityRawAlpha(double inputVelocityRawAlpha)
   {
      this.inputVelocityRawAlpha = inputVelocityRawAlpha;
   }

   public void setInputPoseCorrectionDuration(double inputPoseCorrectionDuration)
   {
      this.inputPoseCorrectionDuration = inputPoseCorrectionDuration;
   }

   public void setUseBBXInputFilter(boolean useBBXInputFilter)
   {
      this.useBBXInputFilter = useBBXInputFilter;
   }

   public void setInputBBXFilterCenter(double x, double y, double z)
   {
      if (inputFilterBBXCenter == null)
         inputFilterBBXCenter = new Point3D();
      inputFilterBBXCenter.set(x, y, z);
   }

   public void setInputFilterBBXCenter(Point3D inputFilterBBXCenter)
   {
      this.inputFilterBBXCenter = inputFilterBBXCenter;
   }

   public void setInputBBXFilterSize(double x, double y, double z)
   {
      if (inputFilterBBXSize == null)
         inputFilterBBXSize = new Vector3D();
      inputFilterBBXSize.set(x, y, z);
   }

   public void setInputFilterBBXSize(Vector3D inputFilterBBXSize)
   {
      this.inputFilterBBXSize = inputFilterBBXSize;
   }

   public void setInputFilterMaxLinearDelta(double inputFilterMaxLinearDelta)
   {
      this.inputFilterMaxLinearDelta = inputFilterMaxLinearDelta;
   }

   public void setInputFilterMaxAngularDelta(double inputFilterMaxAngularDelta)
   {
      this.inputFilterMaxAngularDelta = inputFilterMaxAngularDelta;
   }

   public void setInputFilterMaxLinearVelocity(double inputFilterMaxLinearVelocity)
   {
      this.inputFilterMaxLinearVelocity = inputFilterMaxLinearVelocity;
   }

   public void setInputFilterMaxAngularVelocity(double inputFilterMaxAngularVelocity)
   {
      this.inputFilterMaxAngularVelocity = inputFilterMaxAngularVelocity;
   }

   public void setUseStreamingPublisher(boolean useStreamingPublisher)
   {
      this.useStreamingPublisher = useStreamingPublisher;
   }

   public void setPublishingPeriod(double publishingPeriod)
   {
      this.publishingPeriod = publishingPeriod;
   }

   public void setInputStateEstimatorType(InputStateEstimatorType inputStateEstimatorType)
   {
      this.inputStateEstimatorType = inputStateEstimatorType;
   }

   public void setJointCustomPositionUpperLimits(Map<String, Double> jointCustomPositionUpperLimits)
   {
      this.jointCustomPositionUpperLimits = jointCustomPositionUpperLimits;
   }

   public void setJointCustomPositionLowerLimits(Map<String, Double> jointCustomPositionLowerLimits)
   {
      this.jointCustomPositionLowerLimits = jointCustomPositionLowerLimits;
   }

   public void setInitialConfigurationMap(Map<String, Double> initialConfigurationMap)
   {
      this.initialConfigurationMap = initialConfigurationMap;
   }

   public void setSolverNullspaceAlpha(double solverNullspaceAlpha)
   {
      this.solverNullspaceAlpha = solverNullspaceAlpha;
   }

   public void setSolverPrivilegedDefaultWeight(double solverPrivilegedDefaultWeight)
   {
      this.solverPrivilegedDefaultWeight = solverPrivilegedDefaultWeight;
   }

   public void setSolverPrivilegedDefaultGain(double solverPrivilegedDefaultGain)
   {
      this.solverPrivilegedDefaultGain = solverPrivilegedDefaultGain;
   }

   public Map<String, Double> createInitialConfiguration(DRCRobotModel robotModel)
   {
      Map<String, Double> initialConfiguration = new HashMap<>();
      RobotInitialSetup<HumanoidFloatingRootJointRobot> defaultRobotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);
      FullHumanoidRobotModel robot = robotModel.createFullRobotModel();
      HumanoidJointNameMap jointMap = robotModel.getJointMap();
      defaultRobotInitialSetup.initializeFullRobotModel(robot);

      for (OneDoFJointBasics joint : robot.getOneDoFJoints())
      {
         initialConfiguration.put(joint.getName(), joint.getQ());
      }

      return initialConfigurationMap;
   }

   public void restrictZeroCrossover(DRCRobotModel robotModel, String jointName)
   {
      OneDoFJointDefinition joint = robotModel.getRobotDefinition().getOneDoFJointDefinition(jointName);
      if (joint == null)
         return;

      double positionLowerLimit = joint.getPositionLowerLimit();
      double positionUpperLimit = joint.getPositionUpperLimit();

      /* No zero cross-over */
      if (positionLowerLimit * positionUpperLimit > 0.0)
         return;

      double tolerance = 0.1;
      boolean adjustLowerLimit = Math.abs(positionLowerLimit) < Math.abs(positionUpperLimit);
      if (adjustLowerLimit)
      {
         if (jointCustomPositionLowerLimits == null)
            jointCustomPositionLowerLimits = new HashMap<>();
         jointCustomPositionLowerLimits.put(jointName, tolerance);
      }
      else
      {
         if (jointCustomPositionUpperLimits == null)
            jointCustomPositionUpperLimits = new HashMap<>();
         jointCustomPositionUpperLimits.put(jointName, -tolerance);
      }
   }

   public void setJointLimit(String jointName, boolean isUpperLimit, double value)
   {
      if (isUpperLimit)
      {
         if (jointCustomPositionUpperLimits == null)
            jointCustomPositionUpperLimits = new HashMap<>();
         jointCustomPositionUpperLimits.put(jointName, value);
      }
      else
      {
         if (jointCustomPositionLowerLimits == null)
            jointCustomPositionLowerLimits = new HashMap<>();
         jointCustomPositionLowerLimits.put(jointName, value);
      }
   }

   public List<String> getInactiveJoints()
   {
      return inactiveJoints;
   }
}

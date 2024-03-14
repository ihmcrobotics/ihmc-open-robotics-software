package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.tools.UnitConversions;

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

   private ClockType clockType;
   /**
    * Period at which the toolbox will update its internal state.
    * It's best to shoot for a multiple of the controller update period.
    */
   private double toolboxUpdatePeriod;
   /**
    * Duration after which the controller will go to sleep if no input is received.
    */
   private double timeThresholdForSleeping;

   /**
    * Upon reception of a new streaming message from the IK, the controller will extrapolate the solution to the future to avoid discontinuities.
    */
   private double streamIntegrationDuration;
   /**
    * Safety margin to keep the center of mass within the support polygon.
    */
   private double centerOfMassSafeMargin;
   /**
    * Weight used to hold the center of mass in place.
    */
   private double centerOfMassHoldWeight;
   /**
    * Period at which the kinematics solution is published to the controller.
    * The faster, the better, but it also increases the communication load.
    */
   private double publishingSolutionPeriod;

   /**
    * Default weight for holding the arms at the robot initial configuration when no arm message is received.
    */
   private double defaultArmMessageWeight;
   /**
    * Default weight for holding the neck at the robot initial configuration when no neck message is received.
    */
   private double defaultNeckMessageWeight;
   /**
    * Default weight for holding the pelvis at the robot initial configuration when no pelvis message is received.
    */
   private final Vector3D defaultPelvisMessageLinearWeight = new Vector3D();
   /**
    * Default weight for holding the pelvis at the robot initial configuration when no pelvis message is received.
    */
   private final Vector3D defaultPelvisMessageAngularWeight = new Vector3D();
   /**
    * Default weight for holding the chest at the robot initial configuration when no chest message is received.
    */
   private final Vector3D defaultChestMessageAngularWeight = new Vector3D();

   /**
    * Default weight for locking the pelvis at the robot initial configuration when no pelvis message is received.
    */
   private double defaultPelvisMessageLockWeight;
   /**
    * Default weight for locking the chest at the robot initial configuration when no chest message is received.
    */
   private double defaultChestMessageLockWeight;

   /**
    * Default weight for input messages for which no weight is provided.
    */
   private double defaultLinearWeight;
   /**
    * Default weight for input messages for which no weight is provided.
    */
   private double defaultAngularWeight;

   /**
    * Default rate limit for input messages for which no rate limit is provided.
    */
   private double defaultLinearRateLimit;
   /**
    * Default rate limit for input messages for which no rate limit is provided.
    */
   private double defaultAngularRateLimit;
   /**
    * Scale factor used to downscale the joint velocity solution before sending it to the controller.
    */
   private double outputJointVelocityScale;
   /**
    * Break frequency used for the low-pass filter used to filter the output of the IK solver.
    */
   private double outputLPFBreakFrequency;

   /**
    * Whether to minimize the angular momentum in the kinematics solution.
    * Useful to reduce fast motions with the parts of the robot and instead privilege the motion of the extremities.
    */
   private boolean minimizeAngularMomentum;
   /**
    * Whether to minimize the linear momentum in the kinematics solution.
    * Useful to reduce fast motions with the parts of the robot and instead privilege the motion of the extremities.
    */
   private boolean minimizeLinearMomentum;
   /**
    * Weight used to minimize the angular momentum in the kinematics solution.
    */
   private double angularMomentumWeight;
   /**
    * Weight used to minimize the linear momentum in the kinematics solution.
    */
   private double linearMomentumWeight;
   /**
    * Whether to minimize the rate of change of the angular momentum in the kinematics solution.
    */
   private boolean minimizeAngularMomentumRate;
   /**
    * Whether to minimize the rate of change of the linear momentum in the kinematics solution.
    */
   private boolean minimizeLinearMomentumRate;
   /**
    * Weight used to minimize the rate of change of the angular momentum in the kinematics solution.
    */
   private double angularMomentumRateWeight;
   /**
    * Weight used to minimize the rate of change of the linear momentum in the kinematics solution.
    */
   private double linearMomentumRateWeight;
   /**
    * Duration used to smoothly initiate the streaming to the controller.
    */
   private double defaultStreamingBlendingDuration;

   /**
    * Break frequency used for the low-pass filter used to estimate the input pose.
    * Only used when {@link #inputStateEstimatorType} is set to {@link InputStateEstimatorType#FIRST_ORDER_LPF}.
    */
   private double inputPoseLPFBreakFrequency;
   /**
    * Duration use to decay the estimated input weight.
    * When no input is received, the weight is decayed to zero, then the control for that end-effector is disabled.
    */
   private double inputWeightDecayDuration;
   /**
    * Duration use to decay the estimated input velocity.
    * Safety used to limit the extrapolation in the future when inputs are not being received.
    */
   private double inputVelocityDecayDuration;
   /**
    * Duration used to correct the input pose in the state estimator.
    * Should be greater than the period at which inputs are received.
    * Only used when {@link #inputStateEstimatorType} is set to {@link InputStateEstimatorType#FBC_STYLE}.
    */
   private double inputPoseCorrectionDuration;
   /**
    * When {@code true}, a bounding box filter is used to reject input poses that are too far from the current pose.
    * The bottom center of the bounding box is set to line up with the robot's mid-foot z-up frame.
    */
   private boolean useBBXInputFilter;
   /**
    * Size of the bounding box filter used to reject input poses that are too far from the current pose.
    */
   private Vector3D inputFilterBBXSize;
   /**
    * Center of the bounding box with respect to the robot's mid-foot z-up frame.
    */
   private Point3D inputFilterBBXCenter;
   private boolean useStreamingPublisher;
   private double publishingPeriod;

   private InputStateEstimatorType inputStateEstimatorType;

   /**
    * Map from joint name to initial 1-DoF joint position.
    */
   private Map<String, Double> initialConfigurationMap;
   private final KinematicsStreamingToolboxConfigurationMessage defaultConfiguration = new KinematicsStreamingToolboxConfigurationMessage();
   private final KinematicsToolboxConfigurationMessage defaultSolverConfiguration = new KinematicsToolboxConfigurationMessage();

   public static KinematicsStreamingToolboxParameters defaultParameters()
   {
      KinematicsStreamingToolboxParameters parameters = new KinematicsStreamingToolboxParameters();
      parameters.setDefault();
      return parameters;
   }

   public void setDefault()
   {
      clockType = ClockType.CPU_CLOCK;
      toolboxUpdatePeriod = 0.005;
      timeThresholdForSleeping = 3.0;
      streamIntegrationDuration = 0.3;

      centerOfMassSafeMargin = 0.05;
      centerOfMassHoldWeight = 0.001;
      publishingSolutionPeriod = UnitConversions.hertzToSeconds(60.0);
      defaultArmMessageWeight = 10.0;
      defaultNeckMessageWeight = 10.0;
      defaultPelvisMessageLinearWeight.set(2.5, 2.5, 2.5);
      defaultPelvisMessageAngularWeight.set(1.0, 1.0, 1.0);
      defaultChestMessageAngularWeight.set(0.75, 0.75, 0.75);

      defaultPelvisMessageLockWeight = 1000.0;
      defaultChestMessageLockWeight = 1000.0;

      defaultLinearWeight = 20.0;
      defaultAngularWeight = 1.0;

      defaultLinearRateLimit = 1.5;
      defaultAngularRateLimit = 10.0;
      outputJointVelocityScale = 0.75;
      outputLPFBreakFrequency = Double.POSITIVE_INFINITY;

      minimizeAngularMomentum = true;
      minimizeLinearMomentum = false;
      angularMomentumWeight = 0.125;
      linearMomentumWeight = 0.0;

      minimizeAngularMomentumRate = false;
      minimizeLinearMomentumRate = false;
      angularMomentumRateWeight = 0.0;
      linearMomentumRateWeight = 0.0;

      defaultStreamingBlendingDuration = 2.0;

      inputPoseLPFBreakFrequency = 4.0;
      inputWeightDecayDuration = 3.0;
      inputVelocityDecayDuration = 0.5;
      inputPoseCorrectionDuration = 0.15;
      useBBXInputFilter = false;

      useStreamingPublisher = true;
      publishingPeriod = 5.0 * 0.006;

      inputStateEstimatorType = InputStateEstimatorType.FIRST_ORDER_LPF;

      defaultConfiguration.setLockPelvis(false);
      defaultConfiguration.setLockChest(false);
      defaultConfiguration.setEnableLeftArmJointspace(true);
      defaultConfiguration.setEnableRightArmJointspace(true);
      defaultConfiguration.setEnableNeckJointspace(true);
      defaultConfiguration.setEnableLeftHandTaskspace(true);
      defaultConfiguration.setEnableRightHandTaskspace(true);
      defaultConfiguration.setEnableChestTaskspace(true);
      defaultConfiguration.setEnablePelvisTaskspace(true);
      defaultConfiguration.setLeftHandTrajectoryFrameId(ReferenceFrame.getWorldFrame().getFrameNameHashCode());
      defaultConfiguration.setRightHandTrajectoryFrameId(ReferenceFrame.getWorldFrame().getFrameNameHashCode());
      defaultConfiguration.setChestTrajectoryFrameId(ReferenceFrame.getWorldFrame().getFrameNameHashCode());
      defaultConfiguration.setPelvisTrajectoryFrameId(ReferenceFrame.getWorldFrame().getFrameNameHashCode());

      defaultSolverConfiguration.setJointVelocityWeight(1.0);
      defaultSolverConfiguration.setEnableJointVelocityLimits(true);
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

   public double getPublishingSolutionPeriod()
   {
      return publishingSolutionPeriod;
   }

   public double getDefaultArmMessageWeight()
   {
      return defaultArmMessageWeight;
   }

   public double getDefaultNeckMessageWeight()
   {
      return defaultNeckMessageWeight;
   }

   public Vector3D getDefaultPelvisMessageLinearWeight()
   {
      return defaultPelvisMessageLinearWeight;
   }

   public Vector3D getDefaultPelvisMessageAngularWeight()
   {
      return defaultPelvisMessageAngularWeight;
   }

   public Vector3D getDefaultChestMessageAngularWeight()
   {
      return defaultChestMessageAngularWeight;
   }

   public double getDefaultPelvisMessageLockWeight()
   {
      return defaultPelvisMessageLockWeight;
   }

   public double getDefaultChestMessageLockWeight()
   {
      return defaultChestMessageLockWeight;
   }

   public double getDefaultLinearWeight()
   {
      return defaultLinearWeight;
   }

   public double getDefaultAngularWeight()
   {
      return defaultAngularWeight;
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

   public double getAngularMomentumWeight()
   {
      return angularMomentumWeight;
   }

   public double getLinearMomentumWeight()
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

   public double getAngularMomentumRateWeight()
   {
      return angularMomentumRateWeight;
   }

   public double getLinearMomentumRateWeight()
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

   public void setCenterOfMassHoldWeight(double centerOfMassHoldWeight)
   {
      this.centerOfMassHoldWeight = centerOfMassHoldWeight;
   }

   public void setPublishingSolutionPeriod(double publishingSolutionPeriod)
   {
      this.publishingSolutionPeriod = publishingSolutionPeriod;
   }

   public void setDefaultArmMessageWeight(double defaultArmMessageWeight)
   {
      this.defaultArmMessageWeight = defaultArmMessageWeight;
   }

   public void setDefaultNeckMessageWeight(double defaultNeckMessageWeight)
   {
      this.defaultNeckMessageWeight = defaultNeckMessageWeight;
   }

   public void setDefaultPelvisMessageLinearWeight(double xWeight, double yWeight, double zWeight)
   {
      this.defaultPelvisMessageLinearWeight.set(xWeight, yWeight, zWeight);
   }

   public void setDefaultPelvisMessageAngularWeight(double xWeight, double yWeight, double zWeight)
   {
      this.defaultPelvisMessageAngularWeight.set(xWeight, yWeight, zWeight);
   }

   public void setDefaultChestMessageAngularWeight(double xWeight, double yWeight, double zWeight)
   {
      this.defaultChestMessageAngularWeight.set(xWeight, yWeight, zWeight);
   }

   public void setDefaultPelvisMessageLockWeight(double defaultPelvisMessageLockWeight)
   {
      this.defaultPelvisMessageLockWeight = defaultPelvisMessageLockWeight;
   }

   public void setDefaultChestMessageLockWeight(double defaultChestMessageLockWeight)
   {
      this.defaultChestMessageLockWeight = defaultChestMessageLockWeight;
   }

   public void setDefaultLinearWeight(double defaultLinearWeight)
   {
      this.defaultLinearWeight = defaultLinearWeight;
   }

   public void setDefaultAngularWeight(double defaultAngularWeight)
   {
      this.defaultAngularWeight = defaultAngularWeight;
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
      this.angularMomentumWeight = angularMomentumWeight;
   }

   public void setLinearMomentumWeight(double linearMomentumWeight)
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
      this.angularMomentumRateWeight = angularMomentumRateWeight;
   }

   public void setLinearMomentumRateWeight(double linearMomentumRateWeight)
   {
      this.linearMomentumRateWeight = linearMomentumRateWeight;
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

   public void setInitialConfigurationMap(Map<String, Double> initialConfigurationMap)
   {
      this.initialConfigurationMap = initialConfigurationMap;
   }
}

package us.ihmc.avatar.networkProcessor.footstepStreamingModule;

public class FootstepStreamingToolboxParameters
{
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
    * Option to compute the target robot footstep with respect to the stance foot (true)
    * or from the initial swing pose (false)
    */
   private boolean computeFromStance;
   /**
    * Default movement threshold to determine when the user is actively stepping
    */
   private double stepThreshold;
   /**
    * Default step height threshold to determine when the user is actively stepping
    */
   private double liftThreshold;
   /**
    * Default margin time before robot concludes step, when we want to stop sending update footsteps to the controller
    */
   private double footstepMarginTime;
   /**
    * Default stride length to use as a starting guess
    */
   private double defaultStride;
   /**
    * Maximum stride length
    */
   private double maxStride;
   private double maxDistanceToStance;
   private double minDistanceToStance;
   /**
    * Proportional gain for direction control
    */
   private double kpDirection;
   /**
    * Proportional gain for stride control
    */
   private double kpStride;
   /**
    * Scaling factor for horizontal linear velocity of trackers used to estimate the final stride of the step.
    * Takes into account the different rapidity of motion between human and robot
    */
   private double strideVelocityScalingFactor;
   /**
    * Proportional gain for yaw rotation control
    */
   private double kpYaw;
   /**
    * Scaling factor for angular velocity of trackers used to estimate the final yaw of the step.
    * Takes into account the different rapidity of motion between human and robot
    */
   private double yawVelocityScalingFactor;
   /**
    * Default variation in degrees that triggers the turn of the robot step
    */
   private double turningThreshold;
   /**
    * Default turn of the robot step to use as a starting guess
    */
   private double turnDegrees;
   /**
    * Maximum yaw rotation
    */
   private double maxYawRotationDegrees;
   private double maxYawToStanceDegrees;
   private double minYawToStanceDegrees;

   /**
    * Default stability threshold to determine when the user has stopped stepping with a given foot.
    */
   private double stabilityThreshold;
   /**
    * Default number of iterations required to assess stability
    */
   private int stabilityIterations;

   public static FootstepStreamingToolboxParameters defaultParameters()
   {
      FootstepStreamingToolboxParameters parameters = new FootstepStreamingToolboxParameters();
      parameters.setDefault();
      return parameters;
   }

   public void setDefault()
   {
      clockType = ClockType.CPU_CLOCK;
      toolboxUpdatePeriod = 0.001;
      timeThresholdForSleeping = 3.0;

      computeFromStance = true;
      footstepMarginTime = 0.1;

      // Step threshold of 5cm and lift of 2cm, seem to be too conservative. Step is identified after ~0.3s. Too long considering that a step lasts ~0.57s
      // Step threshold of 2cm and lift of 1cm seem to work great!
      stepThreshold = 0.02;
      liftThreshold = 0.02;

      // These stability parameters seem to work great!
      stabilityThreshold = 0.01;
      stabilityIterations = 3;

      defaultStride = 0.20;
      maxStride = 0.65;
      kpDirection = 0.0;
      kpStride = 0.5;
      strideVelocityScalingFactor = 0.8;

      turningThreshold = 20;
      turnDegrees = 33.3;
      maxYawRotationDegrees = 90.0;
      kpYaw = 0.5;
      yawVelocityScalingFactor = 0.5;

      minDistanceToStance = 0.18;
      maxDistanceToStance = 0.6;
      minYawToStanceDegrees = 0.0;
      maxYawToStanceDegrees = 90.0;
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

   public boolean getComputeFromStance()
   {
      return computeFromStance;
   }

   public double getStepThreshold()
   {
      return stepThreshold;
   }

   public double getLiftThreshold()
   {
      return liftThreshold;
   }

   public double getDefaultStride()
   {
      return defaultStride;
   }

   public double getMaxStride()
   {
      return maxStride;
   }

   public double getKpDirection()
   {
      return kpDirection;
   }

   public double getKpStride()
   {
      return kpStride;
   }

   public double getStrideVelocityScalingFactor()
   {
      return strideVelocityScalingFactor;
   }

   public double getKpYaw()
   {
      return kpYaw;
   }

   public double getYawVelocityScalingFactor()
   {
      return yawVelocityScalingFactor;
   }

   public double getTurningThresholdDegrees()
   {
      return turningThreshold;
   }

   public double getTurnDegrees()
   {
      return turnDegrees;
   }

   public double getMaxYawRotationDegrees()
   {
      return maxYawRotationDegrees;
   }

   public double getMaxDistanceToStance()
   {
      return maxDistanceToStance;
   }

   public double getMinDistanceToStance()
   {
      return minDistanceToStance;
   }

   public double getMaxYawToStanceDegrees()
   {
      return maxYawToStanceDegrees;
   }

   public double getMinYawToStanceDegrees()
   {
      return minYawToStanceDegrees;
   }

   public int getStabilityIterations()
   {
      return stabilityIterations;
   }

   public double getStabilityThreshold()
   {
      return stabilityThreshold;
   }

   public double getFootstepMarginTime()
   {
      return footstepMarginTime;
   }
}

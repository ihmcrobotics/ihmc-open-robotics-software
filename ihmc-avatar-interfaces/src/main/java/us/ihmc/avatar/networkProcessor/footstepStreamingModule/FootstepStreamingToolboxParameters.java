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
    * Default movement threshold to determine when the user is actively stepping
    */
   private double stepThreshold;
   /**
    * Default step height threshold to determine when the user is actively stepping
    */
   private double liftThreshold;
   /**
    * Default stride length to use as a starting guess
    */
   private double strideLength;
   /**
    * Default proportional gain for direction control
    */
   private double kpDirection;
   /**
    * Default variation in degrees that triggers the turn of the robot step
    */
   private double turningThreshold;
   /**
    * Default turn of the robot step to use as a starting guess
    */
   private double turnDegrees;
   /**
    * Default stability threshold to determine when the user has stopped stepping with a given foot.
    */
   private double stabilityThreshold;
   /**
    * Default number of iterations required to assess stability
    */
   private int stabilityIterations;
   /**
    * Default acceleration norm threshold, to consider the swing foot is not decelerating
    */
   private double accelerationThreshold;
   /**
    * Default weight for stride length estimation related to a fraction of horizontal tracker acceleration
    */
   private double horizontalAccelerationWeight;
   /**
    * Default weight for stride length estimation related to a fraction of vertical tracker position
    */
   private double verticalComponentWeight;


   private double publishingPeriod;

   public static FootstepStreamingToolboxParameters defaultParameters()
   {
      FootstepStreamingToolboxParameters parameters = new FootstepStreamingToolboxParameters();
      parameters.setDefault();
      return parameters;
   }

   public void setDefault()
   {
      clockType = ClockType.CPU_CLOCK;
      toolboxUpdatePeriod = 0.005;
      timeThresholdForSleeping = 3.0;

      stepThreshold = 0.05;
      liftThreshold = 0.02;
      strideLength = 0.20;
      kpDirection = 0.5;

      turningThreshold = 12;
      turnDegrees = 33.3;

      stabilityThreshold = 0.005;
      stabilityIterations = 20;

      publishingPeriod = 5.0 * 0.006;
      accelerationThreshold = 0.0;

      horizontalAccelerationWeight = 0.1;
      verticalComponentWeight = 0.1;
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

   public double getPublishingPeriod()
   {
      return publishingPeriod;
   }

   public double getStepThreshold()
   {
      return stepThreshold;
   }

   public double getLiftThreshold()
   {
      return liftThreshold;
   }

   public double getStrideLength()
   {
      return strideLength;
   }

   public double getKpDirection()
   {
      return kpDirection;
   }

   public double getTurningThreshold()
   {
      return turningThreshold;
   }

   public double getTurnDegrees()
   {
      return turnDegrees;
   }

   public int getStabilityIterations()
   {
      return stabilityIterations;
   }

   public double getStabilityThreshold()
   {
      return stabilityThreshold;
   }

   public double getAccelerationThreshold()
   {
      return accelerationThreshold;
   }

   public double getHorizontalAccelerationWeight()
   {
      return horizontalAccelerationWeight;
   }

   public double getVerticalComponentWeight()
   {
      return verticalComponentWeight;
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

   public void setStepThreshold(double stepThreshold)
   {
      this.stepThreshold = stepThreshold;
   }

   public void setLiftThreshold(double liftThreshold)
   {
      this.liftThreshold = liftThreshold;
   }

   public void setStrideLength(double strideLength)
   {
      this.strideLength = strideLength;
   }

   public void setKpDirection(double kpDirection)
   {
      this.kpDirection = kpDirection;
   }

   public void setTurningThreshold(double turningThreshold)
   {
      this.turningThreshold = turningThreshold;
   }

   public void setTurnDegrees(double turnDegrees)
   {
      this.turnDegrees = turnDegrees;
   }

   public void setStabilityIterations(int stabilityIterations)
   {
      this.stabilityIterations = stabilityIterations;
   }

   public void setStabilityThreshold(double stabilityThreshold)
   {
      this.stabilityThreshold = stabilityThreshold;
   }

   public void setPublishingPeriod(double publishingPeriod)
   {
      this.publishingPeriod = publishingPeriod;
   }

   public void setAccelerationThreshold(double accelerationThreshold)
   {
      this.accelerationThreshold = accelerationThreshold;
   }

   public void setHorizontalAccelerationWeight(double horizontalAccelerationWeight)
   {
      this.horizontalAccelerationWeight = horizontalAccelerationWeight;
   }

   public void setVerticalComponentWeight(double verticalComponentWeight)
   {
      this.verticalComponentWeight = verticalComponentWeight;
   }
}

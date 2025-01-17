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
    * Default velocity norm threshold, to consider when the swing foot is not moving much anymore
    */
   private double velocityThreshold;
   /**
   /**
    * Default acceleration threshold, to consider when the swing foot is decelerating
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

      computeFromStance = false;

      // Step threshold of 5cm and lift of 2cm, seem to be too conservative. Step is identified after ~0.3s. Too long considering that a step lasts ~0.57s
      // Step threshold of 2cm and lift of 1cm seem to work great!
      stepThreshold = 0.02;
      liftThreshold = 0.01;

      // These stability parameters seem to work great!
      stabilityThreshold = 0.005;
      stabilityIterations = 5;

      strideLength = 0.20;
      kpDirection = 0.0;

      // TODO. tune these ones below
      turningThreshold = 12;
      turnDegrees = 33.3;

      velocityThreshold = 0.05;
      accelerationThreshold = 0.02;

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

   public double getVelocityThreshold()
   {
      return velocityThreshold;
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

   public void setVelocityThreshold(double velocityThreshold)
   {
      this.velocityThreshold = velocityThreshold;
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

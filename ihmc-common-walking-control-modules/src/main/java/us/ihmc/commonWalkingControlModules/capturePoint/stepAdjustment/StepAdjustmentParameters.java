package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

public abstract class StepAdjustmentParameters
{
   /**
    * Specifies the amount of ICP error (the 2D distance in XY from desired to current) that is required for the controller to consider step adjustment.
    */
   public double getMinICPErrorForStepAdjustment()
   {
      return 0.0;
   }

   /**
    * Enabling this boolean enables the use step adjustment for stabilization.
    */
   public abstract boolean allowStepAdjustment();

   /**
    * Enabling this boolean enables the use of crossover steps when performing step adjustment.
    */
   public boolean allowCrossOverSteps()
   {
      return false;
   }

   /**
    * Deadband on the step adjustment.
    * When the adjustment is within the deadband, it is set to zero.
    * When it is outside the deadband, the deadband is subtracted from it.
    */
   public abstract double getAdjustmentDeadband();

   /**
    * Specifies the minimum time remaining for the controller to allow step adjustment. Once the time remaining goes below this value, the step position in
    * the world is frozen.
    */
   public double getMinimumTimeForStepAdjustment()
   {
      return 0.02;
   }

   /**
    * Specifies the distance from the front of the foot that the Cop has to be when computing the capture region. This effectively shrinks the foot.
    */
   public double getCoPDistanceFromFrontOfFoot()
   {
      return 0.02;
   }

   /**
    * Specifies the distance from the back of the foot that the Cop has to be when computing the capture region. This effectively shrinks the foot.
    */
   public double getCoPDistanceFromBackOfFoot()
   {
      return 0.05;
   }

   /**
    * Specifies the distance from the inside of the foot that the Cop has to be when computing the capture region. This effectively shrinks the foot.
    */
   public double getCoPDistanceFromInsideOfFoot()
   {
      return 0.01;
   }

   /**
    * Specifies the distance from the outside of the foot that the Cop has to be when computing the capture region. This effectively shrinks the foot.
    */
   public double getCoPDistanceFromOutsideOfFoot()
   {
      return 0.03;
   }

   /**
    * Returns the paramters for determining the cross over reachability area. This calculation is coupled with the
    * {@link us.ihmc.commonWalkingControlModules.configurations.SteppingParameters} to determine where the robot is allowed to step.
    */
   public CrossOverReachabilityParameters getCrossOverReachabilityParameters()
   {
      return new CrossOverReachabilityParameters();
   }

   public static class CrossOverReachabilityParameters
   {
      /**
       * This is the maximum distance to the outside of the stance foot that the step adjustment can go if stepping in front of the stance foot.
       */
      public double getForwardCrossOverDistance()
      {
         return 0.0;
      }

      /**
       * This is the cut-off angle for the area that the foot can step in front of the stance foot. The angle is from the "nominal width" position to the
       * stance foot.
       */
      public double getForwardCrossOverClearanceAngle()
      {
         return Math.toRadians(35.0);
      }

      /**
       * This is the maximum distance to the outside of the stance foot that the step adjustment can go if stepping behind the stance foot.
       */
      public double getBackwardCrossOverDistance()
      {
         return -0.05;
      }

      /**
       * This is the cut-off angle for the area that the foot can step behind the stance foot. The angle is from the "nominal width" position to the
       * stance foot.
       */
      public double getBackwardCrossOverClearanceAngle()
      {
         return Math.toRadians(45.0);
      }
   }
}

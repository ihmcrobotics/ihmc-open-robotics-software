package us.ihmc.robotics.controllers;

/**
 * Defines parameters to use in the EuclideanTangentialDampingCalculator. This reduces the damping ratio when incurring large tracking errors.
 */
public interface TangentialDampingGains
{
   /**
    * @param kdReductionRatio minimum ratio of the normal Kd that you would like it to be reduced to when incurring large tracking error.
    * @param parallelDampingDeadband deadband on position error outside of which you would like the derivative gain to start being reduced.
    * @param positionErrorForMinimumKd position at which you would like to have a achieved the most reduction of the derivative gain.
    */
   public void set(double kdReductionRatio, double parallelDampingDeadband, double positionErrorForMinimumKd);

   /**
    * @return minimum ratio of the normal Kd that you would like it to be reduced to when incurring large tracking error.
    */
   public double getKdReductionRatio();

   /**
    * @return deadband on position error outside of which you would like the derivative gain to start being reduced.
    */
   public double getParallelDampingDeadband();

   /**
    * @return position at which you would like to have a achieved the most reduction of the derivative gain..
    */
   public double getPositionErrorForMinimumKd();
}

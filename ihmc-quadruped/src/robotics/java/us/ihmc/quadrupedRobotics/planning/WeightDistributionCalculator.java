package us.ihmc.quadrupedRobotics.planning;

public interface WeightDistributionCalculator
{
   double getFractionOfWeightForward(double pitchAngle);
}

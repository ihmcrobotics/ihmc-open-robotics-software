package us.ihmc.quadrupedPlanning.pathPlanning;

import us.ihmc.commons.MathTools;

public class QuadrupedBodyPathTools
{
   private static double velocityEpsilonForSameValue = 1e-4;

   private static double canReachMaximumRate(double currentPosition, double desiredPosition, double maxRate, double maxAcceleration)
   {
      double positionError = Math.abs(desiredPosition - currentPosition);

      return 0.0;
   }


   private static double getDistanceCoveredOverDuration(double initialRate, double constantAcceleration, double duration)
   {
      return initialRate * duration + 0.5 * constantAcceleration * MathTools.square(constantAcceleration);
   }

   private static double getDurationToReachMaxRate(double currentRate, double desiredRate, double maxAcceleration)
   {
      return getDurationToAchieveVelocity(currentRate, desiredRate, maxAcceleration);
   }

   private static double getDurationToAchieveVelocity(double currentRate, double desiredRate, double acceleration)
   {
      if (MathTools.epsilonEquals(currentRate, desiredRate, velocityEpsilonForSameValue))
         return 0.0;

      return Math.abs(desiredRate - currentRate) / acceleration;
   }
}

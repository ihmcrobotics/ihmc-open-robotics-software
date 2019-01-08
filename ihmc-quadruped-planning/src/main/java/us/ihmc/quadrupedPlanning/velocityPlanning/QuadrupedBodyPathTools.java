package us.ihmc.quadrupedPlanning.velocityPlanning;

import us.ihmc.commons.MathTools;

public class QuadrupedBodyPathTools
{
   private static double positionEpsilonForSameValue = 1e-2;
   static double yawEpsilonForSameValue = 1e-2;
   private static double velocityEpsilonForSameValue = 1e-4;

   public static double computeTimeToAchieveHeading(double currentYaw, double currentYawRate, double desiredYaw, double desiredYawRate, double maxRate,
                                                    double maxAcceleration)
   {
      double angleDelta = desiredYaw - currentYaw;
      double accelerationSign = Math.signum(angleDelta);
      double timeToAccelerateWithNoMaxVelocity = computeTimeToAccelerateToAchieveValueWithNoMaxRate(currentYaw, currentYawRate, desiredYaw, desiredYawRate,
                                                                                                    maxAcceleration);
      double maxVelocity = currentYawRate + accelerationSign * maxAcceleration * timeToAccelerateWithNoMaxVelocity;

      if (Math.abs(maxVelocity) > maxRate)
      {
         double timeToAccelerate = (maxRate - Math.abs(currentYawRate)) / maxAcceleration;
         double distanceWhileAccelerating = currentYawRate * timeToAccelerate + 0.5 * accelerationSign * maxAcceleration * MathTools.square(timeToAccelerate);
         double timeToDecelerate = Math.abs(accelerationSign - desiredYawRate) / maxAcceleration;
         double distanceWhileDecelerating =
               accelerationSign * maxRate * timeToDecelerate - 0.5 * accelerationSign * maxAcceleration * MathTools.square(timeToDecelerate);

         double timeAtConstantVelocity = (Math.abs(angleDelta) - distanceWhileAccelerating - distanceWhileDecelerating) / maxRate;

         return timeAtConstantVelocity + timeToAccelerate + timeToDecelerate;
      }
      else
      {
         double timeToDecelerate = Math.abs(maxVelocity - desiredYawRate) / maxAcceleration;

         return timeToAccelerateWithNoMaxVelocity + timeToDecelerate;
      }
   }

   public static double computeTimeToAccelerateToAchieveValueWithNoMaxRate(double currentValue, double currentRate, double desiredValue, double desiredRate,
                                                                           double maxAcceleration)
   {
      double angleDelta = desiredValue - currentValue;
      double motionDirection = Math.signum(desiredValue - currentValue);
      double acceleration = motionDirection * maxAcceleration;
      double velocityDelta = 1.0 / (2.0 * acceleration) * (MathTools.square(currentRate) - MathTools.square(desiredRate));
      double a = acceleration;
      double b = 2.0 * currentRate;
      double c = -angleDelta + velocityDelta;

      return largestQuadraticSolution(a, b, c);
   }

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

   static double largestQuadraticSolution(double a, double b, double c)
   {
      double radical = Math.sqrt(MathTools.square(b) - 4.0 * a * c);

      if (a > 0)
      {
         return (-b + radical) / (2.0 * a);
      }
      else
      {
         return (-b - radical) / (2.0 * a);
      }
   }

}

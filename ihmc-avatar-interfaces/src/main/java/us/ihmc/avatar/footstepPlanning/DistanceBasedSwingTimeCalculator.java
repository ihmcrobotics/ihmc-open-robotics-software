package us.ihmc.avatar.footstepPlanning;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Calculates a suggested swing time by linearly interpolating between two given data points as a function of the translation
 * between the start and end of a step
 * */
class DistanceBasedSwingTimeCalculator
{
   private final double minimumSwingTime;
   private final double maximumTranslationForMinimumSwingTime;

   private final double maximumSwingTime;
   private final double minimumTranslationForMaximumSwingTime;

   DistanceBasedSwingTimeCalculator(double minimumSwingTime, double maximumTranslationForMinimumSwingTime, double maximumSwingTime,
                                    double minimumTranslationForMaximumSwingTime)
   {
      this.minimumSwingTime = minimumSwingTime;
      this.maximumTranslationForMinimumSwingTime = maximumTranslationForMinimumSwingTime;
      this.maximumSwingTime = maximumSwingTime;
      this.minimumTranslationForMaximumSwingTime = minimumTranslationForMaximumSwingTime;
   }

   double calculateSwingTime(Point3DReadOnly startPosition, Point3DReadOnly endPosition)
   {
      double distance = startPosition.distance(endPosition);
      double alpha = (distance - maximumTranslationForMinimumSwingTime) / (minimumTranslationForMaximumSwingTime - maximumTranslationForMinimumSwingTime);
      alpha = EuclidCoreTools.clamp(alpha, 0.0, 1.0);
      return EuclidCoreTools.interpolate(minimumSwingTime, maximumSwingTime, alpha);
   }
}

package us.ihmc.avatar.footstepPlanning;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.AdaptiveSwingParameters;

/**
 * Calculates a suggested swing time and height by linearly interpolating between two given data points as a function of step poses
 * */
public class AdaptiveSwingTrajectoryCalculator
{
   private final AdaptiveSwingParameters adaptiveSwingParameters;

   public AdaptiveSwingTrajectoryCalculator(AdaptiveSwingParameters adaptiveSwingParameters)
   {
      this.adaptiveSwingParameters = adaptiveSwingParameters;
   }

   public double calculateSwingTime(Point3DReadOnly startPosition, Point3DReadOnly endPosition)
   {
      double horizontalDistance = startPosition.distanceXY(endPosition);
      double maximumTranslationForMinimumSwingTime = adaptiveSwingParameters.getMaximumStepTranslationForMinimumSwingTime();
      double minimumTranslationForMaximumSwingTime = adaptiveSwingParameters.getMinimumStepTranslationForMaximumSwingTime();
      double alphaHorizontal = (horizontalDistance - maximumTranslationForMinimumSwingTime) / (minimumTranslationForMaximumSwingTime - maximumTranslationForMinimumSwingTime);

      double verticalDistance = Math.abs(startPosition.getZ() - endPosition.getZ());
      double maximumStepHeightForMinimumSwingTime = adaptiveSwingParameters.getMaximumStepHeightForMinimumSwingTime();
      double minimumStepHeightForMaximumSwingTime = adaptiveSwingParameters.getMinimumStepHeightForMaximumSwingTime();
      double alphaVertical = (verticalDistance - maximumStepHeightForMinimumSwingTime) / (minimumStepHeightForMaximumSwingTime - maximumStepHeightForMinimumSwingTime);
      
      double alpha = EuclidCoreTools.clamp(alphaHorizontal + alphaVertical, 0.0, 1.0);
      return EuclidCoreTools.interpolate(adaptiveSwingParameters.getMinimumSwingTime(), adaptiveSwingParameters.getMaximumSwingTime(), alpha);
   }

   public double calculateSwingHeight(Point3DReadOnly startPosition, Point3DReadOnly endPosition)
   {
      double verticalDistance = Math.abs(startPosition.getZ() - endPosition.getZ());
      double maximumStepHeightForMinimumSwingHeight = adaptiveSwingParameters.getMaximumStepHeightForMinimumSwingHeight();
      double minimumStepHeightForMaximumSwingHeight = adaptiveSwingParameters.getMinimumStepHeightForMaximumSwingHeight();
      double alpha = (verticalDistance - maximumStepHeightForMinimumSwingHeight) / (minimumStepHeightForMaximumSwingHeight - maximumStepHeightForMinimumSwingHeight);
      alpha = EuclidCoreTools.clamp(alpha, 0.0, 1.0);
      
      return EuclidCoreTools.interpolate(adaptiveSwingParameters.getMinimumSwingHeight(), adaptiveSwingParameters.getMaximumSwingHeight(), alpha);
   }
}

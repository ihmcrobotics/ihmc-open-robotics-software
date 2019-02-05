package us.ihmc.avatar.footstepPlanning;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepProcessingParameters;

/**
 * Calculates a suggested swing time and height by linearly interpolating between two given data points as a function of step poses
 * */
class DistanceBasedSwingParameterCalculator
{
   private final FootstepProcessingParameters footstepProcessingParameters;

   DistanceBasedSwingParameterCalculator(FootstepProcessingParameters footstepProcessingParameters)
   {
      this.footstepProcessingParameters = footstepProcessingParameters;
   }

   double calculateSwingTime(Point3DReadOnly startPosition, Point3DReadOnly endPosition)
   {
      double horizontalDistance = startPosition.distanceXY(endPosition);
      double maximumTranslationForMinimumSwingTime = footstepProcessingParameters.getMaximumStepTranslationForMinimumSwingTime();
      double minimumTranslationForMaximumSwingTime = footstepProcessingParameters.getMinimumStepTranslationForMaximumSwingTime();
      double alphaHorizontal = (horizontalDistance - maximumTranslationForMinimumSwingTime) / (minimumTranslationForMaximumSwingTime - maximumTranslationForMinimumSwingTime);

      double verticalDistance = Math.abs(startPosition.getZ() - endPosition.getZ());
      double maximumStepHeightForMinimumSwingTime = footstepProcessingParameters.getMaximumStepHeightForMinimumSwingHeight();
      double minimumStepHeightForMaximumSwingTime = footstepProcessingParameters.getMinimumStepHeightForMaximumSwingHeight();
      double alphaVertical = (verticalDistance - maximumStepHeightForMinimumSwingTime) / (minimumStepHeightForMaximumSwingTime - maximumStepHeightForMinimumSwingTime);
      
      double alpha = EuclidCoreTools.clamp(alphaHorizontal + alphaVertical, 0.0, 1.0);
      return EuclidCoreTools.interpolate(footstepProcessingParameters.getMinimumSwingTime(), footstepProcessingParameters.getMaximumSwingTime(), alpha);
   }
   
   double calculateSwingHeight(Point3DReadOnly startPosition, Point3DReadOnly endPosition)
   {
      double verticalDistance = Math.abs(startPosition.getZ() - endPosition.getZ());
      double maximumStepHeightForMinimumSwingHeight = footstepProcessingParameters.getMaximumStepHeightForMinimumSwingHeight();
      double minimumStepHeightForMaximumSwingHeight = footstepProcessingParameters.getMinimumStepHeightForMaximumSwingHeight();
      double alpha = (verticalDistance - maximumStepHeightForMinimumSwingHeight) / (minimumStepHeightForMaximumSwingHeight - maximumStepHeightForMinimumSwingHeight);
      alpha = EuclidCoreTools.clamp(alpha, 0.0, 1.0);
      
      return EuclidCoreTools.interpolate(footstepProcessingParameters.getMinimumSwingHeight(), footstepProcessingParameters.getMaximumSwingHeight(), alpha);      
   }
}

package us.ihmc.footstepPlanning;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface AStarBodyPathPlannerParametersBasics extends AStarBodyPathPlannerParametersReadOnly, StoredPropertySetBasics
{
   default void setRollCostWeight(double rollCostWeight)
   {
      set(AStarBodyPathPlannerParameters.rollCostWeight, rollCostWeight);
   }

   default void setRollCostDeadband(double rollCostDeadband)
   {
      set(AStarBodyPathPlannerParameters.rollCostDeadband, rollCostDeadband);
   }

   default void setMaxPenalizedRollAngle(double maxPenalizedRollAngle)
   {
      set(AStarBodyPathPlannerParameters.maxPenalizedRollAngle, maxPenalizedRollAngle);
   }

   default void setInclineCostWeight(double inclineCostWeight)
   {
      set(AStarBodyPathPlannerParameters.inclineCostWeight, inclineCostWeight);
   }

   default void setInclineCostDeadband(double inclineCostDeadband)
   {
      set(AStarBodyPathPlannerParameters.inclineCostDeadband, inclineCostDeadband);
   }

   default void setMaxIncline(double maxIncline)
   {
      set(AStarBodyPathPlannerParameters.maxIncline, maxIncline);
   }

   default void setCheckForCollisions(boolean checkForCollisions)
   {
      set(AStarBodyPathPlannerParameters.checkForCollisions, checkForCollisions);
   }

   default void setComputeSurfaceNormalCost(boolean computeSurfaceNormalCost)
   {
      set(AStarBodyPathPlannerParameters.computeSurfaceNormalCost, computeSurfaceNormalCost);
   }

   default void setPerformSmoothing(boolean performSmoothing)
   {
      set(AStarBodyPathPlannerParameters.performSmoothing, performSmoothing);
   }

   default void setTraversibilityWeight(double traversibilityWeight)
   {
      set(AStarBodyPathPlannerParameters.traversibilityWeight, traversibilityWeight);
   }
}
